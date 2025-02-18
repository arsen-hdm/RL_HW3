// Copyright  (C)  2007  Francois Cauwe <francois at cauwe dot org> 
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
 
#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"

#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"

#include <Eigen/Geometry>
 
using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;

class Iiwa_pub_sub : public rclcpp::Node
{
    public:
        Iiwa_pub_sub()
        : Node("ros2_kdl_node"), node_handle_(std::shared_ptr<Iiwa_pub_sub>(this))
        {
            // declare cmd_interface parameter (position, velocity)
            declare_parameter("cmd_interface", "velocity"); // defaults to position
            get_parameter("cmd_interface", cmd_interface_);
            declare_parameter("task", "positioning"); // Default is positioning
            get_parameter("task", task_);
            
            RCLCPP_INFO(get_logger(),"Current cmd interface is: '%s'", cmd_interface_.c_str());

            if (!(cmd_interface_ == "position" || cmd_interface_ == "velocity" || cmd_interface_ == "effort" ))
            {
                RCLCPP_ERROR(get_logger(),"Selected cmd interface is not valid! Use 'position', 'velocity' or 'effort' instead...");
                return;
            }
            
            RCLCPP_INFO(get_logger(), "Current task is: '%s'", task_.c_str());

            // Validate the task parameter
            if (!(task_ == "positioning" || task_ == "look-at-point"))
            {
                RCLCPP_ERROR(get_logger(), "Selected task is not valid! Use 'positioning' or 'look-at-point' instead...");
                return;
            }

            iteration_ = 0; t_ = 0;
            joint_state_available_ = false;
            aruco_pose_available_ = false;

            // retrieve robot_description param
            auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node_handle_, "robot_state_publisher");
            while (!parameters_client->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    rclcpp::shutdown();
                }
                RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
            }
            auto parameter = parameters_client->get_parameters({"robot_description"});

            // create KDLrobot structure
            KDL::Tree robot_tree;
            if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree)){
                std::cout << "Failed to retrieve robot_description param!";
            }
            robot_ = std::make_shared<KDLRobot>(robot_tree);  
            
            // Create joint array
            unsigned int nj = robot_->getNrJnts();
            KDL::JntArray q_min(nj), q_max(nj);
            q_min.data << -2.96,-2.09,-2.96,-2.09,-2.96,-2.09,-2.96; //-2*M_PI,-2*M_PI; // TODO: read from urdf file
            q_max.data <<  2.96,2.09,2.96,2.09,2.96,2.09,2.96; //2*M_PI, 2*M_PI; // TODO: read from urdf file          
            robot_->setJntLimits(q_min,q_max);            
            joint_positions_.resize(nj); 
            joint_velocities_.resize(nj); 
            joint_positions_cmd_.resize(nj); 
            joint_velocities_cmd_.resize(nj);
            joint_accelerations_cmd_.resize(nj);
            joint_efforts_cmd_.resize(nj);

            // Subscriber to jnt states
            jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, std::bind(&Iiwa_pub_sub::joint_state_subscriber, this, std::placeholders::_1));
                
            // Subscribe to the ArUco marker pose
            aruco_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/aruco_pose", 10, std::bind(&Iiwa_pub_sub::aruco_pose_subscriber, this, std::placeholders::_1));

            // Wait for the joint_state topic
            while(!joint_state_available_){
                RCLCPP_INFO(this->get_logger(), "No data received yet! ...");
                rclcpp::spin_some(node_handle_);
            }

            // Update KDLrobot object
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
            KDL::Frame f_T_ee = KDL::Frame::Identity();
            robot_->addEE(f_T_ee);
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

            // Compute EE frame
            init_cart_pose_ = robot_->getEEFrame();

            // Compute IK
            KDL::JntArray q(nj);
            robot_->getInverseKinematics(init_cart_pose_, q);

            // Initialize controller with the robot model
            controller_ = std::make_shared<KDLController>(*robot_);

            // EE's trajectory initial position
            Eigen::Vector3d init_position(Eigen::Vector3d(init_cart_pose_.p.data));
            //std::cout << "The initial EE pose is: " << std::endl;  
            //std::cout << init_position <<std::endl;

            // Final trajectory EE position
            Eigen::Vector3d end_position;
            KDL::Vector pos;
            
            // Set end_position based on the task
            if (task_ == "positioning")
            {
                if (aruco_pose_available)
                {
                    pos = aruco_pose_.p();
                    end_position.x() = pos.x();
                    end_position.y() = pos.y() + 0.1;
                    end_position.z() = pos.z();
                }
            }
            else
            {
                end_position << init_position[0], -init_position[1], init_position[2];
            }
            
            // Define initial orientation
            Eigen::Quaterniond orientationInit = Eigen::Quaterniond::Identity();

            // Define final orientation
            double roll = 0.0;
            double pitch = -2.2; // Offset
            double yaw = 0.0;

            Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

            Eigen::Quaterniond orientationEnd = yawAngle * pitchAngle * rollAngle;            

            // Plan trajectory
            double traj_duration = 3, acc_duration = 1, t = 0.0;
            planner_ = KDLPlanner(traj_duration, acc_duration, init_position, end_position, orientationInit, orientationEnd);

            // Retrieve the first trajectory point
            trajectory_point p = planner_.compute_trajectory(t);
              
            // compute errors
            Eigen::Vector3d error = computeLinearError(p.pos, Eigen::Vector3d(init_cart_pose_.p.data));
            //std::cout << "The initial error is : " << error << std::endl;
            
            if(cmd_interface_ == "position"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/iiwa_arm_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));
            
                // Send joint position commands
                for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                    desired_commands_[i] = joint_positions_(i);
                }
            }
            else if(cmd_interface_ == "velocity"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));
            
                // Set joint velocity commands
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = joint_velocities_(i);
                }
            }
            else if(cmd_interface_ == "effort"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/effort_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));
            
                // Set joint effort commands
                for (long int i = 0; i < joint_efforts_cmd_.data.size(); ++i) {
                    desired_commands_[i] = joint_efforts_cmd_(i);
                }
            } 

            // Create msg and publish
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);

            RCLCPP_INFO(this->get_logger(), "Starting trajectory execution ...");
        }

    private:
        void cmd_publisher(){

            iteration_ = iteration_ + 1;

            // define trajectory
            double total_time = 3; // 
            int trajectory_len = 300; // 
            int loop_rate = trajectory_len / total_time;
            double dt = 1.0 / loop_rate;
            t_+=dt;
            
            // Update the KDLRobot structure with current states
            robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));

            if (task_ == "positioning" && t_< total_time){

                // Retrieve the trajectory point
                trajectory_point p = planner_.compute_trajectory(t_);

                // Compute EE frame
                KDL::Frame cartpos = robot_->getEEFrame();           

                // Compute the desired frame
                KDL::Frame desFrame;
                desFrame.M = KDL::Rotation::Quaternion(p.orientation.x(), p.orientation.y(), p.orientation.z(), p.orientation.w());
                desFrame.p = toKDL(p.pos); 

                // compute errors
                Eigen::Vector3d error = computeLinearError(p.pos, Eigen::Vector3d(cartpos.p.data));
                Eigen::Vector3d o_error = computeOrientationError(toEigen(init_cart_pose_.M), toEigen(cartpos.M));
                //std::cout << "The error norm is : " << error.norm() << std::endl;

                if(cmd_interface_ == "position"){
                    // Next Frame
                    KDL::Frame nextFrame; nextFrame.M = cartpos.M; nextFrame.p = cartpos.p + (toKDL(p.vel) + toKDL(1*error))*dt; 

                    // Compute IK
                    joint_positions_cmd_ = joint_positions_;
                    robot_->getInverseKinematics(nextFrame, joint_positions_cmd_);
                }
                else if(cmd_interface_ == "velocity"){
                    // Compute differential IK
                    
                    Vector6d cartvel; cartvel << p.vel + 5*error, o_error;
                    joint_velocities_cmd_.data = pseudoinverse(robot_->getEEJacobian().data)*cartvel;
                }
                else if(cmd_interface_ == "effort"){
                    // Effort control using idCntrOther
                    // Desired position
                    KDL::Frame desPos;
                    desPos.p = toKDL(p.pos);
                    desPos.M = cartpos.M;

                    // Desired velocity
                    KDL::Twist desVel;
                    desVel.vel = toKDL(p.vel);
                    desVel.rot = KDL::Vector(0.0, 0.0, 0.0);

                    // Desired acceleration
                    KDL::Twist desAcc;
                    desAcc.vel = toKDL(p.acc);
                    desAcc.rot = KDL::Vector(0.0, 0.0, 0.0);

                    // Control gains
                    double Kpp = 350;
                    double Kpo = 350;
                    double Kdp = 100;
                    double Kdo = 100;

                    // Compute torques using idCntr
                    Eigen::VectorXd torques = controller_->idCntrOther(desPos, desVel, desAcc, Kpp, Kpo, Kdp, Kdo);

                    // Assign computed torques to joint effort commands
                    for (int i = 0; i < torques.size(); ++i)
                    {
                        joint_efforts_cmd_(i) = torques(i);
                    }
                }

                // Update KDLrobot structure
                robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

                if(cmd_interface_ == "position"){
                    // Set joint position commands
                    for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                        desired_commands_[i] = joint_positions_cmd_(i);
                    }
                }
                else if(cmd_interface_ == "velocity"){
                    // Set joint velocity commands
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = joint_velocities_cmd_(i);
                    }
                }
                else if(cmd_interface_ == "effort"){
                    // Set joint effort commands
                    for (long int i = 0; i < joint_efforts_cmd_.data.size(); ++i) {
                        desired_commands_[i] = joint_efforts_cmd_(i);
                    }
                } 

                // Create msg and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);

                // std::cout << "/////////////////////////////////////////////////" <<std::endl <<std::endl;
                // std::cout << "EE pose is: " << robot_->getEEFrame() <<std::endl;  
                // std::cout << "Jacobian: " << robot_->getEEJacobian().data <<std::endl;
                // std::cout << "joint_positions_: " << joint_positions_.data <<std::endl;
                // std::cout << "joint_velocities_: " << joint_velocities_.data <<std::endl;
                // std::cout << "iteration_: " << iteration_ <<std::endl <<std::endl;
                // std::cout << "/////////////////////////////////////////////////" <<std::endl <<std::endl;
            }
            else if(t_ > total_time){
                RCLCPP_INFO_ONCE(this->get_logger(), "Trajectory executed successfully ...");
                robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
                
                KDL::Frame current_cart_pose = robot_->getEEFrame();
                
                if (cmd_interface_ == "velocity"){
                    // Send joint velocity commands
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = 0.0;
                    }
                
                    // Create msg and publish
                    std_msgs::msg::Float64MultiArray cmd_msg;
                    cmd_msg.data = desired_commands_;
                    cmdPublisher_->publish(cmd_msg);
                }
                else if (cmd_interface_ == "effort"){
                    // Continue using idCntr to maintain the final position
                    // Desired position
                    KDL::Frame desPos = current_cart_pose;

                    // Desired velocities and accelerations set to zero
                    KDL::Twist desVel;
                    desVel.vel = KDL::Vector(0.0, 0.0, 0.0);
                    desVel.rot = KDL::Vector(0.0, 0.0, 0.0);

                    KDL::Twist desAcc;
                    desAcc.vel = KDL::Vector(0.0, 0.0, 0.0);
                    desAcc.rot = KDL::Vector(0.0, 0.0, 0.0);

                    // Control gains for final position maintenance
                    double Kpp = 50;
                    double Kpo = 50;
                    double Kdp = 5;
                    double Kdo = 50;

                    // Compute torques using idCntr
                    Eigen::VectorXd torques = controller_->idCntrOther(desPos, desVel, desAcc, Kpp, Kpo, Kdp, Kdo);

                    // Assign computed torques to joint effort commands
                    for (int i = 0; i < torques.size(); ++i)
                    {
                        joint_efforts_cmd_(i) = torques(i);
                        desired_commands_[i] = joint_efforts_cmd_(i);
                    }

                    // Create message and publish
                    std_msgs::msg::Float64MultiArray cmd_msg;
                    cmd_msg.data = desired_commands_;
                    cmdPublisher_->publish(cmd_msg);
                }
            }
        }

        void joint_state_subscriber(const sensor_msgs::msg::JointState& sensor_msg){

            joint_state_available_ = true;
            for (unsigned int i  = 0; i < sensor_msg.position.size(); i++){
                joint_positions_.data[i] = sensor_msg.position[i];
                joint_velocities_.data[i] = sensor_msg.velocity[i];
            }
        }
        
        void aruco_pose_subscriber(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
        
            // Store the pose of the ArUco marker
            aruco_pose_ = KDL::Frame(
                KDL::Rotation::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w),
                KDL::Vector(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));

            // Update a variable indicating that the pose is available
            aruco_pose_available_ = true;
        }

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_pose_subscriber_;
        rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
        rclcpp::TimerBase::SharedPtr timer_; 
        rclcpp::TimerBase::SharedPtr subTimer_;
        rclcpp::Node::SharedPtr node_handle_;

        std::vector<double> desired_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        KDL::JntArray joint_positions_;
        KDL::JntArray joint_velocities_;
        KDL::JntArray joint_positions_cmd_;
        KDL::JntArray joint_velocities_cmd_;
        KDL::JntArray joint_accelerations_cmd_;
        KDL::JntArray joint_efforts_cmd_;

        std::shared_ptr<KDLRobot> robot_;
        std::shared_ptr<KDLController> controller_;
        
        KDLPlanner planner_;

        int iteration_;
        bool joint_state_available_;
        double t_;
        
        bool joint_state_available_;
        bool aruco_pose_available_;
        
        std::string cmd_interface_;
        std::string task_;
        
        KDL::Frame init_cart_pose_;
        KDL::Frame aruco_pose_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Iiwa_pub_sub>());
    rclcpp::shutdown();
    return 1;
}
