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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"

#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"
 
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
            declare_parameter("cmd_interface", "effort"); // defaults to "position"
            declare_parameter("trajectory", "linear"); // Default is linear
            declare_parameter("abscissa", "trapezoidal"); // Default is trapezoidal
            get_parameter("cmd_interface", cmd_interface_);
            get_parameter("trajectory", trajectory_);
            get_parameter("abscissa", abscissa_);
            RCLCPP_INFO(get_logger(),"Current cmd interface is: '%s'", cmd_interface_.c_str());

            if (!(cmd_interface_ == "position" || cmd_interface_ == "velocity" || cmd_interface_ == "effort" ))
            {
                RCLCPP_ERROR(get_logger(),"Selected cmd interface is not valid! Use 'position', 'velocity' or 'effort' instead...");
                return;
            }
            
            // Declare the trajectory parameter (linear, circular)
            RCLCPP_INFO(get_logger(), "Current trajectory is: '%s'", trajectory_.c_str());

            // Validate the trajectory parameter
            if (!(trajectory_ == "linear" || trajectory_ == "circular"))
            {
                RCLCPP_ERROR(get_logger(), "Selected trajectory is not valid! Use 'linear' or 'circular' instead...");
                return;
            }
            
             // Declare the curvilinear abscissa parameter (trapezoidal, polinomial) 
            RCLCPP_INFO(get_logger(), "Current curvilinear abscissa is: '%s'", abscissa_.c_str());

            // Validate the abscissa parameter
            if (!(abscissa_ == "trapezoidal" || abscissa_ == "polinomial"))
            {
                RCLCPP_ERROR(get_logger(), "Selected abscissa is not valid! Use 'trapezoidal' or 'polinomial' instead...");
                return;
            }

            iteration_ = 0; t_ = 0;
            joint_state_available_ = false; 

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

            // EE's trajectory initial position (just an offset)
            Eigen::Vector3d init_position(Eigen::Vector3d(init_cart_pose_.p.data));
            //std::cout << "The initial EE pose is: " << std::endl;  
            //std::cout << init_position <<std::endl;

            // EE's trajectory end position (just opposite y)
            Eigen::Vector3d end_position; end_position << init_position[0], -init_position[1], init_position[2];
            //std::cout << "The final EE pose is: " << std::endl;  
            //std::cout << end_position <<std::endl;
            

            // Plan trajectory
            double traj_duration = 3, acc_duration = 1, t = 0.0, traj_radius = 0.15;
            trajectory_point p;
            if (trajectory_ == "circular"){
                if (abscissa_ == "trapezoidal"){
                    planner_ = KDLPlanner(traj_duration, acc_duration, traj_radius, init_position);
                    p = planner_.compute_trajectory_circular_trapezoidal(t);
                }
                else if (abscissa_ == "polinomial"){
                    planner_ = KDLPlanner(traj_radius, init_position);
                    p = planner_.compute_trajectory_circular_polinomial(t);
                }
            }
            else if (trajectory_ == "linear"){
                if (abscissa_ == "trapezoidal"){
                    planner_ = KDLPlanner(traj_duration, acc_duration, init_position);
                    p = planner_.compute_trajectory_linear_trapezoidal(t);
                }
                else if (abscissa_ == "polinomial"){
                    planner_ = KDLPlanner(init_position);
                    p = planner_.compute_trajectory_linear_polinomial(t);
                }
            }
              
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

            if (t_ < total_time){

                // Retrieve the trajectory point
                trajectory_point p;
                if (trajectory_ == "circular"){
                    if (abscissa_ == "trapezoidal"){
                        p = planner_.compute_trajectory_circular_trapezoidal(t_);
                    }
                    else if (abscissa_ == "polinomial"){
                        p = planner_.compute_trajectory_circular_polinomial(t_);
                    }
                }
                else if (trajectory_ == "linear"){
                    if (abscissa_ == "trapezoidal"){
                        p = planner_.compute_trajectory_linear_trapezoidal(t_);
                    }
                    else if (abscissa_ == "polinomial"){
                        p = planner_.compute_trajectory_linear_polinomial(t_);
                    }
                }

                // Compute EE frame
                KDL::Frame cartpos = robot_->getEEFrame();           

                // Compute desired Frame
                KDL::Frame desFrame; desFrame.M = cartpos.M; desFrame.p = toKDL(p.pos); 

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

            // for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
            //     RCLCPP_INFO(this->get_logger(), "Positions %zu: %f", i, sensor_msg.position[i]);                
            // }
            // std::cout<<"\n";
            // for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
            //     RCLCPP_INFO(this->get_logger(), "Velocities %zu: %f", i, sensor_msg.velocity[i]);
            // }
            // std::cout<<"\n";
            // for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
            //     RCLCPP_INFO(this->get_logger(), "Efforts %zu: %f", i, sensor_msg.effort[i]);
            // }

            joint_state_available_ = true;
            for (unsigned int i  = 0; i < sensor_msg.position.size(); i++){
                joint_positions_.data[i] = sensor_msg.position[i];
                joint_velocities_.data[i] = sensor_msg.velocity[i];
            }
        }

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
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
        std::string cmd_interface_;
        std::string trajectory_;
        std::string abscissa_;
        KDL::Frame init_cart_pose_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Iiwa_pub_sub>());
    rclcpp::shutdown();
    return 1;
}
