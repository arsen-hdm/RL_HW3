#include "kdl_control.h"
#include "utils.h"


KDLController::KDLController(KDLRobot &_robot)
{
    robot_ = &_robot;
}

Eigen::VectorXd KDLController::idCntr(KDL::JntArray &_qd,
                                      KDL::JntArray &_dqd,
                                      KDL::JntArray &_ddqd,
                                      double _Kp, double _Kd)
{
    // read current state
    Eigen::VectorXd q = robot_->getJntValues();
    Eigen::VectorXd dq = robot_->getJntVelocities();

    // calculate errors
    Eigen::VectorXd e = _qd.data - q;
    Eigen::VectorXd de = _dqd.data - dq;

    Eigen::VectorXd ddqd = _ddqd.data;
    return robot_->getJsim() * (ddqd + _Kd*de + _Kp*e)
            + robot_->getCoriolis() + robot_->getGravity() /*friction compensation?*/;
}

Eigen::VectorXd KDLController::idCntrOther(KDL::Frame &_desPos,
                                      KDL::Twist &_desVel,
                                      KDL::Twist &_desAcc,
                                      double _Kpp, double _Kpo,
                                      double _Kdp, double _Kdo)
{
    // Retrieve the current state of the manipulator
    KDL::Frame ee_frame = robot_->getEEFrame();
    KDL::Twist ee_vel = robot_->getEEVelocity();
    Eigen::MatrixXd j = robot_->getEEJacobian().data;
    Eigen::VectorXd j_dot_q_dot = robot_->getEEJacDotqDot();
    
    // Compute errors in the operational space
    Vector6d e;    
    Vector6d edot; 
    computeErrors(_desPos, ee_frame, _desVel, ee_vel, e, edot);

    // Build the gain matrices
    Matrix6d Kp, Kd;
    Kp.setZero();
    Kd.setZero();

    for (int i = 0; i < 3; ++i) {
        Kp(i, i) = _Kpp; 
        Kd(i, i) = _Kdp; 
    }
    for (int i = 3; i < 6; ++i) {
        Kp(i, i) = _Kpo; 
        Kd(i, i) = _Kdo; 
    }

    // Control output combining desired acceleration and PD terms
    Eigen::VectorXd desAcc_ = toEigen(_desAcc);
    Eigen::VectorXd control_output = desAcc_ + Kd * edot + Kp * e;

    // Desired joint accelerations
    Eigen::VectorXd y = pseudoinverse(j) * (control_output - j_dot_q_dot);

    // Compute control law
    Eigen::MatrixXd Jsim = robot_->getJsim();                                       // Inertia matrix, B
    
    Eigen::VectorXd n = robot_->getCoriolis() + robot_->getGravity();               // Coriolis + Gravity terms

    Eigen::VectorXd tau = Jsim * y + n;                                             // Desired torques

    return tau;
}

