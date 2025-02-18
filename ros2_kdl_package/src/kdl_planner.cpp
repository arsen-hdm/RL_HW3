#include "kdl_planner.h"


KDLPlanner::KDLPlanner(){}

/*KDLPlanner::KDLPlanner(double _maxVel, double _maxAcc)
{
    velpref_ = new KDL::VelocityProfile_Trap(_maxVel,_maxAcc);
}*/

/*KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd)
{
    trajDuration_ = _trajDuration;
    accDuration_ = _accDuration;
    trajInit_ = _trajInit;
    trajEnd_ = _trajEnd;
}*/

KDLPlanner::KDLPlanner(Eigen::Vector3d _trajInit) //For the cubic polinomial and linear
{
  trajInit_ = _trajInit;
}

KDLPlanner::KDLPlanner(double _trajRadius, Eigen::Vector3d _trajInit) //For the cubic polinomial and circular trajectory
{
  trajRadius_ = _trajRadius;
  trajInit_ = _trajInit;
}


KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration, Eigen::Vector3d _trajInit) //For the trapezoidal velocity profile and linear trajectory
{
  trajDuration_ = _trajDuration;
  accDuration_ = _accDuration;
  trajInit_ = _trajInit;
}

KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration, double _trajRadius, Eigen::Vector3d _trajInit) //For the trapezoidal velocity profile and circular trajectory
{
  trajDuration_ = _trajDuration;
  accDuration_ = _accDuration;
  trajRadius_ = _trajRadius;
  trajInit_ = _trajInit;
}

KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration, 
                       Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd, 
                       Eigen::Quaterniond& orientationInit, Eigen::Quaterniond& orientationEnd)  //Defined only for the HM3
{
    trajDuration_ = _trajDuration;
    accDuration_ = _accDuration;
    trajInit_ = _trajInit;
    trajEnd_ = _trajEnd;
    orientationInit_ = orientationInit; 
    orientationEnd_ = orientationEnd;
}

/*void KDLPlanner::CreateTrajectoryFromFrames(std::vector<KDL::Frame> &_frames,
                                            double _radius, double _eqRadius
                                            )
{
    path_ = new KDL::Path_RoundedComposite(_radius,_eqRadius,new KDL::RotationalInterpolation_SingleAxis());

    for (unsigned int i = 0; i < _frames.size(); i++)
    {
        path_->Add(_frames[i]);
    }
    path_->Finish();

    velpref_->SetProfile(0,path_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_, velpref_);
}*/

/*void KDLPlanner::createCircPath(KDL::Frame &_F_start,
                                KDL::Vector &_V_centre,
                                KDL::Vector& _V_base_p,
                                KDL::Rotation& _R_base_end,
                                double alpha,
                                double eqradius
                                )
{
    KDL::RotationalInterpolation_SingleAxis* otraj;
    otraj = new KDL::RotationalInterpolation_SingleAxis();
    otraj->SetStartEnd(_F_start.M,_R_base_end);
    path_circle_ = new KDL::Path_Circle(_F_start,
                                        _V_centre,
                                        _V_base_p,
                                        _R_base_end,
                                        alpha,
                                        otraj,
                                        eqradius);
    velpref_->SetProfile(0,path_circle_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_circle_, velpref_);
}*/

void KDLPlanner::trapezoidal_vel(double time, double &s, double &sdot, double &sddot)
{
  double vel_max = 0;
  double acc = 0;
  double constantVDuration = 0;
  double pos_after_acc = 0;
  double pos_before_dec = 0;
  vel_max = (trajDuration_/accDuration_)*(trajDuration_/2-accDuration_);
  acc = vel_max/accDuration_;
  constantVDuration = trajDuration_ - 2*accDuration_;
  
  if(time <= accDuration_)
  {
    s = 0.5*acc*(time*time);
    sdot = acc*time;
    sddot = acc;
    if(time==accDuration_)
    {
      pos_after_acc = s;
    }
  }
  else if(time <= trajDuration_-accDuration_)
  {
    s = pos_after_acc + vel_max*(time-accDuration_);
    sdot = vel_max;
    sddot = 0;
    if(time == trajDuration_-accDuration_)
    {
      pos_before_dec = s;
    }
  }
  else if(time <= trajDuration_)
  {
    s = pos_before_dec + vel_max*(time - accDuration_ - constantVDuration) - 0.5*acc*((time - accDuration_ - constantVDuration)*(time - accDuration_ - constantVDuration));
    sdot = vel_max - (vel_max/0.5)*(time - accDuration_ - constantVDuration);
    sddot = -acc;
  }
  else if(time > trajDuration_)
  {
    s = 1;
    sdot = 0;
    sddot = 0;
  }
}

void KDLPlanner::cubic_polinomial(double time, double &s, double &sdot, double &sddot)
{
  double a3 = 1/3;
  double a2 = 0.75;
  double a0 = trajInit_.y();
  
  if (time <= trajDuration_)
  {
    s = a3*(time*time*time) + a2*(time*time) + a0;
    sdot = 3*a3*(time*time) + 2*a2*time;
    sddot = 6*a3*time + 2*a2;
  }
  else if(time > trajDuration_)
  {
    s = 1;
    sdot = 0;
    sddot = 0;
  }
} 

/*KDL::Trajectory* KDLPlanner::getTrajectory()
{
	return traject_;
}*/

trajectory_point KDLPlanner::compute_trajectory_circular_polinomial(double time)
{

  trajectory_point traj;
  double s = 0;
  double sdot=0;
  double sddot=0;
  
  cubic_polinomial(time, s, sdot, sddot);

  traj.pos.x() = trajInit_.x();
  traj.pos.y() = trajInit_.y() - trajRadius_*cos(2*3.14*s);
  traj.pos.z() = trajInit_.z() - trajRadius_*sin(2*3.14*s); 
  traj.vel.x() = 0;
  traj.vel.y() = trajRadius_*sin(2*3.14*s)*2*3.14*sdot;
  traj.vel.z() = -trajRadius_*cos(2*3.14*s)*2*3.14*sdot;
  traj.acc.x() = 0;
  traj.acc.y() = trajRadius_*2*3.14*(cos(2*3.14*s)*(2*3.14*sdot)*(2*3.14*sdot)+sin(2*3.14*s)*sddot);
  traj.acc.z() = trajRadius_*2*3.14*(sin(2*3.14*s)*(2*3.14*sdot)*(2*3.14*sdot)-cos(2*3.14*s)*sddot);

  return traj;

}

trajectory_point KDLPlanner::compute_trajectory_circular_trapezoidal(double time)
{

  trajectory_point traj;
  double s = 0;
  double sdot=0;
  double sddot=0;
  
  trapezoidal_vel(time, s, sdot, sddot);

  traj.pos.x() = trajInit_.x();
  traj.pos.y() = trajInit_.y() - trajRadius_*cos(2*3.14*s);
  traj.pos.z() = trajInit_.z() - trajRadius_*sin(2*3.14*s); 
  traj.vel.x() = 0;
  traj.vel.y() = trajRadius_*sin(2*3.14*s)*2*3.14*sdot;
  traj.vel.z() = -trajRadius_*cos(2*3.14*s)*2*3.14*sdot;
  traj.acc.x() = 0;
  traj.acc.y() = trajRadius_*2*3.14*(cos(2*3.14*s)*(2*3.14*sdot)*(2*3.14*sdot)+sin(2*3.14*s)*sddot);
  traj.acc.z() = trajRadius_*2*3.14*(sin(2*3.14*s)*(2*3.14*sdot)*(2*3.14*sdot)-cos(2*3.14*s)*sddot);

  return traj;

}

trajectory_point KDLPlanner::compute_trajectory_linear_trapezoidal(double time)
{

  trajectory_point traj;
  double s = 0;
  double sdot=0;
  double sddot=0;
  
  trapezoidal_vel(time, s, sdot, sddot);

  traj.pos.x() = trajInit_.x();
  traj.pos.y() = trajInit_.y() + s;
  traj.pos.z() = trajInit_.z(); 
  traj.vel.x() = 0;
  traj.vel.y() = sdot;
  traj.vel.z() = 0;
  traj.acc.x() = 0;
  traj.acc.y() = sddot;
  traj.acc.z() = 0;

  return traj;

}

trajectory_point KDLPlanner::compute_trajectory_linear_polinomial(double time)
{

  trajectory_point traj;
  double s = 0;
  double sdot=0;
  double sddot=0;
  
  cubic_polinomial(time, s, sdot, sddot);

  traj.pos.x() = trajInit_.x();
  traj.pos.y() = trajInit_.y() + s;
  traj.pos.z() = trajInit_.z(); 
  traj.vel.x() = 0;
  traj.vel.y() = sdot;
  traj.vel.z() = 0;
  traj.acc.x() = 0;
  traj.acc.y() = sddot;
  traj.acc.z() = 0;

  return traj;

}

trajectory_point KDLPlanner::compute_trajectory(double time)
{
  /* trapezoidal velocity profile with accDuration_ acceleration time period and trajDuration_ total duration.
     time = current time
     trajDuration_  = final time
     accDuration_   = acceleration time
     trajInit_ = trajectory initial point
     trajEnd_  = trajectory final point */

  trajectory_point traj;

  Eigen::Vector3d ddot_traj_c = -1.0/(std::pow(accDuration_,2)-trajDuration_*accDuration_)*(trajEnd_-trajInit_);

  if(time <= accDuration_)
  {
    traj.pos = trajInit_ + 0.5*ddot_traj_c*std::pow(time,2);
    traj.vel = ddot_traj_c*time;
    traj.acc = ddot_traj_c;
  }
  else if(time <= trajDuration_-accDuration_)
  {
    traj.pos = trajInit_ + ddot_traj_c*accDuration_*(time-accDuration_/2);
    traj.vel = ddot_traj_c*accDuration_;
    traj.acc = Eigen::Vector3d::Zero();
  }
  else if(time >= trajDuration_)
  {
    traj.pos = trajEnd_ - 0.5*ddot_traj_c*std::pow(trajDuration_-time,2);
    traj.vel = ddot_traj_c*(trajDuration_-time);
    traj.acc = -ddot_traj_c;
  }
  
  // Calculate the interpolation factor s (between 0 and 1)
  double s = time / trajDuration_;
  if (s > 1.0) s = 1.0;

  // Interpolate orientation
  traj.orientation = orientationInit_.slerp(s, orientationEnd_);

  return traj;

}
