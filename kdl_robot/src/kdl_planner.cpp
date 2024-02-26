#include "kdl_ros_control/kdl_planner.h"
#include <cmath>

KDLPlanner::KDLPlanner(double _maxVel, double _maxAcc)
{
    velpref_ = new KDL::VelocityProfile_Trap(_maxVel,_maxAcc);
}

KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd)
{
    trajDuration_ = _trajDuration;
    accDuration_ = _accDuration;
    trajInit_ = _trajInit;
    trajEnd_ = _trajEnd;
}

//////////////////////////  (2.a) Definition of Circ Constructor   ////////////////////////////////////////////////////

KDLPlanner::KDLPlanner(double _trajDuration, Eigen::Vector3d _trajInit, double _trajRadius)
{   
    trajDuration_ = _trajDuration;
    trajInit_ = _trajInit;
    trajRadius_ = _trajRadius;
}


void KDLPlanner::CreateTrajectoryFromFrames(std::vector<KDL::Frame> &_frames,
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
}

void KDLPlanner::createCircPath(KDL::Frame &_F_start,
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
}

KDL::Trajectory* KDLPlanner::getTrajectory()
{
	return traject_;
}



//////////////////////////  (1.a) Definition of trapezoidal_vel function   ////////////////////////////////////////////////////

void KDLPlanner::trapezoidal_vel(double time, double &s, double &dots,double &ddots)
{
  
  double si=0;
  double sf=1;

  double ddot_traj_c = -1.0/(std::pow(accDuration_,2)-trajDuration_*accDuration_)*(sf-si);

  if(time <= accDuration_)
  {
    s = si + 0.5*ddot_traj_c*std::pow(time,2);
    dots = ddot_traj_c*time;
    ddots = ddot_traj_c;
  }
  else if(time <= trajDuration_-accDuration_)
  {
    s = si + ddot_traj_c*accDuration_*(time-accDuration_/2);
    dots = ddot_traj_c*accDuration_;
    ddots = 0;
  }
  else
  {
    s = sf - 0.5*ddot_traj_c*std::pow(trajDuration_-time,2);
    dots = ddot_traj_c*(trajDuration_-time);
    ddots = -ddot_traj_c;
  }

}


//////////////////////////  (1.b) Definition of cubic_polinomial function   ////////////////////////////////////////////////////


void KDLPlanner::cubic_polinomial(double time, double &s, double &dots,double &ddots)
{
  
  double si=0;
  double sf=1;
  double dsi=0;
  double dsf=0;

  double a0=si;
  double a1=dsi;
  double a2=3/std::pow(trajDuration_,2);
  double a3=-2/(std::pow(trajDuration_,3));

  s=a3*std::pow(time,3)+a2*std::pow(time,2)+a1*time+a0;
  dots=3*a3*std::pow(time,2)+2*a2*time+a1;
  ddots=6*a3*time+2*a2;
}


//////////////////////////  (2.c) Definition of Lin_Trap trajectory  ////////////////////////////////////////////////////

trajectory_point KDLPlanner::compute_trapezoidal_linear( double time){
  double s;
  double ds;
  double dds;
  trapezoidal_vel(time,s,ds,dds);

  trajectory_point traj;
  Eigen::Vector3d pi=trajInit_;
  Eigen::Vector3d pf=trajEnd_;
  Eigen::Vector3d dif=pf-pi;

  traj.pos=(1-s)*pi+s*pf;
  traj.vel=(-pi+pf)*ds;
  traj.acc=(-pi+pf)*dds;

    return traj;  
}

//////////////////////////  (2.c) Definition of Lin_Cubic trajectory  ////////////////////////////////////////////////////

trajectory_point KDLPlanner::compute_cubic_linear( double time){
  double s;
  double ds;
  double dds;
  cubic_polinomial(time,s,ds,dds);

  trajectory_point traj;
  Eigen::Vector3d pi=trajInit_;
  Eigen::Vector3d pf=trajEnd_;
  Eigen::Vector3d dif=pf-pi;

  traj.pos=(1-s)*pi+s*pf;
  traj.vel=(-pi+pf)*ds;
  traj.acc=(-pi+pf)*dds;

    return traj;  
}



//////////////////////////  (2.b) Definition of Circ_Cubic trajectory  ////////////////////////////////////////////////////


trajectory_point KDLPlanner::compute_cubic_circular( double time, double trajRadius_ ){

  trajectory_point traj;

  double s;
  double ds;
  double dds;
  cubic_polinomial(time, s, ds, dds);

  traj.pos(0) = trajInit_(0);
  traj.pos(1) = trajInit_(1)- trajRadius_*(std::cos(2*M_PI*s));
  traj.pos(2) = trajInit_(2) - trajRadius_*(std::sin(2*M_PI*s));

  traj.vel(0) = 0;
  traj.vel(1) = 2*M_PI*trajRadius_*std::sin(2*M_PI*s)*ds;
  traj.vel(2) = -2*M_PI*trajRadius_*std::cos(2*M_PI*s)*ds;
  
  traj.acc(0) = 0;
  traj.acc(1) = 2*M_PI*trajRadius_*(std::cos(2*M_PI*s)*2*M_PI*std::pow(ds,2)+std::sin(2*M_PI*s)*dds);
  traj.acc(2) = 2*M_PI*trajRadius_*(std::sin(2*M_PI*s)*2*M_PI*std::pow(ds,2)-std::cos(2*M_PI*s)*dds);

  return traj;

}

trajectory_point KDLPlanner::compute_trapezoidal_circular(double time, double trajRadius_)
{

  trajectory_point traj;

  double s;
  double ds;
  double dds;
  trapezoidal_vel(time,s,ds,dds);


  traj.pos(0) = trajInit_(0);
  traj.pos(1) = trajInit_(1)- trajRadius_*(std::cos(2*M_PI*s));
  traj.pos(2) = trajInit_(2) - trajRadius_*(std::sin(2*M_PI*s));

  traj.vel(0) = 0;
  traj.vel(1) = 2*M_PI*trajRadius_*std::sin(2*M_PI*s)*ds;
  traj.vel(2) = -2*M_PI*trajRadius_*std::cos(2*M_PI*s)*ds;
  
  traj.acc(0) = 0;
  traj.acc(1) = 2*M_PI*trajRadius_*(std::cos(2*M_PI*s)*2*M_PI*std::pow(ds,2)+std::sin(2*M_PI*s)*dds);
  traj.acc(2) = 2*M_PI*trajRadius_*(std::sin(2*M_PI*s)*2*M_PI*std::pow(ds,2)-std::cos(2*M_PI*s)*dds);

  return traj;
}

trajectory_point KDLPlanner::compute_trajectory(double time,double trajRadius_,std::string profile, std::string traje)
{
  if(profile=="cubic" && traje == "linear"){
     return compute_cubic_linear(time);
   }

   if(profile=="cubic" && traje == "circular"){
     return compute_cubic_circular(time,trajRadius_);
   }

   if(profile=="trapezoidal" && traje == "linear"){
     return compute_trapezoidal_linear(time);
   }

 if(profile=="trapezoidal" && traje == "circular"){
     return compute_trapezoidal_circular(time,trajRadius_);
   }

}




