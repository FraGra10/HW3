#ifndef KDLPlanner_H
#define KDLPlanner_H

#include <kdl/frames_io.hpp>
#include <kdl/frames.hpp>
#include <kdl/trajectory.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_stationary.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/path_circle.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/utilities/error.h>
#include <kdl/trajectory_composite.hpp>
#include "Eigen/Dense"
#include <cmath>

struct trajectory_point{
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();
};

class KDLPlanner
{

public:

    KDLPlanner(double _maxVel, double _maxAcc);

    void CreateTrajectoryFromFrames(std::vector<KDL::Frame> &_frames,
                                    double _radius, double _eqRadius);
    void createCircPath(KDL::Frame &_F_start,
                        KDL::Vector &_V_centre,
                        KDL::Vector &_V_base_p,
                        KDL::Rotation &_R_base_end,
                        double alpha,
                        double eqradius);

    KDL::Trajectory* getTrajectory();

    //Base Constructor
    KDLPlanner(double _trajDuration, double _accDuration,
               Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd);

    //Circular Constructor
    KDLPlanner(double _trajDuration, Eigen::Vector3d _trajInit, double _trajRadius);

   //trap and cubic abscissa
   
    void trapezoidal_vel(double time, double &s, double &dots,double &ddots);
    void cubic_polinomial(double time, double &s, double &dots,double &ddots); 

    //trajectory definition
    
    trajectory_point compute_trajectory(double time,double _trajRadius, std::string profile, std::string traje); 
    trajectory_point compute_trapezoidal_linear( double t); 
    trajectory_point compute_cubic_linear( double t);
    trajectory_point compute_cubic_circular( double t , double _trajRadius);
    trajectory_point compute_trapezoidal_circular( double t , double _trajRadius); 



private:

    KDL::Path_RoundedComposite* path_;
    KDL::Path_Circle* path_circle_;
	KDL::VelocityProfile* velpref_;
	KDL::Trajectory* traject_;
    

    //////////////////////////////////
    double trajDuration_, accDuration_;
    Eigen::Vector3d trajInit_, trajEnd_;
    trajectory_point p;
    double trajRadius_;


};

#endif
