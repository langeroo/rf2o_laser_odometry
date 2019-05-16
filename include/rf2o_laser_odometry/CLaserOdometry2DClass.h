/** ****************************************************************************************
*  This node presents a fast and precise method to estimate the planar motion of a lidar
*  from consecutive range scans. It is very useful for the estimation of the robot odometry from
*  2D laser range measurements.
*  This module is developed for mobile robots with innacurate or inexistent built-in odometry.
*  It allows the estimation of a precise odometry with low computational cost.
*  For more information, please refer to:
*
*  Planar Odometry from a Radial Laser Scanner. A Range Flow-based Approach. ICRA'16.
*  Available at: http://mapir.isa.uma.es/mapirwebsite/index.php/mapir-downloads/papers/217
*
* Maintainer: Javier G. Monroy
* MAPIR group: http://mapir.isa.uma.es/
*
* Modifications: Jeremie Deray
* Modifications: Greg Langer
******************************************************************************************** */

#include "rf2o_laser_odometry/CLaserOdometry2D.h"

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

namespace rf2o {

class CLaserOdometry2DClass : CLaserOdometry2D
{
public:

  CLaserOdometry2DClass();
  ~CLaserOdometry2DClass() = default;

  void process(const ros::TimerEvent &);
  void publish();

  bool setLaserPoseFromTf();

public:

  bool publish_tf, new_scan_available;

  double freq;

  std::string         laser_scan_topic;
  std::string         odom_topic;
  std::string         base_frame_id;
  std::string         odom_frame_id;
  std::string         init_pose_from_topic;

  ros::NodeHandle             n;
  sensor_msgs::LaserScan      last_scan;
  bool                        GT_pose_initialized;
  tf::TransformListener       tf_listener;          //Do not put inside the callback
  tf::TransformBroadcaster    odom_broadcaster;
  nav_msgs::Odometry          initial_robot_pose;

  //Subscriptions & Publishers
  ros::Subscriber laser_sub, initPose_sub;
  ros::Publisher odom_pub;

  bool scan_available();

  //CallBacks
  void LaserCallBack(const sensor_msgs::LaserScan::ConstPtr& new_scan);
  void initPoseCallBack(const nav_msgs::Odometry::ConstPtr& new_initPose);
};
}