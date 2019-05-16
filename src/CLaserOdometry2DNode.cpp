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

#include "rf2o_laser_odometry/CLaserOdometry2DClass.h"
#include "rf2o_laser_odometry/CLaserOdometry2DNodelet.h"
#include <pluginlib/class_list_macros.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


PLUGINLIB_EXPORT_CLASS(rf2o_laser_odometry::rf2oNodelet, nodelet::Nodelet)
namespace rf2o_laser_odometry
{
  void rf2oNodelet::onInit()
  {
    rf2o::CLaserOdometry2DClass myLaserOdomNode;

    ros::TimerOptions timer_opt;
    timer_opt.oneshot   = false;
    timer_opt.autostart = true;
    timer_opt.callback_queue = ros::getGlobalCallbackQueue();
    timer_opt.tracked_object = ros::VoidConstPtr();

    timer_opt.callback = boost::bind(&rf2o::CLaserOdometry2DClass::process, &myLaserOdomNode, _1);
    timer_opt.period   = ros::Rate(myLaserOdomNode.freq).expectedCycleTime();

    ros::Timer rf2o_timer = ros::NodeHandle("~").createTimer(timer_opt);
    ros::spin();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "RF2O_LaserOdom");
  rf2o_laser_odometry::rf2oNodelet node;
  node.onInit();
  return EXIT_SUCCESS;
}
