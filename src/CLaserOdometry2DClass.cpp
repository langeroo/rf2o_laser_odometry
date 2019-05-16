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


namespace rf2o {

CLaserOdometry2DClass::CLaserOdometry2DClass() :
  CLaserOdometry2D()
{
  ROS_INFO("[rf2o] Initializing RF2O node...");

  //Read Parameters
  //----------------
  ros::NodeHandle pn("~");

  ROS_INFO("[rf2o] private %s", pn.getNamespace().c_str());
  ROS_INFO("[rf2o] non-private %s", n.getNamespace().c_str());

  pn.param<std::string>("laser_scan_topic",laser_scan_topic,"/laser_scan");
  pn.param<std::string>("odom_topic", odom_topic, "/odom_rf2o");
  pn.param<std::string>("base_frame_id", base_frame_id, "/base_link");
  pn.param<std::string>("odom_frame_id", odom_frame_id, "/odom");
  pn.param<bool>("publish_tf", publish_tf, true);
  pn.param<std::string>("init_pose_from_topic", init_pose_from_topic, "/base_pose_ground_truth");
  pn.param<double>("freq",freq,10.0);
  pn.param<bool>("verbose", verbose, true);

  //Publishers and Subscribers
  //--------------------------
  odom_pub  = pn.advertise<nav_msgs::Odometry>(odom_topic, 5);
  laser_sub = n.subscribe<sensor_msgs::LaserScan>(laser_scan_topic,1,&CLaserOdometry2DClass::LaserCallBack,this);

  //init pose??
  if (init_pose_from_topic != "")
  {
    initPose_sub = n.subscribe<nav_msgs::Odometry>(init_pose_from_topic,1,&CLaserOdometry2DClass::initPoseCallBack,this);
    GT_pose_initialized  = false;
  }
  else
  {
    GT_pose_initialized = true;
    initial_robot_pose.pose.pose.position.x = 0;
    initial_robot_pose.pose.pose.position.y = 0;
    initial_robot_pose.pose.pose.position.z = 0;
    initial_robot_pose.pose.pose.orientation.w = 0;
    initial_robot_pose.pose.pose.orientation.x = 0;
    initial_robot_pose.pose.pose.orientation.y = 0;
    initial_robot_pose.pose.pose.orientation.z = 0;
  }

  setLaserPoseFromTf();

  //Init variables
  module_initialized = false;
  first_laser_scan   = true;

  ROS_INFO_STREAM("[rf2o] Listening laser scan from topic: " << laser_sub.getTopic());
}

bool CLaserOdometry2DClass::setLaserPoseFromTf()
{
  bool retrieved = false;

  // Set laser pose on the robot (through tF)
  // This allow estimation of the odometry with respect to the robot base reference system.
  tf::StampedTransform transform;
  transform.setIdentity();
  try
  {
    tf_listener.lookupTransform(base_frame_id, last_scan.header.frame_id, ros::Time(0), transform);
    retrieved = true;
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
    retrieved = false;
  }

  //TF:transform -> Eigen::Isometry3d

  const tf::Matrix3x3 &basis = transform.getBasis();
  Eigen::Matrix3d R;

  for(int r = 0; r < 3; r++)
    for(int c = 0; c < 3; c++)
      R(r,c) = basis[r][c];

  Pose3d laser_tf(R);

  const tf::Vector3 &t = transform.getOrigin();
  laser_tf.translation()(0) = t[0];
  laser_tf.translation()(1) = t[1];
  laser_tf.translation()(2) = t[2];

  setLaserPose(laser_tf);

  return retrieved;
}

bool CLaserOdometry2DClass::scan_available()
{
  return new_scan_available;
}

void CLaserOdometry2DClass::process(const ros::TimerEvent&)
{
  if( is_initialized() && scan_available() )
  {
    //Process odometry estimation
    odometryCalculation(last_scan);
    publish();
    new_scan_available = false; //avoids the possibility to run twice on the same laser scan
  }
  else
  {
    ROS_WARN("[rf2o] Waiting for laser_scans....") ;
  }
}

//-----------------------------------------------------------------------------------
//                                   CALLBACKS
//-----------------------------------------------------------------------------------

void CLaserOdometry2DClass::LaserCallBack(const sensor_msgs::LaserScan::ConstPtr& new_scan)
{
  if (GT_pose_initialized)
  {
    //Keep in memory the last received laser_scan
    last_scan = *new_scan;
    current_scan_time = last_scan.header.stamp;

    //Initialize module on first scan
    if (!first_laser_scan)
    {
      //copy laser scan to internal variable
      for (unsigned int i = 0; i<width; i++)
        range_wf(i) = new_scan->ranges[i];
      new_scan_available = true;
    }
    else
    {
      init(last_scan, initial_robot_pose.pose.pose);
      first_laser_scan = false;
    }
  }
}

void CLaserOdometry2DClass::initPoseCallBack(const nav_msgs::Odometry::ConstPtr& new_initPose)
{
  //Initialize module on first GT pose. Else do Nothing!
  if (!GT_pose_initialized)
  {
    initial_robot_pose = *new_initPose;
    GT_pose_initialized = true;
  }
}

void CLaserOdometry2DClass::publish()
{
  //first, we'll publish the odometry over tf
  //---------------------------------------
  if (publish_tf)
  {
    //ROS_INFO("[rf2o] Publishing TF: [base_link] to [odom]");
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = last_odom_time;
    odom_trans.header.frame_id = odom_frame_id;
    odom_trans.child_frame_id = base_frame_id;
    odom_trans.transform.translation.x = robot_pose_.translation()(0);
    odom_trans.transform.translation.y = robot_pose_.translation()(1);
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(rf2o::getYaw(robot_pose_.rotation()));
    //send the transform
    odom_broadcaster.sendTransform(odom_trans);
  }

  //next, we'll publish the odometry message over ROS
  //-------------------------------------------------
  //ROS_INFO("[rf2o] Publishing Odom Topic");
  nav_msgs::Odometry odom;
  odom.header.stamp = last_odom_time;
  odom.header.frame_id = odom_frame_id;
  //set the position
  odom.pose.pose.position.x = robot_pose_.translation()(0);
  odom.pose.pose.position.y = robot_pose_.translation()(1);
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(rf2o::getYaw(robot_pose_.rotation()));
  //set the velocity
  odom.child_frame_id = base_frame_id;
  odom.twist.twist.linear.x = lin_speed;    //linear speed
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = ang_speed;   //angular speed
  //fill in covariance
  odom.twist.covariance[6*0 + 0] = update_cov(0,0);     //x,x
  odom.twist.covariance[6*0 + 1] = update_cov(0,1);     //x,y
  odom.twist.covariance[6*1 + 0] = update_cov(1,0);     //y,x
  odom.twist.covariance[6*1 + 1] = update_cov(1,1);     //y,y

  //publish the message
  odom_pub.publish(odom);
}

} /* namespace rf2o */
