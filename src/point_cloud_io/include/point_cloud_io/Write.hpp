/*
 * Write.hpp
 *
 *  Created on: Nov 13, 2015
 *      Author: Remo Diethelm
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/JointState.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <eigen_conversions/eigen_kdl.h>
#include <cstdlib>

#include <iostream>
#include <fstream>

namespace point_cloud_io {

class RobotState
{
public:
  RobotState();
  Eigen::Affine3d getState();
  void setState(const sensor_msgs::JointState msg);
private:
  sensor_msgs::JointState current_state;
  robot_model_loader::RobotModelLoader robot_ml ;
  robot_model::RobotModelPtr kinematic_mdl ;
};

class Record
{
public:
  Record(ros::NodeHandle& nodeHandle, RobotState * rs_ptr);
  void jointStateCallback(const sensor_msgs::JointState & msg);
  ~Record();
private:
  RobotState * rs;
  ros::NodeHandle& nodeHandle_ ;
  ros::Subscriber jointStateSubscriber_;
  std::ofstream myfile;
  ros::Time start_time;
	timeval start_t;
	timeval stop_t;
};



class Write
{
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  Write(ros::NodeHandle& nodeHandle, RobotState * rs_ptr);

  /*!
   * Destructor.
   */
  ~Write();

 private:
  RobotState * rs;

  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * Point cloud callback function
   * @param cloud point cloud message.
   */
  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud);

  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

  //! Point cloud subscriber.
  ros::Subscriber pointCloudSubscriber_;

  //! Point cloud topic to subscribe to.
  std::string pointCloudTopic_;

  //! Path to the point cloud folder.
  std::string folderPath_;

  //! Point cloud file prefix.
  std::string filePrefix_;

  //! Point cloud file ending.
  std::string fileEnding_;

  //! Point cloud counter.
  unsigned int counter_ = 0;

  //! Settings for generating file name.
  bool addCounterToPath_ = true;
  bool addFrameIdToPath_ = false;
  bool addStampSecToPath_ = false;
  bool addStampNSecToPath_ = false;

  std::ofstream myfile;

  ros::Time start_time;
};

} /* namespace */
