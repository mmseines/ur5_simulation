/*
 * write_node.cpp
 *
 *  Created on: Nov 13, 2015
 *      Author: Remo Diethelm
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <ros/ros.h>
#include <stdio.h>
#include "point_cloud_io/Write.hpp"

//point_cloud_io::RobotState stateMonitor;


int main(int argc, char** argv)
{
	std::cout << "whoa..." << std::endl;
	ros::init(argc, argv, "logger");
	ros::NodeHandle nodeHandle("~");
	

  point_cloud_io::RobotState stateMonitor;
  point_cloud_io::Record record(nodeHandle, &stateMonitor);
 //nodeHandle.subscribe("/joint_states", 1, &joint_state_callback);

  //ros::spin();

  ros::spin();
  return 0;
}