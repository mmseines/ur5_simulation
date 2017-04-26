
#ifndef __DCPLANNER_HPP__
#define __DCPLANNER_HPP__


#include<ros/ros.h>
#include<fstream>
#include<string>
#include <cstdlib>
#include <ros/package.h>
#include "tf/tf.h"
#include <Eigen/Geometry>

// ROS Trajectory Action server definition
#include <control_msgs/FollowJointTrajectoryAction.h>
// Means by which we communicate with above action-server
#include <actionlib/client/simple_action_client.h>
 
// Includes the descartes robot model we will be using
#include <descartes_moveit/moveit_state_adapter.h>
// Includes the descartes trajectory type we will be using
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_trajectory/cart_trajectory_pt.h>
// Includes the planner we will be using
 #include <descartes_planner/dense_planner.h>


typedef std::vector<descartes_core::TrajectoryPtPtr> TrajectoryVec;
typedef TrajectoryVec::const_iterator TrajectoryIter;

/* 
	Help function from reading pose from text(.csv) file. 
		takes pose in string format
		returns pose in double values by reference.
*/
void getPose(std::string, double*);

/*
	Make a 3x3 rotation matrix from R,P,Y
*/
Eigen::Affine3d createRotationMatrix(double ax, double ay, double az);

/*
	Create a 4x4 rotation matrix from X,Y,Z, Roll,Pitch,Yaw  vector.
*/
Eigen::Affine3d makeRotMatrix(double * pose);

/*
	Transform between descartes joint trajectory and standard ROS joint trajectory.
*/
trajectory_msgs::JointTrajectory toROSJointTrajectory(const TrajectoryVec& trajectory,
                         const descartes_core::RobotModel& model,
                         const std::vector<std::string>& joint_names,
                         double time_delay);


/*
	Make a locked 6D pose target.
*/
descartes_core::TrajectoryPtPtr makeCartesianPoint(const Eigen::Affine3d& pose);

/*
	Make a trajectory point with a free axis of rotation.
*/
descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(const Eigen::Affine3d& pose);

/*
	Execute a standard ROS trajectory using a simple action client.
*/
bool excecuteTrajectory(const trajectory_msgs::JointTrajectory& trajectory);

#endif 
