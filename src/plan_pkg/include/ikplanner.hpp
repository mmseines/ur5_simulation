/*!
	* \file ikplanner.hpp
	*
	* Decleration of help functions for moveit interface.
*/

#ifndef __IKPLANNER_HPP__
#define __IKPLANNER_HPP__


#include<ros/ros.h>
#include<fstream>
#include<string>
#include <cstdlib>
#include <ros/package.h>
#include "tf/tf.h"
#include <Eigen/Geometry>


#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <cmath>

// ROS and "MoveIt!" packages. Some of these are probably not needed. 
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <pluginlib/class_list_macros.h>
#include <pluginlib/class_loader.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

//Inverse kinematics 
//#include <descartes_moveit/moveit_state_adapter.h>
#include <urdf/model.h>
#include <eigen_conversions/eigen_kdl.h>
#include <kdl_parser/kdl_parser.hpp>
#include <tf_conversions/tf_kdl.h>
#include <ur_kinematics/ur_kin.h>


//Planning scene collision objects.
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>
//#include <actionlib/client/simple_action_client.h>
#include <sys/time.h>

/* 
	Help function from reading pose from text(.csv) file. 
		takes pose in string format
		returns pose in double values by reference.
*/
void getPose(std::string, double*);

/*
	Translates a tool pose to a 4x4 D-H transformation matrix.	
*/
void getTransform(double*, double [][4]);

/*
	functions for working with Eigen datastructures. 
*/
Eigen::Affine3d createRotationMatrix(double ax, double ay, double az);

/*
	Help function as inverse function from ur_kin takes a tf matrix as a double array and not en Eigen matrix. 
*/
void setFromMatrix(Eigen::Matrix4d, double [][4]);

/*
	standard sign function
*/
template <typename T> int sgn(T val);


/*
	Weigthed distance between two vectors. 
*/
double weightedDistance(std::vector<double> p, std::vector<double> v, double [6]);

/*
 
*/
int check_collision(std::vector<double> p, std::vector<double> v);

void add_table(moveit_msgs::PlanningScene &ps);

void add_wall(moveit_msgs::PlanningScene &ps);


#endif
