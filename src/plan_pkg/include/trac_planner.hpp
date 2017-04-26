#include <boost/date_time.hpp>
#include <trac_ik/trac_ik.hpp>
#include <ros/ros.h>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl_conversions/kdl_msg.h>
#include <geometry_msgs/Pose.h>
#include <kdl_parser/kdl_parser.hpp>
#include "tf/tf.h"

//Moveit
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <pluginlib/class_list_macros.h>
#include <pluginlib/class_loader.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <moveit/kinematic_constraints/utils.h>

double fRand(double min, double max);

void tourToJointPosition(ros::NodeHandle& nh, std::string chain_start, std::string chain_end, double timeout, std::string urdf_param, std::vector<geometry_msgs::Pose> tour, std::vector<KDL::JntArray>  &ik_solutions);


int moveRobot(ros::NodeHandle& nh, const std::vector<KDL::JntArray>  &ik_solutions);
