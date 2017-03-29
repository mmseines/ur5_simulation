#include <boost/date_time.hpp>
#include <trac_ik/trac_ik.hpp>
#include <ros/ros.h>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include "kdl_conversions/kdl_msg.h"
#include <geometry_msgs/Pose.h>

double fRand(double min, double max);

void tourToJointPosition(ros::NodeHandle& nh, std::string chain_start, std::string chain_end, double timeout, std::string urdf_param, std::vector<geometry_msgs::Pose> &tour, const std::vector<KDL::JntArray> & ik_solutions);
