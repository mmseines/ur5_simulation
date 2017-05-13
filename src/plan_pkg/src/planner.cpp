/*
	Using dense planner from ros descartes to make optimal trajectory.

*/


#include<ros/ros.h>
#include<fstream>
#include<string>
#include <cstdlib>
#include <ros/package.h>
#include "tf/tf.h"

#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

// Ros and MoveIt!
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


void getPose(std::string s, double* v){
	int p = 0;
	int q = 0;
	for(int i = 0; i< s.size(); i++){
		if(s[i] == ',' || i == s.size() -1){
			char tab2[16];
			strcpy(tab2, s.substr(p,i-p).c_str());
			v[q] = std::strtod(tab2, NULL);
			p = i+1;
			q++; 
		}
	}
	return;
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "path_sim");
	ros::NodeHandle n;
	
	ros::AsyncSpinner spinner(1);
	spinner.start();

	moveit::planning_interface::MoveGroup group("manipulator");
	//moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	
	std::ifstream f("/home/magnus/Documents/path_ctrl/src/plan_pkg/paths/obstaclePath.csv");
	//f.open(ros::package::find(plan_pkg)+"/paths/path.csv"); //ros::package::find(plan_pkg)
	if( !f.is_open())
		return 0;
	ROS_INFO("file is open");
	std::string line;	
	
	std::vector<geometry_msgs::Pose> waypoints;

	double scale = 100;

	int count  = -1;
	while(std::getline(f, line))
	{	
		count++;

		double pose [6];
		getPose(line, pose);

		tf::Quaternion q = tf::createQuaternionFromRPY(pose[3], pose[4],pose[5]);
	
		geometry_msgs::Pose pose1;

		pose1.orientation.x = q.x();
		pose1.orientation.y = q.y();
		pose1.orientation.z = q.z();
		pose1.orientation.w = q.w();
		pose1.position.x = pose[0]/scale;
		pose1.position.y = pose[1]/scale;
		pose1.position.z = pose[2]/scale;
		ROS_INFO("Adding waypoint x,y,z =  [%.2f, %0.2f, %0.2f]" ,pose1.position.x, pose1.position.y, pose1.position.z); 
		
		if(count % 2 == 0)
		{
			waypoints.push_back(pose1);
		}	
	}

	moveit_msgs::RobotTrajectory trajectory;
		
  group.setPlanningTime(1000.0);

	double fraction = group.computeCartesianPath(waypoints,
                                             0.00005,  // eef_step
                                             0.0,   // jump_threshold
                                             trajectory);
			//bool success = group.plan(my_plan);
	robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), "manipulator");

  		// Second get a RobotTrajectory from trajectory
  rt.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory);
 
  		// Thrid create a IterativeParabolicTimeParameterization object
  trajectory_processing::IterativeParabolicTimeParameterization iptp;

  		// Fourth compute computeTimeStamps
  bool success = iptp.computeTimeStamps(rt);
  		//ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");

  		// Get RobotTrajectory_msg from RobotTrajectory
  rt.getRobotTrajectoryMsg(trajectory);
	ROS_INFO("Path computed: (%.2f%% of pablovo)",
      	fraction * 100.0);
			
	moveit::planning_interface::MoveGroup::Plan plan;

	plan.trajectory_ = trajectory;

	group.execute(plan);
	
	return 0;
}




