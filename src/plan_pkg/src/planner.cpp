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


// -----------------
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include "ikplanner.hpp"

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

	ros::Duration sleep_time(10.0);

	moveit::planning_interface::MoveGroup group("manipulator");
	
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());

	ROS_INFO("End effector frame: %s", group.getEndEffectorLink().c_str());
	
	group.setPoseReferenceFrame("base_link");

	group.startStateMonitor();
  group.setStartStateToCurrentState();
  group.setPlanningTime(12);
	
// ---------------- Add collision object(s). -----------------
	ros::Publisher planning_scene_diff_publisher = n.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  	while(planning_scene_diff_publisher.getNumSubscribers() < 1)
  	{
    		ros::WallDuration sleep_t(0.5);
   		 sleep_t.sleep();
	}

	moveit_msgs::CollisionObject table;
	table.id = "table";
	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = 1.0;
	primitive.dimensions[1] = 1.0;
	primitive.dimensions[2] = 0.1;

	table.primitives.push_back(primitive);
	
	table.header.frame_id = "/world";

	geometry_msgs::Pose table_pose;
	table_pose.position.z = -0.3;
	table_pose.position.x = 0.2;
	table_pose.position.y = 0.2;
	table_pose.orientation.x = 0.0;
	table_pose.orientation.y = 0.0;
	table_pose.orientation.z = 0.0;
	table_pose.orientation.w = 0.0;

	table.primitive_poses.push_back(table_pose);

	table.operation = table.ADD;

	moveit_msgs::PlanningScene planning_sc;
	planning_sc.world.collision_objects.push_back(table);
	planning_sc.is_diff = true;
	planning_scene_diff_publisher.publish(planning_sc);
	sleep_time.sleep();
	
	ros::ServiceClient planning_scene_diff_client = n.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
	planning_scene_diff_client.waitForExistence();
// and send the diffs to the planning scene via a service call:
	moveit_msgs::ApplyPlanningScene srv;
	srv.request.scene = planning_sc;
	planning_scene_diff_client.call(srv);

// ------------- End adding collision objects. -----------

	std::ifstream f("/home/magnus/Documents/path_ctrl/src/plan_pkg/paths/latestPath.csv");
	//f.open(ros::package::find(plan_pkg)+"/paths/path.csv"); //ros::package::find(plan_pkg)
	if(!f.is_open()){
		ROS_ERROR("failed to open file");
		return 0;
	}
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
		
	group.setPlanningTime(10.0);

	double fraction = group.computeCartesianPath(waypoints,
                                             0.01,  // eef_step
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




