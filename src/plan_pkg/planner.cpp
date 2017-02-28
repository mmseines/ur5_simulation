
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

	//ROS_INFO("running loader");
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
	//ROS_INFO("Model frame: %s", robot_model->getModelFrame().c_str());

	moveit::planning_interface::MoveGroup group("manipulator");
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	
	std::ifstream f("/home/magnus/Documents/path_ctrl/src/plan_pkg/paths/path.csv");
	//f.open(ros::package::find(plan_pkg)+"/paths/path.csv"); //ros::package::find(plan_pkg)
	if( !f.is_open())
		return 0;
	ROS_INFO("file is open");
	std::string line;	
	
	std::vector<geometry_msgs::Pose> waypoints;


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
		pose1.position.x = pose[0];
		pose1.position.y = pose[1];
		pose1.position.z = pose[2];
		ROS_INFO("Adding waypoint x,y,z =  [%.2f, %0.2f, %0.2f]" ,pose[0], pose[1], pose[2]); 
		
		
		waypoints.push_back(pose1);
		
		if(waypoints.size() > 2 || f.eof()){
			moveit_msgs::RobotTrajectory trajectory;
		
  		group.setPlanningTime(20.0);

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
			ROS_INFO("Path computed: (%.2f%% of implied path acheived)",
      	fraction * 100.0);
			
			moveit::planning_interface::MoveGroup::Plan plan;
  		// Finally plan and execute the trajectory
  		plan.trajectory_ = trajectory;


  		group.execute(plan);
			ros::WallDuration sleep_time(2.0);
			sleep_time.sleep();

			waypoints.clear();
			if(f.eof()){
				ROS_INFO("Path completed ?");
			}
		}
			
	}

	if(waypoints.size() != 0){
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
			ROS_INFO("Path computed: (%.2f%% of implied path acheived)",
      	fraction * 100.0);
			
			moveit::planning_interface::MoveGroup::Plan plan;
  		// Finally plan and execute the trajectory
  		plan.trajectory_ = trajectory;
  		group.execute(plan);
			sleep(0.5);
	}
	
return 0;
}













/*
	robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(robot_model));
	kinematic_state->setToDefaultValues();
	const robot_state::JointModelGroup* joint_model_group = robot_model->getJointModelGroup("manipulator");
	//robot_model->printModelInfo(std::cout);


	planning_scene::PlanningScenePtr planning_scene( new planning_scene::PlanningScene(robot_model));
*/

/*
	planning_interface::MotionPlanRequest req;
	planning_interface::MotionPlanResponse res;

	geometry_msgs::PoseStamped target_pose1;
	
	target_pose1.header.frame_id = "base_link";
	target_pose1.pose.orientation.x = 0.0;
	target_pose1.pose.orientation.y = 0.0;
	target_pose1.pose.orientation.z = 0.0;
	target_pose1.pose.orientation.w = 1.0;
	target_pose1.pose.position.x = 0.2;
	target_pose1.pose.position.y = 0.4;
	target_pose1.pose.position.z = 0.2;

	std::vector<double> tolerance_pose(3, 0.01);
	std::vector<double> tolerance_angle(3, 0.01);

//Make goal constraints.
	
	req.group_name = "manipulator";
	ROS_INFO("SO FAR SO GOOD");
	moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints("ee_link", target_pose1, tolerance_pose, tolerance_angle);
	req.goal_constraints.push_back(pose_goal);

	 ROS_INFO("fuckin hell");
	boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > planner_plugin_loader;
	planning_interface::PlannerManagerPtr planner_instance;
	std::string planner_plugin_name;

	if (!n.getParam("move_group/planning_plugin", planner_plugin_name))
  	ROS_FATAL_STREAM("Could not find planner plugin name");
	try
	{
  	planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
	}
	catch(pluginlib::PluginlibException& ex)
	{
	  ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
	}
	try
	{
  	planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
  	if (!planner_instance->initialize(robot_model, n.getNamespace()))
    	ROS_FATAL_STREAM("Could not initialize planner instance");
  	ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
	}
	catch(pluginlib::PluginlibException& ex)
	{
  	const std::vector<std::string> &classes = planner_plugin_loader->getDeclaredClasses();
  	std::stringstream ss;
  	for (std::size_t i = 0 ; i < classes.size() ; ++i)
    	ss << classes[i] << " ";
  	ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                   << "Available plugins: " << ss.str());
	}


	planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
	context->solve(res);
	if(res.error_code_.val != res.error_code_.SUCCESS)
	{
  	ROS_ERROR("Could not compute plan successfully");
  	return 0;
	}else{
		ROS_INFO("We did it boys");
	}
*/


