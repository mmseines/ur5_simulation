/*
	Using dense planner from ros descartes to make optimal trajectory.

*/

// header file with all necessary libraries.
#include "desc_planner.hpp"


int main(int argc, char** argv)
{
	ros::init(argc, argv, "descartes_planner");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner (1);
  spinner.start();


	TrajectoryVec points; 
	/*
	std::ifstream f("/home/magnus/Documents/path_ctrl/src/plan_pkg/paths/obstaclePath.csv");
	std::string line;
	double pose[6];
	while(std::getline(f, line))
	{
		getPose(line, pose);
		Eigen::Affine3d wobj_pt = makeRotMatrix(pose);
		//Eigen::Affine3d position = Eigen::Translation3d(pose[0], pose[1], pose[3]);	

	
		descartes_core::TrajectoryPtPtr pt = makeCartesianPoint(wobj_pt);

		points.push_back(pt);
	}

	/*
			load points from file.
	*/

 for (unsigned int i = 0; i < 10; ++i)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(0.370 - 0.05 * i, -0.450, 0.675);
    descartes_core::TrajectoryPtPtr pt = makeTolerancedCartesianPoint(pose);
    points.push_back(pt);
  }


	descartes_core::RobotModelPtr model (new descartes_moveit::MoveitStateAdapter);
	
	const std::string robot_description = "robot_description";
	const std::string group_name = "manipulator";
	const std::string world_frame = "world";
	const std::string tcp_frame = "ee_link";

	if (!model->initialize(robot_description, group_name, world_frame, tcp_frame))
  {
  	ROS_INFO("Could not initialize robot model");
  	return -1;
  }
	
	descartes_planner::DensePlanner planner;
  planner.initialize(model);


	if(!planner.planPath(points) )
	{
		ROS_ERROR("Could not solve for a valid path");
 		return -2;
	}

	TrajectoryVec result;
	if (!planner.getPath(result))
	{
		ROS_ERROR("Could not retrieve path");
		return -3;
 	}

	std::vector<std::string> names;
	nh.getParam("controller_joint_names", names);
	
	trajectory_msgs::JointTrajectory joint_solution = toROSJointTrajectory(result, *model, names, 1.0);

	if(!excecuteTrajectory(joint_solution))
	{
		ROS_ERROR("Could not excecute joint solution");
		return -4;

	}
	
	for(int i = 0; i < points.size(); i++)
	{
		//delete points[i];
	}


	ROS_INFO("Done!");
	return 0;

}


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

//Creation of rotation matrix
Eigen::Affine3d create_rotation_matrix(double ax, double ay, double az) {
  Eigen::Affine3d rx =
      Eigen::Affine3d(Eigen::AngleAxisd(ax, Eigen::Vector3d(1, 0, 0)));
  Eigen::Affine3d ry =
      Eigen::Affine3d(Eigen::AngleAxisd(ay, Eigen::Vector3d(0, 1, 0)));
  Eigen::Affine3d rz =
      Eigen::Affine3d(Eigen::AngleAxisd(az, Eigen::Vector3d(0, 0, 1)));
  return rz * ry * rx;
}


// function returning TolerancedFrame
Eigen::Affine3d makeRotMatrix(double * pose)
{
	Eigen::Affine3d rot = create_rotation_matrix(pose[3], pose[4], pose[5]);
	Eigen::Affine3d t(Eigen::Translation3d(pose[0], pose[1], pose[3]));
	Eigen::Matrix4d m = (rot * t).matrix();
	Eigen::Affine3d frame(m);
	return frame;
}

trajectory_msgs::JointTrajectory toROSJointTrajectory(const TrajectoryVec& trajectory,
                         const descartes_core::RobotModel& model,
                         const std::vector<std::string>& joint_names,
                         double time_delay)
{
// Fill out information about our trajectory
	trajectory_msgs::JointTrajectory result;
	result.header.stamp = ros::Time::now();
	result.header.frame_id = "world_frame";
	result.joint_names = joint_names;
	// For keeping track of time-so-far in the trajectory
	double time_offset = 0.0;
	// Loop through the trajectory
	for (TrajectoryIter it = trajectory.begin(); it != trajectory.end(); ++it)
	{
		// Find nominal joint solution at this point
		std::vector<double> joints;
		it->get()->getNominalJointPose(std::vector<double>(), model, joints);
	 
	// Fill out a ROS trajectory point
		trajectory_msgs::JointTrajectoryPoint pt;
		pt.positions = joints;
		// velocity, acceleration, and effort are given dummy values
	// we'll let the controller figure them out
		pt.velocities.resize(joints.size(), 0.0);
		pt.accelerations.resize(joints.size(), 0.0);
		pt.effort.resize(joints.size(), 0.0);
		 // set the time into the trajectory
		pt.time_from_start = ros::Duration(time_offset);
		// increment time
		time_offset += time_delay;

		result.points.push_back(pt);
	}
		
	return result;
}

descartes_core::TrajectoryPtPtr makeCartesianPoint(const Eigen::Affine3d& pose)
{
  using namespace descartes_core;
  using namespace descartes_trajectory;

  return TrajectoryPtPtr( new CartTrajectoryPt( TolerancedFrame(pose)) );
}

descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(const Eigen::Affine3d& pose)
{
  using namespace descartes_core;
  using namespace descartes_trajectory;
  return TrajectoryPtPtr( new AxialSymmetricPt(pose, M_PI/2.0-0.0001, AxialSymmetricPt::Z_AXIS) );
}



bool excecuteTrajectory(const trajectory_msgs::JointTrajectory& trajectory)
{
  // Create a Follow Joint Trajectory Action Client
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("joint_trajectory_action", true);
  if (!ac.waitForServer(ros::Duration(2.0)))
  {
    ROS_ERROR("Could not connect to action server");
    return false;
  }

  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;
  goal.goal_time_tolerance = ros::Duration(1.0);
  
  ac.sendGoal(goal);

  if (ac.waitForResult(goal.trajectory.points[goal.trajectory.points.size()-1].time_from_start + ros::Duration(5)))
  {
    ROS_INFO("Action server reported successful execution");
    return true;
  } else {
    ROS_WARN("Action server could not execute trajectory");
    return false;
  }
}




/*
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
	//robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  //robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
	//ROS_INFO("Model frame: %s", robot_model->getModelFrame().c_str());

	moveit::planning_interface::MoveGroup group("manipulator");
	//moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	
	std::ifstream f("/home/magnus/Documents/path_ctrl/src/plan_pkg/paths/detailedPath.csv");
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
		ROS_INFO("Adding waypoint x,y,z =  [%.2f, %0.2f, %0.2f]" ,pose[0], pose[1], pose[2]); 
		
		
		//waypoints.push_back(pose1);
		group.setPoseTarget(pose1);
		group.move();
		/*
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

	}
*/
/*	
return 0;
}
*/












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


