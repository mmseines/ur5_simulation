/*
	Using dense planner from ros descartes to make optimal trajectory.

*/

// Core ros functionality like ros::init and spin
#include <ros/ros.h>
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

int main(int argc, char** argv)
{
	ros::init(argc, argv, "descartes_planner");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner (1);
  spinner.start();


	TrajectoryVec points;

	std::ifstream f("/home/magnus/Documents/path_ctrl/src/plan_pkg/paths/detailedPath.csv"); 
	/*
			load points from file.
	*/


	descartes_core::RobotModelPtr model (new descartes_moveit::MoveitStateAdapter);
	
	const std::string robot_description = "robot_description";
	const std::string group_name = "manipulator";
	const std::string world_frame = "base_link";
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
	
	ROS_INFO("Done!");
	return 0;

}

