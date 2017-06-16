/* ROS interface for implementing full path given by multiple pse goals using moveit functionality.
	
*/
//Header

#include "ikplanner.hpp"
#include "trac_planner.hpp"

// ROS message types.

Eigen::Vector3f x_axis(1, 0, 0);
Eigen::Vector3f y_axis(0, 1, 0);
Eigen::Vector3f z_axis(0, 0, 1);
Eigen::Vector3d end_effector_offset(0.045, 0.0, 0.035);
/*
		Main function
*/
int main(int argc, char **argv)
{

	ros::init(argc, argv, "path_sim");
	ros::NodeHandle n;	
	ros::AsyncSpinner spinner(1);
	spinner.start();
	
	ros::Duration sleep_time(10.0);

	timeval start_t;
	timeval stop_t;
	

	long execution_time_ms = 0;
	long planning_time_ms = 0;
	long init_time_ms = 0; 
	
	gettimeofday(&start_t, NULL);
/*
	Load urdf, setup joint model groups and kinematic state. 
*/

	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
	ROS_INFO("Model frame: %s", robot_model->getModelFrame().c_str());
	
	planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
	robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(robot_model));
	kinematic_state->setToDefaultValues();


 	//Get joint model names and group values
	const robot_state::JointModelGroup * joint_model_group = robot_model->getJointModelGroup("manipulator");
	const std::vector<std::string>  joint_names = joint_model_group->getJointModelNames();
	
	std::vector<double> joint_values;	

	

/* -----------------------------Loading planner --------------------- (necessary for constraint based pose goal planning.) */
	boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > planner_plugin_loader;
	planning_interface::PlannerManagerPtr planner_instance;
	std::string planner_plugin_name = "ompl_interface/OMPLPlanner";

  // Get name from the ROS param server, and then load the planner
	if (!n.getParam("planning_plugin", planner_plugin_name))
		ROS_FATAL_STREAM("Could not find planner plugin name");
	try{
		planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
	}catch(pluginlib::PluginlibException& ex){
		ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
	}try{
		planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
		if (!planner_instance->initialize(robot_model, n.getNamespace()))
			ROS_FATAL_STREAM("Could not initialize planner instance");
		ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
	}catch(pluginlib::PluginlibException& ex){
		const std::vector<std::string> &classes = planner_plugin_loader->getDeclaredClasses();
		std::stringstream ss;
		for (std::size_t i = 0 ; i < classes.size() ; ++i)
				ss << classes[i] << " ";
		ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                     << "Available plugins: " << ss.str());
	}

	std::vector< std::string> algs;
	planner_instance->getPlanningAlgorithms(algs);
	std::stringstream sstr;
	for(int i = 0; i < algs.size(); i++){
		sstr << algs[i] << "\n";
	}
	ROS_INFO_STREAM("algorithms: " << sstr.str() << "....");
/* ----------------- End planner loader --------*/
	

/* Moveit group interface for actually moving the robot */	
	moveit::planning_interface::MoveGroup group("manipulator");

	/* Specify tolerance and declare request and response */	
	planning_interface::MotionPlanRequest req;
	req.group_name = "manipulator";
	req.planner_id = "manipulator";
	req.allowed_planning_time = 2.5;
	planning_interface::MotionPlanResponse res;
	moveit_msgs::MotionPlanResponse response;
/* Specify bounds on the workspace */


// ----------- Add collision for table. ----------------------

	moveit_msgs::PlanningScene planning_sc;
	add_table(planning_sc);
	add_wall(planning_sc);
	add_mount(planning_sc);
	planning_sc.is_diff = true;
	//planning_scene->setPlanningSceneDiffMsg(planning_sc);
	
//---------------------------------------------------------------------------------------------------------------


	double tolerance_below = 1e-3;
	double tolerance_above = 1e-3;
	double scale = 100; //Viewpoint planning done in cm for visibility (in both model editors and rviz), but ros operates with m.
	req.workspace_parameters.min_corner.z = -0.25;
	req.workspace_parameters.min_corner.x = req.workspace_parameters.min_corner.y = -0.8;
	req.workspace_parameters.max_corner.x = 0.25;
	req.workspace_parameters.max_corner.y =  0.8;
	req.workspace_parameters.max_corner.z = 0.8;

	/* Open and terate through path composed of several pose targets */
	std::string path_name;	
	if(!n.getParam("path_sim/path", path_name)){
		ROS_ERROR("Error reading argument");
		path_name = "Augmented.csv";
	}
	std::ifstream f(ros::package::getPath("plan_pkg")+"/paths/"+ path_name);
	if( !f.is_open()){
		ROS_ERROR("Could not open path, exiting.."); 
		return 0;
	}
	ROS_INFO("file is open");
	std::string line;	

	int count = 0;
	
	double tf_matrix[4][4];

	std::vector<double> curr_joint_states;

	gettimeofday(&stop_t, NULL);
	init_time_ms += (stop_t.tv_sec - start_t.tv_sec)*1000000 +  stop_t.tv_usec - start_t.tv_usec;
	
	int sucess_points = 0;
	int error_points = 0; 

	while(std::getline(f, line) && ros::ok())
	{	
		if(count != 0){
			if(!std::getline(f,line))
			{
				count++;
				break;
			}
		}
		
		count++;
		gettimeofday(&start_t, NULL);

		double pose [6];
		getPose(line, pose);

		pose[0] = pose[0]/scale;
		pose[1] = pose[1]/scale;
		pose[2] = pose[2]/scale;
		 

		// calculate ee_offset. 
		/*
  		Eigen::AngleAxisf mroll = Eigen::AngleAxisf(pose[3] + M_PI, x_axis);
		Eigen::AngleAxisf mpitch = Eigen::AngleAxisf(pose[4],y_axis);
  		Eigen::AngleAxisf myaw = Eigen::AngleAxisf(pose[5], z_axis);
		*/

		Eigen::Affine3d ee_rot = createRotationMatrix(pose[3], pose[4], pose[5]);

		
		Eigen::Vector3d ee_offset = ee_rot.rotation()*end_effector_offset; //myaw*(mpitch*(mroll*end_effector_offset));	
		// ---
		
 		Eigen::Affine3d r = createRotationMatrix(pose[3], pose[4], pose[5]);
		Eigen::Affine3d t(Eigen::Translation3d(Eigen::Vector3d(pose[0]-ee_offset[0],pose[1]-ee_offset[1],pose[2]-ee_offset[2])));
		Eigen::Matrix4d m = (t * r).matrix();	
		setFromMatrix(m, tf_matrix);

		double q_ik_sols[8][6];
		std::vector< std::vector<double> > valid_solutions;
		double weights[6] = {8.0, 4.0, 2.0 ,1.0, 0.2, 2.0};

		//Compute inverse kinematics and check them to be within joint limits -pi to pi
		int num_sols = ur_kinematics::inverse((double*) tf_matrix,(double*) q_ik_sols, 0.0f);  
		for(int i = 0; i < num_sols; i++){
			std::vector<double> val_sol;
			for(int q = 0; q < 6; q++){
				if (q_ik_sols[i][q] != q_ik_sols[i][q] || !std::isfinite(q_ik_sols[i][q]) ){ //Check for nan / inf
					break;				
				}else if( fabs(q_ik_sols[i][q]) <= M_PI){
						val_sol.push_back(q_ik_sols[i][q]);
				}else if( q_ik_sols[i][q] + 2*M_PI <= M_PI || q_ik_sols[i][q] - 2*M_PI >= -M_PI){
						val_sol.push_back(q_ik_sols[i][q] - sgn(q_ik_sols[i][q])*2*M_PI);
				}else{
					//val_sol.push_back(q_ik_sols[i][q]);
					ROS_ERROR("WUT? %0.2f", q_ik_sols[i][q]);
					break;
				} 
				 
			}
			if(val_sol.size() == 6 && (val_sol[1] >= -3.14 && val_sol[1] <= 0.00)){ // could be stricter, avoiding configurations where the lift joint is bent downward.
					valid_solutions.push_back(val_sol);
			}
			val_sol.clear();  				
		}
		
		ROS_INFO("Inverse kinematics gave: %i, solutsions where %i where valid", num_sols, (int) valid_solutions.size()); 
		

		if (valid_solutions.size() != 0)
		{
			//semi temp fix to get current state. 
			moveit::core::RobotState blah = *group.getCurrentState();
			blah.copyJointGroupPositions(joint_model_group, curr_joint_states);
			planning_scene->setCurrentState(*group.getCurrentState());			

			collision_detection::CollisionRequest c_req;
			c_req.contacts = 1;  // check for self collision via contact. 
			collision_detection::CollisionResult c_res;			
			joint_values.clear();			//Incase there are no solutions without self collision.

			//minimum distance collision free joint configuration.
			double min_distance = FLT_MAX;		
			for(int i = 0; i < valid_solutions.size(); i++) {
				double dis = weightedDistance(curr_joint_states, valid_solutions[i], weights);									
				
				if( dis < min_distance){
					blah.setJointGroupPositions(joint_model_group, valid_solutions[i]);
					planning_scene->checkCollision(c_req, c_res, blah);	
			
					if(!c_res.collision){
						min_distance = dis;			
						joint_values = valid_solutions[i];
					}
					c_res.clear();
				}
			}

			
			if(joint_values.size() != 6){
				ROS_ERROR("Error selecting IK solution: %i, size of joint state vector = %zu, number of assumed valid solutions: %zu", count, joint_values.size(), valid_solutions.size());
				error_points++;
				continue;
			}

			//Define goals etc.
			robot_state::RobotState goal_state(robot_model);

			goal_state.setJointGroupPositions(joint_model_group, joint_values);

			//const Eigen::Affine3d &end_effector_state = goal_state.getGlobalLinkTransform("tool_tip");
			
			//ROS_INFO_STREAM("FK says translation is: \n " << end_effector_state.translation());
			//ROS_INFO_STREAM("When it is actually: \n" << pose[0] << " " << pose[1] << " " << pose[2] << "\n");
			
			//ROS_INFO_STREAM("Joint states should be: \n" << joint_values[0] << "\n"<< joint_values[1] << "\n"<< joint_values[2] << "\n"<< joint_values[3] << "\n"<< joint_values[4] << "\n"<< joint_values[5] << "\n");

			moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group, tolerance_below, tolerance_above);
			req.goal_constraints.clear();
			req.goal_constraints.push_back(joint_goal);				

			/* Call the Planner */
			planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
			context->solve(res);

  			if(res.error_code_.val != res.error_code_.SUCCESS)
  			{
   				ROS_ERROR("Could not compute plan sucsessfully for wp: %i", count );
				error_points++;
				//return 0;
			}else{
			
				//Success 
				res.getMessage(response);				
				moveit::planning_interface::MoveGroup::Plan plan;
				moveit_msgs::RobotTrajectory trajectory = response.trajectory;
				
				// Using suggested fix to solve time parameterization of the proposed trajectory 
				robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), "manipulator");
				rt.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory);
				trajectory_processing::IterativeParabolicTimeParameterization iptp;
				bool success = iptp.computeTimeStamps(rt);
				rt.getRobotTrajectoryMsg(trajectory);
				
				//Finaly have a properly parametarized trajectory. 
				plan.start_state_ = response.trajectory_start;			
				plan.trajectory_ = trajectory;

				gettimeofday(&stop_t, NULL);
				planning_time_ms += (stop_t.tv_sec - start_t.tv_sec)*1000000 +  stop_t.tv_usec - start_t.tv_usec;

				gettimeofday(&start_t, NULL);
				//And this works somehow.
				moveit_msgs::MoveItErrorCodes err = group.execute(plan); //is this blocking? if not then how to time it?
				if( err.val != moveit_msgs::MoveItErrorCodes::SUCCESS ){
					ROS_ERROR("error executing path");
				}

				gettimeofday(&stop_t, NULL);
				execution_time_ms += (stop_t.tv_sec - start_t.tv_sec)*1000000 +  stop_t.tv_usec - start_t.tv_usec;
				sucess_points++;


				//ros::WallDuration sleep_t(3.0);
   				//sleep_t.sleep();
			}


			
		}else{
  		ROS_ERROR("Did not find IK solution for viewpoint %i", count);
			error_points++;
		}
		
		
	}
	//Publish timing.
	
	execution_time_ms = execution_time_ms/1000;
	init_time_ms = init_time_ms/1000;
	planning_time_ms = planning_time_ms/1000; 
	long total_time = execution_time_ms + init_time_ms + planning_time_ms;
	ROS_INFO("total time spent %ld ms", total_time);
	ROS_INFO("initialisation: %ld ms", init_time_ms);
	ROS_INFO("Planning %ld ms", planning_time_ms);
	ROS_INFO("Movement %ld ms", execution_time_ms);

	int total_points = sucess_points + error_points;
	float percentage = sucess_points/((float) total_points);
	ROS_INFO("Sucessfully reached %i viewpoints out of a total: %i", sucess_points, total_points);
	return 0;
}


//Add the table to the planning scene
void add_table(moveit_msgs::PlanningScene &ps)
{
	moveit_msgs::CollisionObject table;
	table.id = "table";
	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = 1.0;
	primitive.dimensions[1] = 1.0;
	primitive.dimensions[2] = 0.2;

	table.primitives.push_back(primitive);
	
	table.header.frame_id = "world";

	geometry_msgs::Pose table_pose;
	table_pose.position.z = -0.33;
	table_pose.position.x = -0.2;
	table_pose.position.y = 0.0;
	table_pose.orientation.x = 0.0;
	table_pose.orientation.y = 0.0;
	table_pose.orientation.z = 0.0;
	table_pose.orientation.w = 0.0;

	table.primitive_poses.push_back(table_pose);

	table.operation = table.ADD;
	ps.world.collision_objects.push_back(table);

}

//Add the wall to the planning scene
void add_wall(moveit_msgs::PlanningScene &ps)
{
	moveit_msgs::CollisionObject wall;
	wall.id = "wall";
	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = 0.1;
	primitive.dimensions[1] = 2.0;
	primitive.dimensions[2] = 1.5;

	wall.primitives.push_back(primitive);
	
	wall.header.frame_id = "world";

	geometry_msgs::Pose wall_pose;
	wall_pose.position.x = 0.0;
	wall_pose.position.y = -0.4;
	wall_pose.position.z = 0.0;

	tf::Quaternion q = tf::createQuaternionFromRPY(0.0, 0.0, -M_PI/4.0);
	wall_pose.orientation.x = q.x();
	wall_pose.orientation.y = q.y();
	wall_pose.orientation.z = q.z();
	wall_pose.orientation.w = q.w();

	wall.primitive_poses.push_back(wall_pose);

	wall.operation = wall.ADD;
	ps.world.collision_objects.push_back(wall);

}

void add_mount(moveit_msgs::PlanningScene &ps)
{
	moveit_msgs::CollisionObject wall;
	wall.id = "mount";
	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = 0.15;
	primitive.dimensions[1] = 0.15;
	primitive.dimensions[2] = 1.0;

	wall.primitives.push_back(primitive);
	
	wall.header.frame_id = "world";

	geometry_msgs::Pose wall_pose;
	wall_pose.position.x = 0.0;
	wall_pose.position.y = 0.0;
	wall_pose.position.z = -0.52;

	tf::Quaternion q = tf::createQuaternionFromRPY(0.0, 0.0,-M_PI/4.0);
	wall_pose.orientation.x = q.x();
	wall_pose.orientation.y = q.y();
	wall_pose.orientation.z = q.z();
	wall_pose.orientation.w = q.w();

	wall.primitive_poses.push_back(wall_pose);

	wall.operation = wall.ADD;
	ps.world.collision_objects.push_back(wall);

}



//Simple help function
void getPose(std::string s, double* v)
{
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

//Simpe but maybe unneccesary help function.
void getTransform(double * pose, double tf[][4])
{
	double roll = pose[3];
	double pitch = pose[4];
	double yaw = pose[5];
	
//Standard Rot matrix from RPY.
	tf[0][0] = cos(yaw)*cos(pitch);
	tf[0][1] = cos(yaw)*sin(pitch)*sin(roll) - sin(yaw)*cos(roll);
	tf[0][2] = cos(yaw)*sin(pitch)*cos(roll) + sin(yaw)*sin(roll);

	tf[1][0] = sin(yaw)*cos(pitch);
	tf[1][1] = sin(yaw)*sin(pitch)*sin(roll) + cos(yaw)*cos(roll);	
	tf[1][2] = sin(yaw)*sin(pitch)*cos(roll) - cos(yaw)*sin(roll);

	tf[2][0] = -sin(pitch);
	tf[2][1] = cos(pitch)*sin(roll);
	tf[2][2] = cos(pitch)*cos(roll);

//position (4th column)
	tf[0][3] = pose[0];
	tf[1][3] = pose[1];
	tf[2][3] = pose[2];
	tf[3][3] = 1;  

	return;
}

//Stolen from stackoverflow...
Eigen::Affine3d createRotationMatrix(double ax, double ay, double az)
{
	Eigen::Affine3d rx =
      Eigen::Affine3d(Eigen::AngleAxisd(ax, Eigen::Vector3d(1, 0, 0)));
  Eigen::Affine3d ry =
      Eigen::Affine3d(Eigen::AngleAxisd(ay, Eigen::Vector3d(0, 1, 0)));
  Eigen::Affine3d rz =
      Eigen::Affine3d(Eigen::AngleAxisd(az, Eigen::Vector3d(0, 0, 1)));
  return rz * ry * rx;
}

template <typename T> int sgn(T val) 
{
    return (T(0) < val) - (val < T(0));
}

//Simple weighted sum. 
double weightedDistance(std::vector<double> p, std::vector<double> v, double w[6])
{
	if(p.size() != v.size()){
		ROS_ERROR("Distance between states cannot be found, as states have different dimension");		
		return -1;
	}
	double sum = 0.0;
	for(int i = 0; i< p.size(); i++){
		
		sum += std::pow(p[i] - v[i], 2)*w[i];
	}
	return std::sqrt(sum); 
}

void setFromMatrix(Eigen::Matrix4d m, double tf[][4])
{
	for(int n = 0; n < 4; n++){
			for(int q = 0; q < 4; q++){
				tf[n][q] = m(n, q);
			}
	}
	return;
}


