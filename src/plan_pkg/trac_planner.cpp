#include "include/trac_planner.hpp"




double fRand(double min, double max)
{
  double f = (double)rand() / RAND_MAX;
  return min + f * (max - min);
}

/*
	Simple parsing function.
*/
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

/*
KDL::Frame vectorToFrame(std::vector<double> v)
{
	KDL::Vector l(v[0], v[1], v[2]);
	Eigen::Affine3d r = createRotationMatrix(pose[3], pose[4], pose[5]);	
	KDL::Rotation(
	return KDL::Frame f(r, l);

}
*/
void tourToJointPosition(ros::NodeHandle& nh, std::string chain_start, std::string chain_end, double timeout, std::string urdf_param, std::vector<geometry_msgs::Pose> tour, std::vector<KDL::JntArray>  &ik_solutions)
{

  double eps = 1e-5;

	int num_samples = (int) tour.size();

  // This constructor parses the URDF loaded in rosparm urdf_param into the
  // needed KDL structures.  We then pull these out to compare against the KDL
  // IK solver.
	ROS_INFO("this runs");
  TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param, timeout, eps, TRAC_IK::Distance);

  KDL::Chain chain;
  KDL::JntArray ll, ul; //lower joint limits, upper joint limits

	ROS_INFO("this runs aswell");
  bool valid = tracik_solver.getKDLChain(chain);
  
  if (!valid) {
    ROS_ERROR("There was no valid KDL cdhain found");
    return;
  }

  valid = tracik_solver.getKDLLimits(ll,ul);

  if (!valid) {
    ROS_ERROR("There were no valid KDL joint limits found");
    return;
  }

  assert(chain.getNrOfJoints() == ll.data.size());
  assert(chain.getNrOfJoints() == ul.data.size());

  ROS_INFO ("Using %d joints",chain.getNrOfJoints());


  // Set up KDL IK
  KDL::ChainFkSolverPos_recursive fk_solver(chain); // Forward kin. solver
  KDL::ChainIkSolverVel_pinv vik_solver(chain); // PseudoInverse vel solver
  KDL::ChainIkSolverPos_NR_JL kdl_solver(chain,ll,ul,fk_solver, vik_solver, 1, eps); // Joint Limit Solver
  // 1 iteration per solve (will wrap in timed loop to compare with TRAC-IK) 


  // Create Nominal chain configuration midway between all joint limits
  KDL::JntArray nominal(chain.getNrOfJoints());

	nominal(0) = 0.0;
	nominal(1) = -M_PI/2;
	nominal(2) = 0.0;
	nominal(3) = 0.0;
	nominal(4) = 0.0;
	nominal(5) = -M_PI; 	
	/*
  for (uint j=0; j<nominal.data.size(); j++) {
    nominal(j) = (ll(j)+ul(j))/2.0;
  } 
	*/   

  // Create desired number of valid, random joint configurations
  //std::vector<KDL::JntArray> ik_solutions;
  KDL::JntArray q(chain.getNrOfJoints());


  boost::posix_time::ptime start_time;
  boost::posix_time::time_duration diff;

  KDL::JntArray result;
	

	std::vector<KDL::Frame> viewpoints;
	for (uint i=0; i < tour.size(); i++) {
		KDL::Rotation rot = KDL::Rotation::Quaternion(tour[i].orientation.x, tour[i].orientation.y, tour[i].orientation.z, tour[i].orientation.w);  
		KDL::Vector transl(tour[i].position.x, tour[i].position.y, tour[i].position.z);
		KDL::Frame fam(rot, transl);

		viewpoints.push_back(fam);
  }

  int rc;

  double total_time=0;
  uint success=0;

  ROS_INFO_STREAM("*** Testing TRAC-IK with "<< tour.size()<<" viewpoints");

  for (uint i=0; i < tour.size(); i++) {
		
		KDL::Frame end_effector_pose = viewpoints[i];
    double elapsed = 0;
    start_time = boost::posix_time::microsec_clock::local_time();
    rc=tracik_solver.CartToJnt(nominal,end_effector_pose,result); //Må kanskje stille på hvordan end effector pose brukes. 
    diff = boost::posix_time::microsec_clock::local_time() - start_time;
    elapsed = diff.total_nanoseconds() / 1e9;
    total_time+=elapsed;
    if (rc>=0){
      success++;
			KDL::JntArray tmp = result;
    	ik_solutions.push_back(tmp);
		}
    if (int((double)i/tour.size()*100)%10 == 0)
      ROS_INFO_STREAM_THROTTLE(1,int((i)/num_samples)<<"\% done");
  }

  ROS_INFO_STREAM("TRAC-IK found "<<success<<" solutions ("<<100.0*success/num_samples<<"\%) with an average of "<<total_time/num_samples<<" secs per sample");

	return;

}



int main(int argc, char** argv)
{
  srand(1);
  ros::init(argc, argv, "ik_tests");
  ros::NodeHandle nh("~");

	ros::AsyncSpinner spinner(1);
	spinner.start();


  int num_samples;
  std::string chain_start, chain_end, urdf_param;
  double timeout;

  nh.param("num_samples", num_samples, 1000);
  nh.param("chain_start", chain_start, std::string(""));
  nh.param("chain_end", chain_end, std::string(""));
  
  if (chain_start=="" || chain_end=="") {
    ROS_FATAL("Missing chain info in launch file");
    exit (-1);
  }


  nh.param("timeout", timeout, 0.005);
  nh.param("urdf_param", urdf_param, std::string("/robot_description"));

  if (num_samples < 1)
    num_samples = 1;

	std::vector<KDL::JntArray> sol;
	std::vector<geometry_msgs::Pose> tour;
	
	std::ifstream f("/home/magnus/Documents/path_ctrl/src/plan_pkg/paths/latestPath.csv");
	if( !f.is_open()){
		ROS_ERROR("Could not open path, exiting.."); 
		return 0;
	}
	ROS_INFO("file is open");
	std::string line;	
	while(std::getline(f, line) && ros::ok()){
		double pose[6];
		getPose(line, pose);		

		tf::Quaternion qt = tf::createQuaternionFromRPY(pose[3], pose[4],pose[5]);

		geometry_msgs::Pose p;
		p.orientation.x = qt.x();
		p.orientation.y = qt.y();
		p.orientation.z = qt.z();
		p.orientation.w = qt.w();
		p.position.x = pose[0]/100;
		p.position.y = pose[1]/100;
		p.position.z = pose[2]/100;
		
		tour.push_back(p);

	}

  tourToJointPosition(nh, chain_start, chain_end, timeout, urdf_param, tour, sol);

	
	int succ = moveRobot(nh, sol); // moves the robot through the given path. by point to point planning.
	if(succ != 1){
		ROS_ERROR("Nopeti nope nope");
	}
  return 0;
}

/*
	Function for moving the robot.

*/

// TODO : this appears to either not move or have the same solution to everything. 

int moveRobot(ros::NodeHandle& nh, const std::vector<KDL::JntArray>  &ik_solutions){
	
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
	
	planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
	robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(robot_model));
	kinematic_state->setToDefaultValues();
	const robot_state::JointModelGroup * joint_model_group = robot_model->getJointModelGroup("manipulator");
	
	std::vector<double> joint_config;

	boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > planner_plugin_loader;
  planning_interface::PlannerManagerPtr planner_instance;

  std::string planner_plugin_name = "ompl_interface/OMPLPlanner";

  // Get name from the ROS param server, and then load the planner
  if (!nh.getParam("planning_plugin", planner_plugin_name))
    ROS_FATAL_STREAM("Could not find planner plugin name");
  try{
    planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
  }catch(pluginlib::PluginlibException& ex){
    ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
  }try{
    planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
    if (!planner_instance->initialize(robot_model, nh.getNamespace()))
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

	planning_interface::MotionPlanRequest req;
	req.group_name = "manipulator";
	planning_interface::MotionPlanResponse res;
	moveit_msgs::MotionPlanResponse response;

	moveit::planning_interface::MoveGroup group("manipulator");


	double tolerance_below = 1e-3;
	double tolerance_above = 1e-3;
	req.workspace_parameters.min_corner.z = -0.05;
	req.workspace_parameters.min_corner.x = req.workspace_parameters.min_corner.y = -0.7;
	req.workspace_parameters.max_corner.x = req.workspace_parameters.max_corner.y = req.workspace_parameters.max_corner.z =  0.7;


	//TODO : find reason for seemingly 322 identical solutions from trac_ik. 	

	for(int i = 0; i < ik_solutions.size(); i++){
			KDL::JntArray pablo = ik_solutions[i];
			joint_config.clear();			
			for(int q = 0; q<6; q++){
				joint_config.push_back(pablo(q));
			}	
			planning_scene->setCurrentState(*group.getCurrentState());			

			robot_state::RobotState goal_state(robot_model);
			goal_state.setJointGroupPositions(joint_model_group, joint_config);
  		moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group, tolerance_below, tolerance_above);
  		req.goal_constraints.clear();
			req.goal_constraints.push_back(joint_goal);				

			/* Call the Planner */
			planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
  		context->solve(res);

  		if(res.error_code_.val != res.error_code_.SUCCESS)
  		{
   			ROS_ERROR("Could not compute plan sucsessfully for wp: %i", i);
				return 0;
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
				
				//And this works somehow.
				group.execute(plan);

			}
	}
	return 1;
}


