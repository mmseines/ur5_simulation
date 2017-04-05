#include <boost/date_time.hpp>
#include <trac_ik/trac_ik.hpp>
#include <ros/ros.h>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl_conversions/kdl_msg.h>
#include <geometry_msgs/Pose.h>


double fRand(double min, double max)
{
  double f = (double)rand() / RAND_MAX;
  return min + f * (max - min);
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
void tourToJointPosition(ros::NodeHandle& nh, std::string chain_start, std::string chain_end, double timeout, std::string urdf_param, std::vector<geometry_msgs::Pose> tour, std::vector<KDL::JntArray>  ik_solutions)
{

  double eps = 1e-5;

	int num_samples = (int) tour.size();

  // This constructor parses the URDF loaded in rosparm urdf_param into the
  // needed KDL structures.  We then pull these out to compare against the KDL
  // IK solver.
  TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param, timeout, eps);

  KDL::Chain chain;
  KDL::JntArray ll, ul; //lower joint limits, upper joint limits

  bool valid = tracik_solver.getKDLChain(chain);
  
  if (!valid) {
    ROS_ERROR("There was no valid KDL cdhain fousssnd");
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

  for (uint j=0; j<nominal.data.size(); j++) {
    nominal(j) = (ll(j)+ul(j))/2.0;
  }    

  // Create desired number of valid, random joint configurations
  //std::vector<KDL::JntArray> ik_solutions;
  KDL::JntArray q(chain.getNrOfJoints());


  boost::posix_time::ptime start_time;
  boost::posix_time::time_duration diff;

  KDL::JntArray result;
  KDL::Frame end_effector_pose;

	std::vector<KDL::Frame> viewpoints;
	for (uint i=0; i < tour.size(); i++) {
		geometry_msgs::Pose woppa = tour[i];
		
		//DEPRECIATED: TODO make alternative. 		
		tf::poseMsgToKDL(woppa, end_effector_pose); 


		viewpoints.push_back(end_effector_pose);
  }

  int rc;

  double total_time=0;
  uint success=0;

  ROS_INFO_STREAM("*** Testing TRAC-IK with "<< tour.size()<<" viewpoints");

  for (uint i=0; i < tour.size(); i++) {
    double elapsed = 0;
    start_time = boost::posix_time::microsec_clock::local_time();
    rc=tracik_solver.CartToJnt(nominal,end_effector_pose,result);
    diff = boost::posix_time::microsec_clock::local_time() - start_time;
    elapsed = diff.total_nanoseconds() / 1e9;
    total_time+=elapsed;
    if (rc>=0){
      success++;
    	ik_solutions.push_back(result);
		}
    if (int((double)i/tour.size()*100)%10 == 0)
      ROS_INFO_STREAM_THROTTLE(1,int((i)/num_samples*100)<<"\% done");
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
	
	geometry_msgs::Pose p;
	p.orientation.x = 0.0;
	p.orientation.y = 0.0;
	p.orientation.z = 0.0;
	p.orientation.w = 1;
	p.position.x = 0.6;
	p.position.y = 0.1;
	p.position.z = 0.1;

	tour.push_back(p);


  tourToJointPosition(nh, chain_start, chain_end, timeout, urdf_param, tour, sol);

  // Useful when you make a script that loops over multiple launch files that test different robot chains
  // std::vector<char *> commandVector;
  // commandVector.push_back((char*)"killall");
  // commandVector.push_back((char*)"-9");
  // commandVector.push_back((char*)"roslaunch");
  // commandVector.push_back(NULL);  

  // char **command = &commandVector[0];
  // execvp(command[0],command);

  return 0;
}

