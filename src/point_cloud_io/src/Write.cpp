/*
 * Write.cpp
 *
 *  Created on: Nov 13, 2015
 *      Author: Remo Diethelm
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "point_cloud_io/Write.hpp"

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/io/file_io.h>

#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>




using namespace std;
using namespace ros;
using namespace pcl;
using namespace pcl::io;

//extern point_cloud_io::RobotState stateMonitor;

namespace point_cloud_io {

Write::Write(ros::NodeHandle& nodeHandle, RobotState * rs_ptr)
    : nodeHandle_(nodeHandle),
      filePrefix_("point_cloud"),
      fileEnding_("ply"),
      rs(rs_ptr)
{
  if (!readParameters()) ros::requestShutdown();
  ROS_INFO_STREAM("Subscribed to topic \"" << pointCloudTopic_ << "\".");
  pointCloudSubscriber_ = nodeHandle_.subscribe(pointCloudTopic_, 1, &Write::pointCloudCallback, this);
  myfile.open("/home/magnusms/Documents/pointclouds/stamps.txt");
  start_time = ros::Time::now();
  myfile << start_time << "\n";
}

Record::Record(ros::NodeHandle& nodeHandle, RobotState * rs_ptr)
    : nodeHandle_(nodeHandle), rs(rs_ptr)
{
  ROS_INFO_STREAM("Subscribed to topic \"" << "/joint_states"<< "\".");
  jointStateSubscriber_ = nodeHandle_.subscribe("/joint_states", 1, &Record::jointStateCallback, this);
  myfile.open("/home/magnus/Documents/robot_state_log/stream.txt");
  if(!myfile.is_open() ){
    ROS_ERROR("Could not open ");
  }
  start_time = ros::Time::now();
  myfile << start_time << "\n";
}
Record::~Record()
{
  myfile.close();
}

Write::~Write()
{
  myfile.close();
}

bool Write::readParameters()
{
  bool allParametersRead = true;
  if (!nodeHandle_.getParam("topic", pointCloudTopic_)) allParametersRead = false;
  if (!nodeHandle_.getParam("folder_path", folderPath_)) allParametersRead = false;
  nodeHandle_.getParam("file_prefix", filePrefix_);
  nodeHandle_.getParam("file_ending", fileEnding_);
  nodeHandle_.getParam("add_counter_to_path", addCounterToPath_);
  nodeHandle_.getParam("add_frame_id_to_path", addFrameIdToPath_);
  nodeHandle_.getParam("add_stamp_sec_to_path", addStampSecToPath_);
  nodeHandle_.getParam("add_stamp_nsec_to_path", addStampNSecToPath_);

  if (!allParametersRead)
  {
    ROS_WARN("Could not read all parameters. Typical command-line usage:\n"
        "rosrun point_cloud_io write"
        " _topic:=/my_topic"
        " _folder_path:=/home/user/my_point_clouds"
        " (optional: _file_prefix:=my_prefix"
                   " _file_ending:=my_ending"
                   " _add_counter_to_path:=true/false"
                   " _add_frame_id_to_path:=true/false"
                   " _add_stamp_sec_to_path:=true/false"
                   " _add_stamp_nsec_to_path:=true/false)");
    return false;
  }

  return true;
}

void Write::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  ROS_INFO_STREAM("Received point cloud with " << cloud->height*cloud->width << " points.");
  std::cout << folderPath_ << std::endl;
  stringstream filePath;
  filePath << folderPath_ << "/";
  myfile << counter_;
  myfile << " " << cloud->header.frame_id;
  myfile << " " << (cloud->header.stamp.toSec() - start_time.toSec() ) << "\n";

  if (!filePrefix_.empty()) {
    filePath << filePrefix_;
  }
  if (addCounterToPath_) {
    filePath << "_" << counter_;
    counter_++;
  }
  if (addFrameIdToPath_) {
    filePath << "_" << cloud->header.frame_id;
  }
  if (addStampSecToPath_) {
    filePath << "_" << cloud->header.stamp.sec;
  }
  if (addStampNSecToPath_) {
    filePath << "_" << cloud->header.stamp.nsec;
  }
  filePath << ".";
  filePath << fileEnding_;

  if (fileEnding_ == "ply") {
    // Write .ply file.
    PointCloud<PointXYZRGBNormal> pclCloud;
    fromROSMsg(*cloud, pclCloud);
    PointCloud<PointXYZRGBNormal> pclCloud2;
    std::vector<int> index;
    removeNaNFromPointCloud(pclCloud, pclCloud2, index);

    //there is now a pcl cloud. 
    //Eigen::Affine3d transform = rs->getState(); // returns transform from base to endeffector.
    //transform = transform.inverse();

    //PointCloud<PointXYZRGBNormal>::Ptr transformed_cloud (new pcl::PointCloud<PointXYZRGBNormal> ());
  // You can either apply transform_1 or transform_2; they are the same
    //transformPointCloud(pclCloud2, *transformed_cloud, transform);
    PLYWriter writer;
    if (writer.write(filePath.str(), pclCloud2) != 0) {
      ROS_ERROR("Something went wrong when trying to write the point cloud file.");
      return;
    }
  }
  else {
    ROS_ERROR_STREAM("Data format not supported.");
    return;
  }

  ROS_INFO_STREAM("Saved point cloud to " << filePath.str() << ".");
}

void Record::jointStateCallback(const sensor_msgs::JointState & msg){

// .... hello darkness my old friend... This took WAAAY to loong to figure out, also why? why? WHYYYY would this be a thing?
	gettimeofday(stop_t, NULL);
	long time_between_frames = (stop_t.tv_sec - start_t.tv_sec)*1000000 +  stop_t.tv_usec - start_t.tv_usec;
	double fps = 1000/(time_between_frames);
	ROS_INFO("logging with %f FPS");
	gettimeofday(start_t, NULL);
	sensor_msgs::JointState lul = msg;
	lul.position[0] = msg.position[2];
	lul.position[2] = msg.position[0]; 
	for(int i = 0; i < 6; i++){
		ROS_INFO( (msg.name[i] + "\n").c_str());
	}	
// ---- end fix to get correct order on joint values from message. 
	rs->setState(lul);

  long time_stamp = msg.header.stamp.toSec() - start_time.toSec();
  myfile << time_stamp << " ";
  for(int i = 0; i < 6; i++){
    myfile << msg.position[i] << " ";
  }
  Eigen::Affine3d trans = rs->getState();
	Eigen::Matrix3d mat = trans.rotation();
/*  
	Eigen::Matrix4d mat = trans.matrix();
  for(int i = 0; i <4; i++){
    for(int j = 0; j< 4; j++){
      myfile << mat(i,j) << " ";
    }
  }
*/
	for(int i = 0; i < 3; i++){
		myfile << (trans.translation())[i] << " ";
	}
	for(int i = 0; i < 3; i++){
		myfile << (mat.eulerAngles(0,1,2))[i] << " ";
	}
	
	myfile << "\n";
  return;
}


RobotState::RobotState() : robot_ml("robot_description")
{
  kinematic_mdl = robot_ml.getModel();
  ROS_INFO("Model frame: %s", kinematic_mdl->getModelFrame().c_str());

}

Eigen::Affine3d RobotState::getState()
{
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_mdl));
  //kinematic_state->setToDefaultValues();
  const robot_state::JointModelGroup* joint_model_group = kinematic_mdl->getJointModelGroup("manipulator");

	/*
  std::vector<double> joint_values;
  for(int i = 0; i < 6; i++){
    joint_values.push_back(current_state.position[i]);
  }
	*/

  kinematic_state->setJointGroupPositions(joint_model_group, current_state.position);
	kinematic_state->update();

  Eigen::Affine3d pose = kinematic_state->getGlobalLinkTransform("tool_tip");
	
  return pose;
}

void RobotState::setState(sensor_msgs::JointState msg)
{
  current_state = msg;
}

} /* namespace */
