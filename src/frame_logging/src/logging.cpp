

/*
	subscribe to: /camera/depth/points/
	subscribe to: robot state.

*/
#include "ros/ros.h"
#include <stdio>
#include <sensor_msgs/Pointcloud2.h>
#include <sensor_msgs/JointState.h>


sensor_msgs::Pointcloud2 * buffer;

// Position and orientation in object frame. 

static PositionBuffer camera_position;




void frameDataCallback(const sensor_msgs::Pointcloud2 msg)
{
	/*	
		write sensor msg into a buffer.

		get current camera position.

		then if theres more than X things in the buffer, write it to file.  			
	*/
	Position current_pos = camera_position.get_currrent_state(msg.header.stamp); 
	FrameData2 new_frame(msg, pos);
	
	//frame_buffer.push_back(new_frame);

/*
	if(frame_buffer.size() > some_number){
		frame_buffer.write_to_file();
	}
 	
*/
	return;
}

void stateInfoCallback(const sensor_msgs::JointState msg)
{
	/*	
		Update current camera position.
		
		Write it to buffer class with timestamp? 

		Make that class have a get current position function.
	*/
	return;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "scan_logger");
	
	ros::NodeHandle nh;

	ros::Subscriber sub = n.subscribe("/camera/depth/points", 300, frameDataCallback);
	
	ros::Subscriber sub2 = n.subscribe("/joint_states", 1000, stateInfoCallback); 

	ros::spin();
}
