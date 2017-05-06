/*
	blah blah... logging node.
	
*/

#include "ros/ros.h"
#include <stdio>
#include <sensor_msgs/Pointcloud2.h>


// Position and orientation in object frame. 
class Position
{
public:
	//timestamp
	double x,y,z,yaw,pitch,roll;
	Position(Position p) : x(p.x), y(p.x), z(p.x), yaw(p.yaw), pitch(p.pitch), roll(p.roll)
	Position(double ix, double iy, double iz, double iyaw, double ipitch, double iroll) : x(ix), y(iy), z(iz), yaw(iyaw), pitch(ipicth), roll(iroll) {}  
}

class FrameData2
{
public:
	sensor_msgs::Pointcloud2 pc;
	Position pos;
	FrameData2(sensor_msgs::Pointcloud2 c, Position p) : pc(c), pos(p) {}
}

class FrameBuffer
{
public:
	int size();
	void push_back();
private:
	FrameData2 buffer[30];

}

class PositionBuffer
{
public:
	Position get_current_state(uint_16 timestamp);
	void push_back(Position pos);	
private:
	int iterator = 0;
	Position buffer[20];
}


Position PositionBuffer::get_current_state(uint_16 ts){
	/*	
		Iterate over buffer to find the closest state to the timestamp. And interpolate it linearly towards the second closest state or 
		just 
	*/
	
	Position pos;

	return pos;
}

void PositionBuffer::push_back(Position pos){
	//insert into buffer.
	return;
}
 


