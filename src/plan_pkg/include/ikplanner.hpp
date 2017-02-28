/*!
	* \file ikplanner.hpp
	*
	* Decleration of help functions for moveit interface.
*/

#ifndef __IKPLANNER_HPP__
#define __IKPLANNER_HPP__


#include<ros/ros.h>
#include<fstream>
#include<string>
#include <cstdlib>
#include <ros/package.h>
#include "tf/tf.h"


/* 
	Help function from reading pose from text(.csv) file. 
		takes pose in string format
		returns pose in double values by reference.
*/
void getPose(std::string, double*);

/*
	Translates a tool pose to a 4x4 D-H transformation matrix.	
*/
void getTransform(double*, double [][4]);


#endif
