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
#include <Eigen/Geometry>


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

Eigen::Affine3d create_rotation_matrix(double ax, double ay, double az);

/*
	standard sign function
*/
template <typename T> int sgn(T val);

/*
	Weigthed distance between two vectors. 
*/
double weighted_distance(std::vector<double> p, std::vector<double> v);


#endif
