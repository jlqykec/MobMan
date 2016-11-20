#ifndef MMUR5_H
#define MMUR5_H

#include <Eigen/Dense>
#include <iostream>
#include <math.h>
#include "Quat.h"
#include "DualQuat.h"

class MMUR5
{
private:
	//Length of the links
	double l1, l2, l3, l4, l5, l6;
	
	//Position of the base of the robot with respect to the center of the wheels
	double a, b;
	
	//Points and Directions of each joint
	Eigen::Vector3d p1, p2, p3, p4, p5, p6, p7, p8;
	Eigen::Vector3d d1, d2, d3, d4, d5, d6, d7, d8;

	//Moments of the axes
	Eigen::Vector3d m1, m2, m3, m4, m5, m6, m7, m8;

	//Dual-quaternion representation of each axis of the UR5
	DualQuat dq1, dq2, dq3, dq4, dq5, dq6;

	//Dual-quaternion representation of the mobile platform
	DualQuat dqmp;

	//Dual-quaternion representation of the axis that represent the tool
	DualQuat dqL7;
	DualQuat dqL8;

	//Current position and orientation matrix
	Eigen::Matrix4d T0e;
public:
	MMUR5(); //Default constructor uses the real dimensions of the robot
	Eigen::Matrix4d forwardKin(Eigen::VectorXd q);
};




#endif
