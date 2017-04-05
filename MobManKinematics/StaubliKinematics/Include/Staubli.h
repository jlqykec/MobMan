#ifndef STAUBLI_H
#define STAUBLI_H

#include <Eigen/Dense>
#include <iostream>
#include <math.h>
#include "Quat.h"
#include "DualQuat.h"

class Staubli
{
private:
	//Length of the links
	double l0z, l1x, l1y, l2z, l4z, l5z;
	
	//Position of the base of the robot with respect to the center of the wheels
	double a, b;
	
	//Points and Directions of each joint
	Eigen::Vector3d p1, p2, p3, p4, p5, p6, p7, p8;
	Eigen::Vector3d d1, d2, d3, d4, d5, d6, d7, d8;

	//Moments of the axes
	Eigen::Vector3d m1, m2, m3, m4, m5, m6, m7, m8;

	//Dual-quaternion representation of each axis of the UR5
	DualQuat dq1, dq2, dq3, dq4, dq5, dq6;

	//Dual-quaternion representation of the needed axes for Plucker Coordinates
	DualQuat dqL4;	
	DualQuat dqL5;	
	DualQuat dqL6;	//approach direction
	DualQuat dqL7;	//orientation direction
	DualQuat dqL8;	//auxiliar axis for inverse kinematics


	//Current position and orientation matrix
	Eigen::Matrix4d T0e;

	//Solutions from inverse kinematics
	Eigen::MatrixXd solutions;
	//int solFlags[8];

public:
	Staubli(); //Default constructor uses the real dimensions of the robot
	Eigen::Matrix4d forwardKin(const Eigen::VectorXd& q);
	Eigen::MatrixXd inverseKin(const Eigen::Matrix4d& T, int* solFlags);
protected:
	Vector3d LinesIntersection(DualQuat dqL1, DualQuat dqL2);
};




#endif
