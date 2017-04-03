#ifndef DUALQUAT_H
#define DUALQUAT_H

#include <math.h>
#include <Eigen/Dense>
#include <iostream>
#include "Quat.h"
using namespace std;
using namespace Eigen;

class DualQuat
{
private:
	Quaterniond prim;
	Quaterniond dual;
public:
	DualQuat();
	DualQuat(Quaterniond prim, Quaterniond dual);
	DualQuat(double w0, Vector3d v0, double wE, Vector3d vE);
	DualQuat(double w0, double x0, double y0, double z0, double wE, double xE, double yE, double zE);
	Quaterniond getPrim();
	Quaterniond getDual();
	DualQuat operator+ (const DualQuat& dq2);
	DualQuat operator- (const DualQuat& dq2);
	DualQuat operator* (const DualQuat& q2); //Dualquaternion-Dualquaternion multiplication
	static DualQuat rotOperator(double angle, Eigen::Vector3d d, Eigen::Vector3d m); //Dual quaternion rotation operator
	friend DualQuat operator* (const double scalar, const DualQuat& q); //Scalar-Dualquaternion multiplication
	friend DualQuat operator* (const DualQuat& q, const double scalar); //Dualquaternion-scalar multiplication
	DualQuat conj();
	friend ostream& operator<<(ostream & os, const DualQuat & q);

	//Intersection of two lines in Plucker Coordinates
	static Eigen::Vector3d linesIntPlucker(DualQuat l1, DualQuat l2);
};


#endif
