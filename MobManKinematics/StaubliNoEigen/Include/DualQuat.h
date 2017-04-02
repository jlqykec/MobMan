#ifndef DUALQUAT_H
#define DUALQUAT_H

#include <math.h>
#include <Eigen/Dense>
#include <iostream>
#include "Quat.h"
using namespace std;

class DualQuat
{
private:
	Quat prim;
	Quat dual;
public:
	DualQuat();
	DualQuat(Quat prim, Quat dual);
	DualQuat(double w0, double x0, double y0, double z0, double wE, double xE, double yE, double zE);
	Quat getPrim();
	Quat getDual();
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
