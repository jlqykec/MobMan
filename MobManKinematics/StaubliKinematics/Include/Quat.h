#ifndef QUAT_H
#define QUAT_H

#include <math.h>
#include <Eigen/Dense>
#include <iostream>
using namespace std;

class Quat
{
private:
	double w;
	double x;
	double y;
	double z;
public:
	Quat();
	Quat(double w, double x, double y, double z);
	double getS();
	Eigen::Vector3d getV();
	Quat(double w, Eigen::Vector3d v);
	Quat operator+ (const Quat& q2);
	Quat operator- (const Quat& q2);
	Quat operator* (const Quat& q2); //Quaternion-quaternion multiplication
	friend Quat operator* (const double scalar, const Quat& q); //Scalar-quaternion multiplication
	friend Quat operator* (const Quat& q, const double scalar); //Quaternion-scalar multiplication
	Quat conj();
	static Quat rotQuat(double angle, Eigen::Vector3d axis);
	friend ostream& operator<<(ostream & os, const Quat & q);
};


#endif