#ifndef QUAT_H
#define QUAT_H

#include <math.h>
#include <Eigen/Dense>
#include <iostream>
using namespace std;
using namespace Eigen;

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
	Vector3d getV();
	Quat(double w, Vector3d v);
	Quat operator+ (const Quat& q2);
	Quat operator- (const Quat& q2);
	Quat operator* (const Quat& q2); //Quaternion-quaternion multiplication
	friend Quat operator* (const double scalar, const Quat& q); //Scalar-quaternion multiplication
	friend Quat operator* (const Quat& q, const double scalar); //Quaternion-scalar multiplication
	Quat conj();
	static Quat rotQuat(double angle, Vector3d axis);
	static Vector3d rotPoint(Quat rotq, Quat rotqc, Vector3d p);
	friend ostream& operator<<(ostream & os, const Quat & q);
};


#endif