#include "Quat.h"
using namespace std;

Quat::Quat()
{
	this->w = 0;
	this->x = 0;
	this->y = 0;
	this->z = 0;
}

Quat::Quat(double w, double x, double y, double z)
{
	this->w = w;
	this->x = x;
	this->y = y;
	this->z = z;
}

Quat::Quat(double w, Eigen::Vector3d v)
{
	this->w = w;
	this->x = v(0);
	this->y = v(1);
	this->z = v(2);
}

Quat Quat::operator+ (const Quat& q2)
{
	return Quat(this->w + q2.w,
				this->x + q2.x,
				this->y + q2.y,
				this->z + q2.z);
}

Quat Quat::operator-(const Quat & q2)
{
	return Quat(this->w - q2.w,
		this->x - q2.x,
		this->y - q2.y,
		this->z - q2.z);
}

Quat Quat::operator*(const Quat & q2)
{
	return Quat(this->w*q2.w-this->x*q2.x-this->y*q2.y-this->z*q2.z,
		this->w*q2.x + this->x*q2.w + this->y*q2.z - this->z*q2.y,
		this->w*q2.y + this->y*q2.w + this->z*q2.x - this->x*q2.z,
		this->w*q2.z + this->z*q2.w + this->x*q2.y - this->y*q2.x);
}

Quat Quat::conj()
{
	return Quat(this->w,-1*this->x,-1*this->y,-1*this->z);
}

Quat Quat::rotQuat(double angle, Eigen::Vector3d axis)
{
	return Quat(cos(angle * 0.5), sin(angle * 0.5)*axis);
}

Vector3d Quat::rotPoint(Quat rotq, Quat rotqc, Vector3d p)
{
	Quat qp = Quat(0,p);
	Quat qp_rot = rotq*qp*rotqc;
	return qp_rot.getV();
}

double Quat::getS()
{
	return this->w;
}

Eigen::Vector3d Quat::getV()
{
	return Eigen::Vector3d(this->x,this->y,this->z);
}

Quat operator*(const double scalar, const Quat & q)
{
	return Quat(scalar*q.w, scalar*q.x, scalar*q.y, scalar*q.z);
}

Quat operator*(const Quat & q, const double scalar)
{
	return Quat(scalar*q.w, scalar*q.x, scalar*q.y, scalar*q.z);
}

ostream & operator<<(ostream & os, const Quat & q)
{
	os << q.w;
	if (q.x >= 0.0)
		os << "+";
	os << q.x <<"i";
	if (q.y >= 0.0)
		os << "+";
	os << q.y << "j";
	if (q.z >= 0.0)
		os << "+";
	os << q.z << "k";
	return os;
}
