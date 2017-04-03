#include "DualQuat.h"

DualQuat::DualQuat()
{
	this->prim = Quaterniond();
	this->dual = Quaterniond();
}

DualQuat::DualQuat(Quaterniond prim, Quaterniond dual)
{
	this->prim = prim;
	this->dual = dual;
}

DualQuat::DualQuat(double w0, Vector3d v0, double wE, Vector3d vE)
{
	Quaterniond prim;
	Quaterniond dual;
	prim.w() = w0;
	prim.vec() = v0;
	dual.w() = wE;
	dual.vec() = vE;
	this->prim = prim;
	this->dual = dual;
}

DualQuat::DualQuat(double w0, double x0, double y0, double z0, double wE, double xE, double yE, double zE)
{
	this -> prim = Quaterniond(w0, x0, y0, z0);
	this -> dual = Quaterniond(wE, xE, yE, zE);
}

Quaterniond DualQuat::getPrim()
{
	return this->prim;
}

Quaterniond DualQuat::getDual()
{
	return this->dual;
}

DualQuat DualQuat::operator+(const DualQuat & dq2)
{
	Quaterniond prim;
	Quaterniond dual;
	prim = Quaterniond(this->prim.w() + dq2.prim.w(), this->prim.x() + dq2.prim.x(), this->prim.y() + dq2.prim.y(), this->prim.z() + dq2.prim.z());
	dual = Quaterniond(this->dual.w() + dq2.dual.w(), this->dual.x() + dq2.dual.x(), this->dual.y() + dq2.dual.y(), this->dual.z() + dq2.dual.z());
	return DualQuat(prim, dual);
}

DualQuat DualQuat::operator-(const DualQuat & dq2)
{
	Quaterniond prim;
	Quaterniond dual;
	prim = Quaterniond(this->prim.w() - dq2.prim.w(), this->prim.x() - dq2.prim.x(), this->prim.y() - dq2.prim.y(), this->prim.z() - dq2.prim.z());
	dual = Quaterniond(this->dual.w() - dq2.dual.w(), this->dual.x() - dq2.dual.x(), this->dual.y() - dq2.dual.y(), this->dual.z() - dq2.dual.z());
	return DualQuat(prim, dual);
}

DualQuat DualQuat::operator*(const DualQuat & q2)
{
	Quaterniond dual1;
	Quaterniond dual2;
	Quaterniond dual;
	dual1 = this->prim*q2.dual;
	dual2 = this->dual*q2.prim;
	dual = Quaterniond(dual1.w() + dual2.w(), dual1.x() + dual2.x(), dual1.y() + dual2.y(), dual1.z() + dual2.z());

	return DualQuat(this->prim*q2.prim, dual);
}

DualQuat DualQuat::rotOperator(double angle, Eigen::Vector3d d, Eigen::Vector3d m)
{
	Quaterniond prim;
	Quaterniond dual;
	prim.w() = cos(angle*0.5);
	prim.vec() = sin(angle*0.5)*d;
	dual.w() = 0.0;
	dual.vec() = sin(angle*0.5)*m;

	return DualQuat(prim, dual);
}

DualQuat DualQuat::conj()
{
	return DualQuat(this->prim.conjugate(),this->dual.conjugate());
}

Eigen::Vector3d DualQuat::linesIntPlucker(DualQuat l1, DualQuat l2)
{
	/*Eigen::Vector3d m1 = l1.getDual().getV();
	Eigen::Vector3d d1 = l1.getPrim().getV();
	Eigen::Vector3d m2 = l2.getDual().getV();
	Eigen::Vector3d d2 = l2.getPrim().getV();*/
	
	return l1.getPrim().vec().cross(l1.getDual().vec()) + l2.getPrim().vec().cross(l2.getDual().vec()).dot(l1.getPrim().vec())*l1.getPrim().vec();
}

DualQuat operator*(const double scalar, const DualQuat & q)
{
	Quaterniond prim;
	Quaterniond dual;
	prim.w() = scalar*q.prim.w();
	prim.vec() = scalar*q.prim.vec();
	dual.w() = scalar*q.dual.w();
	dual.vec() = scalar*q.dual.vec();
	return DualQuat(prim, dual);
}

DualQuat operator*(const DualQuat & q, const double scalar)
{
	return scalar*q;
}

ostream & operator<<(ostream & os, const DualQuat & q)
{
	/*	
	//First output the primary part
	os << q.prim;
	
	//Then the dual part
	cout << "+E(" << q.dual << ")";	
	*/

	return os;
}
