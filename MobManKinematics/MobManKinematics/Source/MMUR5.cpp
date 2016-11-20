#include "MMUR5.h"

MMUR5::MMUR5()
{
	//Lenghts of the links
	this->l1 = 0.0892;
	this->l2 = 0.425;
	this->l3 = 0.392;
	this->l4 = 0.1093;
	this->l5 = 0.09475;
	this->l6 = 0.0825;
	//Position of the base of the robot with respect to the center of the wheels
	this->a = -0.01;
	this->b = 0.06;

	//Points and Directions of each joint (Considering the initial position of the UR5 robot)
	this->p1 << this->a, 0, this->l1;
	this->p2 << this->a, 0, this->l1;
	this->p3 << this->a - this->l2, 0, this->l1; 
	this->p4 << this->a - this->l2 - this->l3, 0, this->l1;
	this->p5 << this->a - this->l2 - this->l3, -this->l4, this->l1;
	this->p6 << this->a - this->l2 - this->l3, -this->l4, this->l1 - this->l5;
	this->d1 << 0, 0, 1;
	this->d2 << 0, -1, 0;
	this->d3 << 0, -1, 0;
	this->d4 << 0, -1, 0;
	this->d5 << 0, 0, -1;
	this->d6 << 0, -1, 0;

	//Auxiliar line for tool position and normal direction
	this->p7 << this->p6 + this->d6 * this->l6;
	this->d7 << 1, 0, 0;
	//Auxiliar line for tool position and approach direction
	this->p8 = this->p6;
	this->d8 = this->d6;
	
	//Get the moments of the axes for the plucker coordinates
	this->m1 = this->p1.cross(d1);
	this->m2 = this->p2.cross(d2);
	this->m3 = this->p3.cross(d3);
	this->m4 = this->p4.cross(d4);
	this->m5 = this->p5.cross(d5);
	this->m6 = this->p6.cross(d6);
	this->m7 = this->p7.cross(d7);
	this->m8 = this->p8.cross(d8);

	//Dual quaternion representation of the axis that represent the tool in the initial position
	this->dqL7 = DualQuat(Quat(0, this->d7), Quat(0, this->m7));
	this->dqL8 = DualQuat(Quat(0, this->d8), Quat(0, this->m8));

	//Initialize the values of the Transformaion matrix
	this->T0e = Eigen::Matrix4d::Zero();
	this->T0e(3, 3) = 1.0;
}

Eigen::Matrix4d MMUR5::forwardKin(Eigen::VectorXd q)
{
	if (q.size() != 10) //The size of the 
	{
		throw "The forward kinematics requires the 10DOF values";
	}
	//Create the dual quaternion to represent each joint axis

	//*********** Mobile platform and prismatic joint ***********//
	Eigen::Vector3d t(q(0), q(1), q(3));	//tx ty tz
	//Translation and rotation quaternions
	Quat qrot = Quat::rotQuat(q(2),Eigen::Vector3d(0,0,1));
	Quat qt(0,t);

	//Composite operator of rotation and translation of the mobile platform
	this->dqmp = DualQuat(qrot,0.5*qt*qrot);

	//************************** UR5 ****************************//
	this->dq1 = DualQuat(Quat(cos(q(4)*0.5), sin(q(4)*0.5)*this->d1), Quat(0, sin(q(4)*0.5)*this->m1));
	this->dq2 = DualQuat(Quat(cos(q(5)*0.5), sin(q(5)*0.5)*this->d2), Quat(0, sin(q(5)*0.5)*this->m2));
	this->dq3 = DualQuat(Quat(cos(q(6)*0.5), sin(q(6)*0.5)*this->d3), Quat(0, sin(q(6)*0.5)*this->m3));
	this->dq4 = DualQuat(Quat(cos(q(7)*0.5), sin(q(7)*0.5)*this->d4), Quat(0, sin(q(7)*0.5)*this->m4));
	this->dq5 = DualQuat(Quat(cos(q(8)*0.5), sin(q(8)*0.5)*this->d5), Quat(0, sin(q(8)*0.5)*this->m5));
	this->dq6 = DualQuat(Quat(cos(q(9)*0.5), sin(q(9)*0.5)*this->d6), Quat(0, sin(q(9)*0.5)*this->m6));

	//******************** Rigid Transfomation ******************//
	DualQuat Q = this->dqmp*this->dq1*this->dq2*this->dq3*this->dq4*this->dq5*this->dq6;
	DualQuat Qc = Q.conj();

	//Get the new positions and orientation of L7 and L8
	DualQuat newQL7 = Q*dqL7*Qc;
	DualQuat newQL8 = Q*dqL8*Qc;

	//Get the position from intersection of L7 and L8 and the directions from the vectors
	Eigen::Vector3d pos = DualQuat::linesIntPlucker(newQL7, newQL8); //Position
	Eigen::Vector3d approach = newQL8.getPrim().getV(); //Approach direction
	Eigen::Vector3d normal = newQL7.getPrim().getV(); //Normal direction
	Eigen::Vector3d orientation = approach.cross(normal);
	
	//Store the vectors in the T0e matrix
	this->T0e.col(0).head(3) = normal;
	this->T0e.col(1).head(3) = orientation;
	this->T0e.col(2).head(3) = approach;
	this->T0e.col(3).head(3) = pos;	

	return this->T0e;
}
