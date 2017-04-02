#include "Staubli.h"

Staubli::Staubli()
{
	//Lenghts of the links
	this->l0z =0.55;
	this->l1x = 0.15;
	this->l1y = 0.3;
	this->l2z = 0.825;
	this->l4z = 0.625;
	this->l5z = 0.11;


	//Points and Directions of each joint
	this->p1 << 0, 0, this->l0z;
	this->p2 << this->l1x, -1 * this->l1y, this->l0z;
	this->p3 << this->l1x, -1 * this->l1y, this->l0z + this->l2z;
	this->p4 << this->l1x, 0, this->l0z + this->l2z;
	this->p5 << this->l1x, 0, this->l0z + this->l2z + this->l4z;
	this->p6 << this->l1x, 0, this->l0z + this->l2z + this->l4z + this->l5z;
	this->d1 << 0, 0, 1;
	this->d2 << 0, 1, 0;
	this->d3 << 0, 1, 0;
	this->d4 << 0, 0, 1;
	this->d5 << 0, 1, 0;
	this->d6 << 0, 0, 1;

	//Auxiliar line for tool position and orientation
	this->p7 << this->p6;
	this->d7 << 0, 1, 0;
	//Auxiliar line for tool position and approach
	this->p8 = this->p6 + 5 * this->d7;
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
	this->dqL6 = DualQuat(Quat(0, this->d6), Quat(0, this->m6));
	this->dqL7 = DualQuat(Quat(0, this->d7), Quat(0, this->m7));
	this->dqL8 = DualQuat(Quat(0, this->d8), Quat(0, this->m8));

	//Initialize the values of the Transformaion matrix
	this->T0e = Eigen::Matrix4d::Zero();
	this->T0e(3, 3) = 1.0;
}

Eigen::Matrix4d Staubli::forwardKin(Eigen::VectorXd q)
{
	if (q.size() != 6) //The size of the 
	{
		throw "The angles of the 6 joints are required";
	}

	//Create the dual quaternions to represent each joint axis
	this->dq1 = DualQuat::rotOperator(q(0), this->d1, this->m1);
	this->dq2 = DualQuat::rotOperator(q(1), this->d2, this->m2);
	this->dq3 = DualQuat::rotOperator(q(2), this->d3, this->m3);
	this->dq4 = DualQuat::rotOperator(q(3), this->d4, this->m4);
	this->dq5 = DualQuat::rotOperator(q(4), this->d5, this->m5);
	this->dq6 = DualQuat::rotOperator(q(5), this->d6, this->m6);
	

	//******************** Rigid Transfomation ******************//
	this->Q = this->dq1*this->dq2*this->dq3*this->dq4*this->dq5*this->dq6;
	this->Qc = this->Q.conj();

	//Rotate L6 and L7 using the rigid transformation operator
	this->newQL6 = this->Q*this->dqL6*this->Qc;
	this->newQL7 = this->Q*this->dqL7*this->Qc;
	
	/*
	//Get the position from intersection of L6 and L7 and the directions from the vectors
	Eigen::Vector3d pos = DualQuat::linesIntPlucker(newQL6, newQL7); //Position
	Eigen::Vector3d approach = newQL6.getPrim().getV(); //Approach direction
	Eigen::Vector3d orientation = newQL7.getPrim().getV();	//Orientation direction
	Eigen::Vector3d normal = orientation.cross(approach);*/
	
	//Store the vectors in the T0e matrix
	this->T0e.col(0).head(3) = newQL7.getPrim().getV().cross(newQL6.getPrim().getV());
	this->T0e.col(1).head(3) = newQL7.getPrim().getV();	//Orientation direction
	this->T0e.col(2).head(3) = newQL6.getPrim().getV(); //Approach direction
	this->T0e.col(3).head(3) = DualQuat::linesIntPlucker(newQL6, newQL7); //Position	

	return this->T0e;
}

Eigen::VectorXd Staubli::inverseKin(Eigen::Matrix4d T)
{
	return Eigen::VectorXd();
}
