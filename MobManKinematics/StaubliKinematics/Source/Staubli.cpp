#include "Staubli.h"

#define _USE_MATH_DEFINES // for the PI constant
#include <math.h>
#include "PadenKahan.h"

using namespace Eigen;

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



Eigen::Matrix4d Staubli::forwardKin(const Eigen::VectorXd& q)
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
	DualQuat Q = this->dq1*this->dq2*this->dq3*this->dq4*this->dq5*this->dq6;
	DualQuat Qc = Q.conj();

	//Rotate L6 and L7 using the rigid transformation operator
	DualQuat newQL6 = Q*dqL6*Qc;
	DualQuat newQL7 = Q*dqL7*Qc;
	
	//Get the position from intersection of L6 and L7 and the directions from the vectors
	Eigen::Vector3d pos = DualQuat::linesIntPlucker(newQL6, newQL7); //Position
	Eigen::Vector3d approach = newQL6.getPrim().getV(); //Approach direction
	Eigen::Vector3d orientation = newQL7.getPrim().getV();	//Orientation direction
	Eigen::Vector3d normal = orientation.cross(approach);
		
	//Store the vectors in the T0e matrix
	this->T0e.col(0).head(3) = normal;
	this->T0e.col(1).head(3) = orientation;
	this->T0e.col(2).head(3) = approach;
	this->T0e.col(3).head(3) = pos;	

	return this->T0e;
}

Eigen::VectorXd Staubli::inverseKin(const Eigen::Matrix4d& T)
{
	//Extract the desried position, approaching direction and normal direction
	Vector3d Ped = T0e.col(3).head(3);
	Vector3d Ad = T0e.col(2).head(3);
	Vector3d Nd = T0e.col(0).head(3);

	//Create a matrix to store the solutions, the columns represent the joint values
	this->solutions = Eigen::MatrixXd::Zero(8,6);
	int solFlags[6] = { 1,1,1,1,1,1 }; //Initialize with all solutions valid
	
	// ************Step 1: Find theta 1 using the geometry of the robot**********//
	//Calculate the position of the wrist
	double th1[2];
	Vector3d Pwd = Ped - l5z * Ad;

	//Use equations (44), (45) and (46)
	double offset = 0; //There is not offset for the Staubli RX160 robot
	double alpha1 = atan2(Pwd(1), Pwd(0));
	double alpha2 = atan2(offset, sqrt(pow(Pwd(0),2)+pow(Pwd(1),2)-pow(offset,2)));
	
	th1[0] = alpha1 + alpha2;
	th1[1] = alpha1 - alpha2 + M_PI;

	//Copy the solutions to the solution matrix
	solutions.col(0) << th1[0], th1[0], th1[0], th1[0], th1[1], th1[1], th1[1], th1[1];
	// **************************************************************************//

	//*******Step 2: Find theta 2 and theta 3 using the position of joint 5******//
	//               from initial to 5_1 and subproblem2pa**********//

	//Rotation operators of theta 1
	DualQuat q1[2];
	q1[0] = DualQuat::rotOperator(th1[0], this->d1, this->m1);
	q1[1] = DualQuat::rotOperator(th1[1], this->d1, this->m1);

	//Quaternion representation of axis d2
	DualQuat dqL2 = DualQuat(Quat(0, this->d2), Quat(0, this->m2));

	//New direction of d2 after rotating theta1
	Vector3d z2_1[2];
	//Using th1[0]
	DualQuat dqL2_1;
	dqL2_1 = q1[0] * dqL2*q1[0].conj();
	z2_1[0] = dqL2_1.getPrim().getV();
	//Using th1[1]
	dqL2_1 = q1[1] * dqL2*q1[1].conj();
	z2_1[1] = dqL2_1.getPrim().getV();
	
	double th2[2], th3[2];
	Quat qr1;
	Quat qr1c;
	Vector3d p2_1, p3_1, p5_1;

	for (int i = 0; i < 2; i++)
	{
		//p2, p3 and p5 must be rotated if they change when rotating about axis d1
		//Create the rotation quaternion
		qr1 = Quat::rotQuat(th1[0], this->d1);
		qr1c = qr1.conj();
		//Rotate p2, p3 and p5
		p2_1 = Quat::rotPoint(qr1, qr1c, this->p2);
		p3_1 = Quat::rotPoint(qr1, qr1c, this->p3);
		p5_1 = Quat::rotPoint(qr1, qr1c, this->p5);

		//Use subproblem 2
		int sol = PadenKahan::subproblem2pa(z2_1[i], p5_1, Pwd, p3_1, p2_1, th2, th3);

		int index = 0;
		if (sol == 1)
		{
			//Copy one solution
			solutions.col(2).row(index) = th2[0];


		}
	}



	//Copy the solutions to the solution matrix
	solutions.col(1) << th1[0], th1[0], th1[0], th1[0], th1[1], th1[1], th1[1], th1[1];
	// **************************************************************************//
	




	cout << solutions * 180 / M_PI << endl;


	return Eigen::VectorXd();
}
