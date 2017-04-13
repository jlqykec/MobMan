#include "Staubli.h"

#define _USE_MATH_DEFINES // for the PI constant
#include <math.h>
#include "PadenKahan.h"

using namespace std;
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
	this->d7 << 1, 0, 0;
	//Auxiliar line for tool position and approach
	this->p8 = this->p6 + 5.0 * this->d7;
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

	//Dual quaternion representation of the needed axes in Plucker Coordinates
	this->dqL4 = DualQuat(Quat(0, this->d4), Quat(0, this->m4));
	this->dqL5 = DualQuat(Quat(0, this->d5), Quat(0, this->m5));
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
	Eigen::Vector3d normal = newQL7.getPrim().getV();	//Orientation direction
	Eigen::Vector3d orientation = normal.cross(approach);
		
	//Store the vectors in the T0e matrix
	this->T0e.col(0).head(3) = normal;
	this->T0e.col(1).head(3) = orientation;
	this->T0e.col(2).head(3) = approach;
	this->T0e.col(3).head(3) = pos;	

	return this->T0e;
}

MatrixXd Staubli::inverseKin(const Eigen::Matrix4d& T, int* solFlags)
{
	//Extract the desried position, approaching direction and normal direction
	Vector3d Ped = T0e.col(3).head(3);
	Vector3d Ad = T0e.col(2).head(3);
	Vector3d Nd = T0e.col(0).head(3);

	//Create a matrix to store the solutions, the columns represent the joint values
	this->solutions = Eigen::MatrixXd::Zero(8,6);
	for (int i=0;i<8;i++)
	{
		solFlags[i] = 1; //Initialize with all solutions valid
	}
	
	
	// ************Step 1: Find theta 1 using the geometry of the robot**********//
	// **************************************************************************//
	// **************************************************************************//

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
	//---------------------------------------------------------------------------//


	//*******Step 2: Find theta 2 and theta 3 using the position of joint 5******//
	//*******from initial to 5_1 and subproblem2pa*******************************//
	// **************************************************************************//

	//Rotation operators of theta 1
	DualQuat q1[2];	
	
	double th2[2], th3[2];
	Quat qr1, qr1c;
	Vector3d p2_1, p3_1, p5_1, d2[2];

	for (int i = 0; i < 2; i++)
	{
		//Rotation operators of theta 1 (Used in next steps)
		q1[i] = DualQuat::rotOperator(th1[i], this->d1, this->m1);

		//p2, p3, p5 and z2 must be rotated if they change when rotating about axis d1
		//Create the rotation quaternion
		qr1 = Quat::rotQuat(th1[i], this->d1);
		qr1c = qr1.conj();
		//Rotate p2, p3 and p5
		p2_1 = Quat::rotPoint(qr1, qr1c, this->p2);
		p3_1 = Quat::rotPoint(qr1, qr1c, this->p3);
		p5_1 = Quat::rotPoint(qr1, qr1c, this->p5);
		d2[i] = Quat::rotPoint(qr1, qr1c, this->d2);

		//Use subproblem 2pa
		//int sol = PadenKahan::subproblem2pa(d2_1[i], p5_1, Pwd, p3_1, p2_1, th2, th3);
		int sol = PadenKahan::subproblem2pa(d2[i], p5_1, Pwd, p3_1, p2_1, th2, th3);
		
		int index = 4*i;
		if (sol == 0)
		{
			//Eliminate all the solutions
			solFlags[index] = 0;
			solFlags[index + 1] = 0;
			solFlags[index + 2] = 0;
			solFlags[index + 3] = 0;
		}
		else if (sol == 1)
		{
			//Copy the solution
			solutions(index, 1) = th2[0];
			solutions(index, 2) = th3[0];
			solutions(index + 1, 1) = th2[0];			
			solutions(index + 1, 2) = th3[0];
			//Eliminate the other one
			solFlags[index + 2] = 0;
			solFlags[index + 3] = 0;
		}
		else if(sol == 2)
		{
			//Copy the two solutions to the solution matrix
			solutions(index, 1) = th2[0];
			solutions(index, 2) = th3[0];
			solutions(index + 1, 1) = th2[0];
			solutions(index + 1, 2) = th3[0];
			solutions(index + 2, 1) = th2[1];
			solutions(index + 2, 2) = th3[1];
			solutions(index + 3, 1) = th2[1];
			solutions(index + 3, 2) = th3[1];
		}
	}
	//---------------------------------------------------------------------------//

	//*******Step 3: Find theta 4 and theta 5 using the position of the end******//
	//*******efector from initial to desired and subproblem2*********************//
	// **************************************************************************//
	
	//Dual Quaternion for the final rotation operator
	DualQuat Q[8], Qc;
	//Dual quaternions for each rotation operator
	DualQuat q2;
	DualQuat q3;
	DualQuat Q123, Q123c;
	DualQuat dqL4_123, dqL7_123;
	Vector3d d4_123, d5_123, Pe_123, Pe_aux[8];
	double th4[2], th5[2];
	//Angle Limit for theta5
	double th5Lim = 125 * M_PI / 180;
	for (int i = 0; i < 8; i = i + 2) //Four iterations are necessary
	{
		if (solFlags[i] != 0)	//First check if the solution is valid
		{
			//Rotation operators of theta 2
			q2 = DualQuat::rotOperator(solutions(i, 1), this->d2, this->m2);
			//Rotation operators of theta 3
			q3 = DualQuat::rotOperator(solutions(i, 2), this->d3, this->m3);
			
			//Create the rotation operator for theta 1, 2 and 3
			Q123 = q1[i / 4] * q2*q3;
			Q123c = Q123.conj();

			//Get the current direction of z5 which is the same as d2 after rotation theta1 
			d5_123 = d2[i / 4];

			//Rotate L4 and L7 to find the current position of the end effector
			dqL4_123 = Q123*this->dqL4*Q123c;
			dqL7_123 = Q123*this->dqL7*Q123c;
			//The current position of the end effector is the intersection of dqL4_123 and dqL7_123
			Pe_123 = LinesIntersection(dqL4_123, dqL7_123);

			//Get z4 current direction
			d4_123 = dqL4_123.getPrim().getV();

			//Use subproblem 2
			int sol = PadenKahan::subproblem2(d4_123, d5_123, Pe_123, Ped, Pwd, th4, th5);

			//If the solution is valid copy it
			if (sol == 0)
			{
				//Eliminate the solutions
				solFlags[i] = 0;
				solFlags[i + 1] = 0;
			}
			else if (sol == 1)
			{
				solFlags[i + 1] = 0; //Eliminate the second solution
				if (abs(th5[0]) <= th5Lim) //Check for axis limit
				{
					//Copy the solution
					solutions(i, 3) = th4[0];
					solutions(i, 4) = th5[0];
					Q[i] = Q123;	//Store the valid transformation operator
					Pe_aux[i] = Pe_123;	//Store the valid position of the end effector
				}
				else
				{
					solFlags[i] = 0;
				}
			}
			else if (sol == 2)
			{
				if (abs(th5[0]) <= th5Lim) //Check for axis limit
				{
					//Copy the first solution to the solution matrix
					solutions(i, 3) = th4[0];
					solutions(i, 4) = th5[0];
					Q[i] = Q123;	//Store the valid transformation operator
					Pe_aux[i] = Pe_123;	//Store the valid position of the end effector
				}
				else
				{
					solFlags[i] = 0;
				}
				if (abs(th5[1]) <= th5Lim) //Check for axis limit
				{
					//Copy the second solution to the solution matrix
					solutions(i + 1, 3) = th4[1];
					solutions(i + 1, 4) = th5[1];
					Q[i + 1] = Q123;	//Store the valid transformation operator
					Pe_aux[i + 1] = Pe_123;	//Store the valid position of the end effector
				}
				else
				{
					solFlags[i] = 0;
				}
			}
		}
	}
	//---------------------------------------------------------------------------//

	//*******Step 4: Find theta 6 using subproblem 1 and two auxiliar points ****//
	//*******The starting point is the intersection of axis 7 and axis 8*********//
	// ******The final point is a point close to Ped in the direction of Nd******//

	//Dual quaternions for each rotation operator
	DualQuat q4;
	DualQuat q5;
	DualQuat dqL6_12345, dqL7_12345, dqL8_12345;
	Vector3d d6_12345, Paux1, Paux2;
	Paux2 = Ped + 5.0 * Nd;
	for (int i = 0; i < 8; i++)
	{
		if (solFlags[i] != 0)	//First check if the solution is valid
		{
			//Rotation operators of theta 4
			q4 = DualQuat::rotOperator(solutions(i, 3), this->d4, this->m4);
			//Rotation operators of theta 5
			q5 = DualQuat::rotOperator(solutions(i, 4), this->d5, this->m5);
			
			//Create the rotation operator for theta 1, 2, 3, 4 and 5
			Q[i] = Q[i]*q4*q5;
			Qc = Q[i].conj();

			//Rotate L6, L7 and L8
			dqL6_12345 = Q[i] * this->dqL6*Qc;
			dqL7_12345 = Q[i] * this->dqL7*Qc;
			dqL8_12345 = Q[i] * this->dqL8*Qc;
			//Get the current direction of z6
			d6_12345 = dqL6_12345.getPrim().getV();
			
			//Get the first auxiliar point
			Paux1 = LinesIntersection(dqL7_12345, dqL8_12345);

			//Use subproblem 1 and copy the solution to the solution matrix
			solutions(i, 5) = PadenKahan::subproblem1(d6_12345, Paux1, Paux2, Ped);
		}
	}
	//---------------------------------------------------------------------------//

	/*std::cout << "Inverse Kinematics Solutions: " << endl;
	std::cout << solutions * 180 / M_PI << endl;*/

	return solutions;
}

Vector3d Staubli::LinesIntersection(DualQuat dqL1, DualQuat dqL2)
{
	//P= (da x ma) + (db x mb)' * da * da
	return dqL1.getPrim().getV().cross(dqL1.getDual().getV())
		+ (dqL2.getPrim().getV().cross(dqL2.getDual().getV())).dot(dqL1.getPrim().getV())*dqL1.getPrim().getV();
}
