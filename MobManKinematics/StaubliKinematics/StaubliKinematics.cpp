// StaubliKinematics.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#define _USE_MATH_DEFINES // for the PI constant
#include <math.h>
#include <Eigen/Dense>
#include <iostream>
#include "Staubli.h"

#include <stdio.h>
#include <stdlib.h>
#include <Windows.h>

using namespace std;
using namespace Eigen;

int main()
{
	//For measuring the execution time
	LARGE_INTEGER freq;
	LARGE_INTEGER start;
	LARGE_INTEGER end;

	//Staubli Robot object
	Staubli robot;

	//Values of the generalized coordinates
	double q1, q2, q3, q4, q5, q6;
	q1 = -90.0 * M_PI / 180.0;
	q2 = 30.0 * M_PI / 180.0;
	q3 = -60.0 * M_PI / 180.0;
	q4 = 45.0 * M_PI / 180.0;
	q5 = 45.0 * M_PI / 180.0;
	q6 = 45.0 * M_PI / 180.0;

	/*q1 = 90.0 * M_PI / 180.0;
	q2 = 45.0 * M_PI / 180.0;
	q3 = -45.0 * M_PI / 180.0;
	q4 = 0.0 * M_PI / 180.0;
	q5 = 0.0 * M_PI / 180.0;
	q6 = 0.0 * M_PI / 180.0;*/

	cout << "Desired joint angles: " << endl
		<< "q1: " << q1 * 180 / M_PI << endl
		<< "q2: " << q2 * 180 / M_PI << endl
		<< "q3: " << q3 * 180 / M_PI << endl
		<< "q4: " << q4 * 180 / M_PI << endl
		<< "q5: " << q5 * 180 / M_PI << endl
		<< "q6: " << q6 * 180 / M_PI << endl;

	//A vector to store the joint angles before sending them to the algorithm
	VectorXd q(6);
	q << q1, q2, q3, q4, q5, q6;

	//Time calculation related variables
	int n = 1000;	//Number of iterations to calculate forward and inverse kinematics
	QueryPerformanceFrequency(&freq);	//For measuring the execution time
	VectorXd times = Eigen::VectorXd::Zero(n);	//To store the times in each step
	

	//-------------------FORWARD KINEMATICS---------------------//
	Matrix4d T0e; //A Matrix to store the Transformation matrix of the forward kinematics
	for (int i = 0; i < n; i++)
	{
		QueryPerformanceCounter(&start);
		T0e = robot.forwardKin(q);		//Calculate the forward kinematics
		QueryPerformanceCounter(&end);
		times(i) = (double)(end.QuadPart - start.QuadPart) / freq.QuadPart * 1000000;
	}
	cout << "Mean time: " << times.mean() << "us" << endl;
	cout << "Min time: " << times.minCoeff() << "us" << endl;
	cout << "Max time: " << times.maxCoeff() << "us" << endl;

	//Show the output of the forward kinematics
	cout << T0e << endl;
	//----------------------------------------------------------//


	//-------------------INVERSE KINEMATICS---------------------//
	MatrixXd solutions;
	int solFlags[8];
	for (int i = 0; i < n; i++)
	{
		QueryPerformanceCounter(&start);
		solutions = robot.inverseKin(T0e, solFlags);	//Calculate the inverse kinematics
		QueryPerformanceCounter(&end);
		times(i) = (double)(end.QuadPart - start.QuadPart) / freq.QuadPart * 1000000;
	}
	cout << "Mean time: " << times.mean() << "us" << endl;
	cout << "Min time: " << times.minCoeff() << "us" << endl;
	cout << "Max time: " << times.maxCoeff() << "us" << endl;

	//Show the output of the inverse kinematics
	std::cout << "Inverse Kinematics Solutions: " << endl;
	std::cout << solutions * 180 / M_PI << endl;
	//----------------------------------------------------------//

	system("pause");
	return 0;
}

