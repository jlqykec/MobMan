// StaubliKinematics.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include <math.h>
#define _USE_MATH_DEFINES // for the PI constant
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
	double interval;

	//Staubli Robot object
	Staubli robot;

	//Values of the generalized coordinates
	double q1, q2, q3, q4, q5, q6;
	q1 = 45.0 * M_PI / 180.0;
	q2 = 30.0 * M_PI / 180.0;
	q3 = 62.0 * M_PI / 180.0;
	q4 = -41.0 * M_PI / 180.0;
	q5 = -30.0 * M_PI / 180.0;
	q6 = 18.0 * M_PI / 180.0;

	//A vector to store the joint angles before sending them to the algorithm
	VectorXd q(6);
	q << q1, q2, q3, q4, q5, q6;

	//A Matrix to store the Transformation matrix of the forward kinematics
	Matrix4d T0e;

	//For measuring the execution time
	QueryPerformanceFrequency(&freq);
	QueryPerformanceCounter(&start);
	//Calculate the forward kinematics
	T0e = robot.forwardKin(q);
	QueryPerformanceCounter(&end);
	//Calculate the execution time in seconds
	interval = (double)(end.QuadPart - start.QuadPart) / freq.QuadPart;
	// Show the execution time
	cout << "Execution time: " << interval << "s" << endl;
	//Show the output of the forward kinematics
	cout << T0e << endl;
	system("pause");
	return 0;
}

