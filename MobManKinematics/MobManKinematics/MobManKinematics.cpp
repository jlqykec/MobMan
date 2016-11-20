// TestForwardKinematics.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include <math.h>
#define _USE_MATH_DEFINES // for the PI constant
#include <Eigen/Dense>
#include <iostream>
#include "MMUR5.h"

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

	//Mobile manipulator object
	MMUR5 robot;

	//Values of the generalized coordinates
	double tx, ty, tz, phi, q1, q2, q3, q4, q5, q6;
	tx = 0.0;
	ty = 0.0;
	tz = 0.0;
	phi = 0.0 * M_PI / 180.0;
	q1 = 0.0 * M_PI / 180.0;
	q2 = 0.0 * M_PI / 180.0;
	q3 = 0.0 * M_PI / 180.0;
	q4 = 0.0 * M_PI / 180.0;
	q5 = 0.0 * M_PI / 180.0;
	q6 = 0.0 * M_PI / 180.0;

	//A vector to store the values before sending them to the algorithm
	VectorXd q(10);
	q << tx, ty, phi, tz, q1, q2, q3, q4, q5, q6;

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
	cout << "Execution time: " << interval << endl;
	//Show the output of the forward kinematics
	cout << T0e << endl;	
	system("pause");
	return 0;
}



