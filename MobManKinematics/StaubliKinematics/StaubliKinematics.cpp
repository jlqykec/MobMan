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
	double interval;

	//Staubli Robot object
	Staubli robot;

	//Values of the generalized coordinates
	double q1, q2, q3, q4, q5, q6;
	q1 = 90.0 * M_PI / 180.0;
	q2 = 45.0 * M_PI / 180.0;
	q3 = -45.0 * M_PI / 180.0;
	q4 = 0.0 * M_PI / 180.0;
	q5 = 0.0 * M_PI / 180.0;
	q6 = 0.0 * M_PI / 180.0;

	//A vector to store the joint angles before sending them to the algorithm
	VectorXd q(6);
	q << q1, q2, q3, q4, q5, q6;

	//A Matrix to store the Transformation matrix of the forward kinematics
	Matrix4d T0e;

	double acc = 0.0;
	int n = 5000;

	//For measuring the execution time
	QueryPerformanceFrequency(&freq);
	QueryPerformanceCounter(&start);
	for (int i = 0; i < n; i++)
	{
		//Calculate the forward kinematics
		T0e = robot.forwardKin(q);
	}
	QueryPerformanceCounter(&end);
	//Calculate the execution time in seconds
	interval = (double)(end.QuadPart - start.QuadPart) / freq.QuadPart * 1000;
	cout << "Execution time: " << interval / n << "ms" << endl;
	//Show the output of the forward kinematics
	cout << T0e << endl;

	//Calculate the inverse kinematics
	robot.inverseKin(T0e);

	system("pause");
	return 0;
}

