// TestForwardKinematics.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include <math.h>
#include <Eigen/Dense>
#include <iostream>
#include "DualQuat.h"
#include "MMUR5.h"
using namespace std;
using namespace Eigen;

int main()
{
	/*
	Vector3d v(1,2,3);
	//v << 1, 2, 3;
	cout << 0.1*v << endl;

	//Quaternion
	Quat q1(1,2,3,4);
	Quat q2(3.1,4.2,5.3,6.4);

	//cout << q1.conj() << endl;

	cout << "Quaternion: " << endl;
	//cout << Quat::rotQuat(0.1,v) << endl;
	cout << 10*q1 << endl;

	//Dual Quaternion
	DualQuat dq1(-0.1,-0.1, -0.1,-0.1,-0.2,-0.2,-0.2,-0.2);
	cout << "Dual Quaternion: " << endl;
	cout << dq1 << endl;
	cout << "Primary: " << dq1.getPrim() << "   ";
	cout << "Dual: " << dq1.getDual() << endl << endl;

	DualQuat dq2(q1, q2);
	cout << dq2 << endl;
	cout << dq2.conj() << endl << endl;

	DualQuat dq3(q2, q1);
	cout << dq3 << endl;
	cout << dq3.conj()*dq2 << endl << endl;

	cout<< DualQuat::linesIntPlucker(dq2, dq3) << endl;*/


	MMUR5 robot;
	VectorXd q(10);
	q << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

	cout << robot.forwardKin(q) << endl;


	system("pause");
	return 0;
}



