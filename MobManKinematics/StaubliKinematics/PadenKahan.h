#ifndef PADENKAHAN_H
#define PADENKAHAN_H

#include <Eigen/Dense>

using namespace Eigen;

class PadenKahan
{
public:
	static int subproblem2pa(Vector3d d1, Vector3d p, Vector3d q, Vector3d r1, Vector3d r2x, double *th1, double *th2);
};


#endif
