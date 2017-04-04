#include "PadenKahan.h"
#define _USE_MATH_DEFINES // for the PI constant
#include <math.h>

using namespace std;

int PadenKahan::subproblem2pa(Vector3d d1, Vector3d p, Vector3d q, Vector3d r1, Vector3d r2x, double *th1, double *th2)
{
	Vector3d rq = (r1 - r2x).dot(d1)*d1;
	return 0;
}
