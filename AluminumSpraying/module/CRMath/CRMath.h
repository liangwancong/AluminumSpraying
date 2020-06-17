#pragma once
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <math.h>
#include <iostream>

namespace CRMath {
	#define M_PI 3.14159265358979323846 
	using namespace Eigen;
	typedef Matrix<double, 1, 6> Vector6d;

	Vector6d  Matrix4ToEuler(Matrix<double, 4, 4> matrix4);
	int getRotation_x(double* xyzrxryrz, double angle_x);
	void getSplingRxRyRz(double* rxryrx, double* rxryrx_spling, double* rxryrz_out);
	int getSplingRxRyRz(double* xyzrxryrz1, double*xyzrxryrz2, double angle_spling, double &Rx, double &Ry, double &Rz);
}
