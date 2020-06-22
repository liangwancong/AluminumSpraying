#include "CRMath.h"

namespace CRMath {
	//using namespace Eigen;

	//typedef Matrix<double, 1, 6> Vector6d;
	//typedef Matrix<double, 4, 4> Matrix4d;

	const double esp = 0.00001;
	double d2o(double data)
	{
		if (abs(data) < esp) return 0;
		else return data;
	}
	double d2One(double data)
	{
		if (abs(data) > 1 && abs(abs(data) - 1) < esp) return (abs(data) / data);
		else return data;
	}
	Matrix<double, 3, 3> EulerRToMatrix3(double rx, double ry, double rz)
	{
		//MatrixTransform3D;
		const double pi = M_PI;

		double Rx = rx * (pi / 180);   //γ
		double Ry = ry * (pi / 180);   //β
		double Rz = rz * (pi / 180);   //α

		double cz = cos(Rz); double sz = sin(Rz);
		double cy = cos(Ry); double sy = sin(Ry);
		double cx = cos(Rx); double sx = sin(Rx);

		Matrix<double, 3, 3>  M;
		M<<
		 cz*cy, cz*sy*sx - sz * cx, cz*sy*cx + sz * sx,
		 sz*cy, sz*sy*sx + cz * cx, sz*sy*cx - cz * sx,
		 -sy,   cy*sx,          cy*cx;

		return M;
	}
	Matrix<double, 4, 4> EulerToMatrix4(Vector6d eulerAngle)
	{
		//MatrixTransform3D;
		const double pi = M_PI;

		double Rx = eulerAngle(0, 3) * (pi / 180);   //γ
		double Ry = eulerAngle(0, 4) * (pi / 180);   //β
		double Rz = eulerAngle(0, 5) * (pi / 180);   //α

		double cz = cos(Rz); double sz = sin(Rz);
		double cy = cos(Ry); double sy = sin(Ry);
		double cx = cos(Rx); double sx = sin(Rx);

		Matrix<double, 4, 4> M;
		M <<
			cz * cy, cz*sy*sx - sz * cx, cz*sy*cx + sz * sx, eulerAngle(0, 0),
			sz*cy, sz*sy*sx + cz * cx, sz*sy*cx - cz * sx, eulerAngle(0, 1),
			-sy, cy*sx, cy*cx, eulerAngle(0, 2),
			0, 0, 0, 1;

		return M;
	}
	void Matrix3ToEulerR(Matrix<double, 3,3> M,  double &Rx,  double &Ry,  double &Rz)
	{
		double rx, rz;
		//判断ry的值是否为+-90°
		double ry = atan2(-d2o(M(2, 0)), sqrt(pow(d2o(M(0, 0)), 2) + pow(d2o(M(1, 0)), 2)));//beta

		if (abs(cos(ry)) < esp)
		{
			rx = M_PI / 2;//gamma
			if (abs(sin(ry)) > esp)
			{
				rz = (atan2(d2o(M(1, 2)), d2o(M(0, 2))) + rx);
			}
			else
			{
				rz = -(atan2(d2o(M(1, 2)), d2o(M(0, 2))) + rx);//alpha
			}
		}
		else
		{
			rx = atan2(d2o(M(2, 1)), d2o(M(2, 2)));
			rz = atan2(d2o(M(1, 0)), d2o(M(0, 0))); //atan2(ny,nx)
		}
		Rx = rx / M_PI * 180;
		Ry = ry / M_PI * 180;
		Rz = rz / M_PI * 180;
		//std::cout << "RX" << Rx << std::endl;
		//std::cout << "RY" << Ry << std::endl;
		//std::cout<<"RZ" << Rz << std::endl;
	}
	Vector6d  Matrix4ToEuler(Matrix<double, 4, 4> matrix4)
	{
		Vector6d eulerAngle;
		Matrix<double, 3, 3> M; 
		M<<
		matrix4(0,0), matrix4(0,1), matrix4(0,2),
		matrix4(1,0), matrix4(1,1), matrix4(1,2),
		matrix4(2,0), matrix4(2,1), matrix4(2,2);

		Matrix3ToEulerR(M, eulerAngle(0,3), eulerAngle(0, 4), eulerAngle(0, 5));

		eulerAngle(0, 0) = matrix4(0, 3); 
		eulerAngle(0, 1) = matrix4(1, 3); 
		eulerAngle(0, 2) = matrix4(2, 3); 
		return eulerAngle;
	}

	void getSplingRxRyRz(double* rxryrz, double* rxryrz_spling, double* rxryrz_out) {
		Vector6d euler_origin;
		euler_origin << 0, 0, 0, rxryrz[0], rxryrz[1], rxryrz[2];
		Matrix4d tm_origin = EulerToMatrix4(euler_origin);
		std::cout << "tm_origin:" << tm_origin << std::endl;
		Vector6d euler_spling;
		euler_spling << 0, 0, 0, rxryrz_spling[0], rxryrz_spling[1], rxryrz_spling[2];
		Matrix4d tm_sping = EulerToMatrix4(euler_spling);
		std::cout << "tm_sping:" << tm_sping << std::endl;

		Matrix4d tm = tm_origin*tm_sping;
		Vector6d euler = Matrix4ToEuler(tm);
		std::cout<<"euler:" << euler << std::endl;
		rxryrz_out[0] = euler(0, 3);
		rxryrz_out[1] = euler(0, 4);
		rxryrz_out[2] = euler(0, 5);
	}

	int getSplingRxRyRz(double* xyzrxryrz1, double*xyzrxryrz2, double angle_spling, double &Rx, double &Ry, double &Rz) {
		double distance = pow(xyzrxryrz1[0] - xyzrxryrz2[0], 2) + pow(xyzrxryrz1[1] - xyzrxryrz2[1], 2) + pow(xyzrxryrz1[2] - xyzrxryrz2[2], 2);
		distance = sqrt(distance);
		if (distance < esp) {
			return -1;
		}
		//第二个点的rxryrz
		double rx = xyzrxryrz2[3];
		double ry = xyzrxryrz2[4];
		double rz = xyzrxryrz2[5];
		//计算姿态矩阵
		Matrix<double, 3, 3> rotation_origin = EulerRToMatrix3( rx,  ry,  rz);
		//枪姿态方向
		Vector3d vec_z(0,0,1);
		Vector3d vec_rotation_z = rotation_origin * vec_z;
		//std::cout <<"vec_rotation_z:"<< vec_rotation_z << std::endl;
		//工件方向
		Vector3d vec_x(xyzrxryrz2[0] - xyzrxryrz1[0], xyzrxryrz2[1] - xyzrxryrz1[1], xyzrxryrz2[2] - xyzrxryrz1[2]);
		//std::cout << "vec_x:" << vec_x << std::endl;
		//工件方向和枪姿态方向的平面,对应的垂线方向
		Vector3d vec_y = vec_rotation_z.cross(vec_x).normalized();
		//std::cout << "vec_y:" << vec_y << std::endl;
		//计算甩枪矩阵
		angle_spling = angle_spling / 180.0*M_PI;
		AngleAxisd v(angle_spling, vec_y);
		Matrix3d roatation_spling =Matrix3d::Identity();
		roatation_spling = v.toRotationMatrix();
		//计算最终姿态方向
		//Matrix3d rotation = rotation_origin * roatation_spling;
		Matrix3d rotation = roatation_spling * rotation_origin;
		//旋转矩阵转欧拉
		Matrix3ToEulerR(rotation, Rx, Ry, Rz);
		return 0;
	}
	

	int getRotation_x(double* xyzrxryrz, double angle_x) {
		double rx = xyzrxryrz[3];
		double ry = xyzrxryrz[4];
		double rz = xyzrxryrz[5];
		Matrix<double, 3, 3> rotation_origin = EulerRToMatrix3(rx, ry, rz);
		angle_x = angle_x / 180.0*M_PI;
		AngleAxisd v(angle_x, Vector3d(1,0,0));
		Matrix3d roatation_x = Matrix3d::Identity();
		roatation_x = v.toRotationMatrix();
		double Rx = 0;
		double Ry = 0;
		double Rz = 0;
		roatation_x = rotation_origin*roatation_x;
		Matrix3ToEulerR(roatation_x, Rx, Ry, Rz);
		std::cout << "Rx:" << Rx << std::endl;
		std::cout << "Ry:" << Ry << std::endl;
		std::cout <<"Rz:"<< Rz << std::endl;


		Vector3d vec_z(0, 0, 1);
		Matrix<double, 3, 3> rotation_A = EulerRToMatrix3(rx, ry, rz);
		Matrix<double, 3, 3> rotation_B = EulerRToMatrix3(angle_x, ry, rz);
		Matrix<double, 3, 3> rotation_AB = rotation_B * rotation_A.inverse();
		Vector3d vec_r = rotation_AB*vec_z;
		std::cout << vec_r << std::endl;
		system("pause");
		return 0;
	}

	double  getV1V2Angle(const Eigen::Vector3f &V1, const Eigen::Vector3f &V2) {
		double tem = V1.dot(V2);
		double tep = sqrt(V1.dot(V1) * V2.dot(V2));
		double angle = acos(tem / tep);
		if (isnan(angle))
		{
			std::cout << "angle error" << std::endl;
			return DBL_MAX;
		}
		return angle;
	}
	int getV12V2Tm(const Eigen::Vector3f &v1, const Eigen::Vector3f &v2, Eigen::Matrix4f &tm) {
		//v1面-->v2面
		//两个平面法向量的夹角
		double angle = getV1V2Angle(v1, v2);
		if (angle == DBL_MAX) {
			return -1;
		}
		//求旋转轴
		Eigen::Vector3f axis = v1.cross(v2);

		//求旋转矩阵
		Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
		Eigen::AngleAxisf v((float)angle, axis.normalized());
		//Eigen::AngleAxisf v((float)angle, Eigen::Vector3f(0, 1, 0));
		R = v.toRotationMatrix();
		tm.block<3, 3>(0, 0) = R;
		tm.block<3, 1>(0, 3) << 0, 0, 0;
		return 0;
	}
}