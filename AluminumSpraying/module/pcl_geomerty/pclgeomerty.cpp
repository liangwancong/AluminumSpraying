#include "pclgeomerty.h"

namespace pclgeomerty {
	/******************************************************************************************
	计算两点之间的距离
	*******************************************************************************************/

	double distance(PointXYZ p1, PointXYZ p2) {
		return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
	}

	/******************************************************************************

	r=multiply(sp,ep,op),得到(sp-op)和(ep-op)的叉积

	r>0：ep在矢量opsp的逆时针方向；

	r=0：opspep三点共线；

	r<0：ep在矢量opsp的顺时针方向

	*******************************************************************************/

	PointXYZ multiply(PointXYZ sp, PointXYZ ep, PointXYZ op)

	{
		float x, y, z;

		x = (sp.y - op.y)*(ep.z - op.z) - (ep.y - op.y)*(sp.z - op.z);
		y = (sp.z - op.z)*(ep.x - op.x) - (ep.z - op.z)*(sp.x - op.x);
		z = (sp.x - op.x)*(ep.y - op.y) - (ep.x - op.x)*(sp.y - op.y);

		return   PointXYZ( x, y ,z);

	}

	/******************************************************************************

	r=dotmultiply(p1,p2,op),得到矢量(p1-op)和(p2-op)的点积，如果两个矢量都非零矢量

	r<0：两矢量夹角为锐角；

	r=0：两矢量夹角为直角；

	r>0：两矢量夹角为钝角

	*******************************************************************************/

	double dotmultiply(PointXYZ p1, PointXYZ p2, PointXYZ p0)

	{
		return ((p1.x - p0.x)*(p2.x - p0.x) + (p1.y - p0.y)*(p2.y - p0.y) + (p1.z - p0.z)*(p2.y - p0.z));

	}

	double angle(PointXYZ o, PointXYZ s, PointXYZ e)

	{

		double cosfi, fi, norm;

		double dsx = s.x - o.x;

		double dsy = s.y - o.y;

		double dsz = s.z - o.z;

		double dex = e.x - o.x;

		double dey = e.y - o.y;

		double dez = e.z - o.z;


		cosfi = dsx * dex + dsy * dey + dsz * dez;

		norm = (dsx * dsx + dsy * dsy + dsz * dsz) * (dex * dex + dey * dey + dez * dez);

		cosfi /= sqrt(norm);



		if (cosfi >= 1.0) return 0;

		if (cosfi <= -1.0) return -M_PI;



		fi = acos(cosfi);

		return fi; 

	}

	// 返回线段l1与l2之间的夹角 单位：弧度 范围(-pi，pi) 

	double lsangle(LINE l1, LINE l2)

	{

		PointXYZ o, s, e;

		o.x = o.y = o.z = 0;

		s.x = l1.end.x - l1.start.x;

		s.y = l1.end.y - l1.start.y;

		s.z = l1.end.z - l1.start.z;

		e.x = l2.end.x - l2.start.x;

		e.y = l2.end.y - l2.start.y;

		e.z = l2.end.z - l2.start.z;

		return angle(o, s, e);

	}


	/*
	获取两个点之间的中点
	*/
	PointXYZ getMidPoint(PointXYZ p1, PointXYZ p2) {
		return PointXYZ((p1.x + p2.x) / 2., (p1.y + p2.y) / 2., (p1.z + p2.z) / 2.);
	}

	Eigen::Matrix3f Calculation(Eigen::Vector3f vectorBefore, Eigen::Vector3f vectorAfter)
	{
		vectorBefore.normalize();
		vectorAfter.normalize();

		Eigen::Vector3f rotationAxis = vectorBefore.cross(vectorAfter);
		std::cout << "rotation axis:\n" << rotationAxis << std::endl;
		double rotationAngle = acos(vectorBefore.dot(vectorAfter));
		std::cout << "rotation angle: " << rotationAngle * 180 / M_PI << std::endl;
		
		rotationAxis.normalize();
		Eigen::AngleAxisf rotation_vector(rotationAngle, rotationAxis);
		return rotation_vector.matrix();
	}

	/*double[] CrossProduct(double[] a, double[] b)
	{
		double[] c = new double[3];

		c[0] = a[1] * b[2] - a[2] * b[1];
		c[1] = a[2] * b[0] - a[0] * b[2];
		c[2] = a[0] * b[1] - a[1] * b[0];

		return c;
	}*/

	//double DotProduct(double[] a, double[] b)
	//{
	//	double result;
	//	result = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];

	//	return result;
	//}

	//double Normalize(double[] v)
	//{
	//	double result;

	//	result = Math.Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);

	//	return result;
	//}

	void getPCATransfrom(const pcl::PointCloud<pcl::PointXYZ> &cloud_in, Eigen::Vector4f &centroid, Eigen::Matrix4f &tm) {

		Eigen::Vector4f pcaCentroid = centroid;//质心点(4x1)

		Eigen::Matrix3f covariance;
		pcl::computeCovarianceMatrixNormalized(cloud_in, pcaCentroid, covariance);
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
		Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();//特征向量ve(3x3)
		Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();//特征值va(3x1)
		eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1)); //校正主方向间垂直
		eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
		eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));

		tm.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();   //R.
		tm.block<3, 1>(0, 3) = -1.0f * (eigenVectorsPCA.transpose()) *(pcaCentroid.head<3>());//  -R*t

	}
}