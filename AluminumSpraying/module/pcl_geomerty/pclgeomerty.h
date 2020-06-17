#pragma once
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/common/time.h>
#include <pcl/common/angles.h>
#include <pcl/common/distances.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <opencv2/core/eigen.hpp>

namespace pclgeomerty {
	using namespace pcl;

	struct PLANE
	{
		double a;
		double b;
		double c;
		double d;

		PointXYZ normal;

		PLANE() {}
		PLANE(PointXYZ p1, PointXYZ p2, PointXYZ n) {
		}
		PLANE(double _a, double _b, double _c, double _d) {
			a = _a;
			b = _b;
			c = _c;
			d = _d;

		}

	};

	struct LINE           
	{
		PointXYZ start;
		PointXYZ end;

		double a1;
		double b1;
		double c1;
		double d1;

		double a2;
		double b2;
		double c2;
		double d2;

		LINE() {}
		LINE(PointXYZ _start, PointXYZ _end) {

			start = _start;
			end = _end;
		}
	};

	/******************************************************************************************
	计算两点之间的距离
	*******************************************************************************************/
	double distance(PointXYZ p1, PointXYZ p2);

	/******************************************************************************************
	返回线段l1与l2之间的夹角 单位：弧度 范围(-pi，pi)
	*******************************************************************************************/
	double lsangle(LINE l1, LINE l2);

	/******************************************************************************************
	获取两个点之间的中点
	*******************************************************************************************/
	PointXYZ getMidPoint(PointXYZ p1, PointXYZ p2);

	Eigen::Matrix3f Calculation(Eigen::Vector3f vectorBefore, Eigen::Vector3f vectorAfter);

	/******************************************************************************************
	根据点云和预估质心点获取PCA仿射矩阵
	*******************************************************************************************/
	void getPCATransfrom(const pcl::PointCloud<pcl::PointXYZ> &cloud_in, Eigen::Vector4f &centroid, Eigen::Matrix4f &tm);

}