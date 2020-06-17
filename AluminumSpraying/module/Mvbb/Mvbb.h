#pragma once
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include "ApproxMVBB/ComputeApproxMVBB.hpp"

namespace G3CppMvbb {
	typedef pcl::PointXYZ PointT;
	typedef pcl::PointCloud<PointT> PointCloud;

	int getCornerPointCloud(PointCloud::Ptr &cloud, PointCloud::Ptr &cornerPointsCloud);
	void sortCornerPoint(PointCloud::Ptr &cornerPointsCloud);
	int parseCornerCloud(PointCloud::Ptr &cloud, PointCloud::Ptr &cornerCloud);
	int parseCornerCloud(PointCloud::Ptr &cloud);
	int makeCornerCloud(PointCloud::Ptr &cloud_in, PointCloud::Ptr &cloud_out);
}


