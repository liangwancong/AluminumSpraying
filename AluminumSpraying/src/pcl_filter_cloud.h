#pragma once
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/common/distances.h>
#include <pcl/common/pca.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include "pcl_filter_cloud.h"
#include "CRMath.h"
#include "parameters.h"
#include "global.h"
#include "logger.h"
#include "pcl_filter_helper.h"
#include "file_util.h"
#include "pclgeomerty.h"
#include "cvskeleton.h"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointT> PointCloud;
namespace pcl_filter_cloud {
	using namespace std;
	using namespace cv;
	using namespace pcl_filter_cloud;
	using namespace CRMath;
	using namespace AluminumParameters;
	using namespace AluminumGlobal;
	using namespace logger;
	using namespace PCLFilterHelper;
	using namespace fileutil;
	using namespace cvgeomerty;

	const int half_project_length = 50;
	const int PICWIDTH = 500;
	const int PICWIDTH_2 = 250;
	const int cloud_head = 0;
	const int cloud_mid = 1;
	const int cloud_tail = 2;

	string savePointCloudCrossPic(PointCloudT::Ptr &cloud_in, float min_x, float max_x, int sequenceNo, double half_project_length, int cloud_pos, float head_offset, float mid_offset, float tail_offset, string out_name = "");
	string savePointCloudCrossPic(PointCloudT::Ptr &cloud_in, float min_x, float max_x, Vec4f &plane_line, Eigen::Affine3f &pca_partial_transform_matrix, int sequenceNo, double half_project_length, int cloud_pos, float head_offset, float mid_offset, float tail_offset, string out_name = "");
	int filterRealPointCloud(char * cloud_front_file, char * cloud_file, std::vector<PointCloudT::Ptr> &originGroupClouds, PointCloudT::Ptr &cloud_real, float &min_real_x, float &max_real_x, double cloud_length, double stepDistanceThresh, double poleWidthThresh, int poleSizeThresh, double poleOffesetThresh);
	//int getPointCloudCrossPaintCorners(const std::string &file_path, Vec4f &plane_line, cvgeomerty::LINE &LineSEG, Eigen::Affine3f &pca_partial_transform_matrix, double minAngleThresh, double minIntervalThresh, double minDistanceThresh);
}