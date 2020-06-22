#pragma once
#include <pcl/io/pcd_io.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/ModelCoefficients.h>

namespace PCLFilterHelper {
	using namespace std;

	typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;

	PointCloudT::Ptr LoadPointCloud(char * cloud_file);

	PointCloudT::Ptr toPtr(PointCloudT cloud);

	void cropFilter(PointCloudT::Ptr &cloud_in, PointCloudT::Ptr &cloud_out, Eigen::Vector4f &min, Eigen::Vector4f &max, bool negative = false);

	void cropFilter(PointCloudT::Ptr &cloud_in, PointCloudT::Ptr &cloud_out, float minx, float maxx, float miny, float maxy, float minz, float maxz, bool negative = false);

	//体素化滤波
	void voxelFilter(PointCloudT::Ptr &cloud_in, PointCloudT::Ptr &cloud_out, float lx, float ly, float lz);
	//统计滤波
	void sorFilter(PointCloudT::Ptr &cloud_in, PointCloudT::Ptr &cloud_out, int meanK, float std_dev_mul, bool negative = false, bool keepOrganized = false);
	//半径滤波
	void rorFilter(PointCloudT::Ptr &cloud_in, PointCloudT::Ptr &cloud_out, int radiusSearch, float minNeighborsInRadius, bool negative = false, bool keepOrganized = false);
	//直通滤波
	void passThroughFilter(PointCloudT::Ptr &cloud_in, PointCloudT::Ptr &cloud_out, const std::string &field_name, const float &limit_min, const float &limit_max, bool negative = false);
	//投影滤波
	void projectInliersFilter(PointCloudT::Ptr &cloud_in, PointCloudT::Ptr &cloud_out, pcl::ModelCoefficients::Ptr &coefficients, int modelType);
	//索引滤波
	void extractIndicesFilter(PointCloudT::Ptr &cloud_in,  PointCloudT::Ptr &cloud_out, pcl::PointIndices::Ptr &indices, bool negative = false);

	//欧式聚类
	void euclideanClusterExtraction(PointCloudT::Ptr &cloud_in, std::vector<pcl::PointIndices> &cluster_indices_out, double tolerance, int min_cluster_size, int max_cluster_size);

	void extractIndices(PointCloudT::Ptr &cloud, const pcl::PointIndices::Ptr &indices, const bool &negative);

	void sacSegmentation(PointCloudT::Ptr &cloud, pcl::ModelCoefficients::Ptr coefficients, const int &model, const int &max_iterations, const double &threshold);

}
