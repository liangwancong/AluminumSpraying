#include "pcl_filter_helper.h"

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

namespace PCLFilterHelper {
	using namespace std;

	PointCloudT::Ptr LoadPointCloud(char * cloud_file)
	{
		PointCloudT::Ptr cloud(new PointCloudT);

		string cloud_file_str = cloud_file;
		string suffixStr = cloud_file_str.substr(cloud_file_str.find_last_of('.') + 1);

		if (strcmp(suffixStr.c_str(), "pcd") == 0) {
			pcl::io::loadPCDFile(cloud_file, *cloud);
		}
		else if (strcmp(suffixStr.c_str(), "ply") == 0) {
			pcl::io::loadPLYFile(cloud_file, *cloud);
		}
		else {
			cout << "not a pcd or ply file" << endl;
		}

		return cloud;
	}

	PointCloudT::Ptr toPtr(PointCloudT cloud) {
		PointCloudT::Ptr cloudPtr(new PointCloudT);
		pcl::copyPointCloud(cloud, *cloudPtr);
		return cloudPtr;
	}

	void cropFilter(PointCloudT::Ptr &cloud_in, PointCloudT::Ptr &cloud_out, Eigen::Vector4f &min, Eigen::Vector4f &max, bool negative) {
		pcl::CropBox<pcl::PointXYZ> box_filter;						//滤波器对象
		box_filter.setMin(min);	//Min和Max是指立方体的两个对角点。每个点由一个四维向量表示，通常最后一个是1.
		box_filter.setMax(max);
		box_filter.setNegative(negative);
		box_filter.setInputCloud(cloud_in);
		box_filter.filter(*cloud_out);
	}

	void cropFilter(PointCloudT::Ptr &cloud_in, PointCloudT::Ptr &cloud_out, float minx, float maxx, float miny, float maxy, float minz, float maxz, bool negative) {
		Eigen::Vector4f min = Eigen::Vector4f(minx, miny, minz, 1.0);
		Eigen::Vector4f max = Eigen::Vector4f(maxx, maxy, maxz, 1.0);
		cropFilter(cloud_in, cloud_out, min, max, negative);
	}

	void voxelFilter(PointCloudT::Ptr &cloud_in, PointCloudT::Ptr &cloud_out, float lx, float ly, float lz) {
		std::cout << "size before voxelFilter: " << cloud_in->size() << std::endl;
		pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
		vox_grid.setLeafSize(lx, ly, lz);
		vox_grid.setInputCloud(cloud_in);
		vox_grid.filter(*cloud_out);
		std::cout << "size after voxelFilter: " << cloud_out->size() << std::endl;
	}

	void sorFilter(PointCloudT::Ptr &cloud_in, PointCloudT::Ptr &cloud_out, int meanK, float std_dev_mul,bool negative, bool keepOrganized) {
		std::cout << "size before statisticalOutlierRemoval: " << cloud_in->size() << std::endl;
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> filter(true);
		filter.setInputCloud(cloud_in);
		filter.setMeanK(meanK);
		filter.setStddevMulThresh(std_dev_mul);
		filter.setNegative(negative);
		filter.setKeepOrganized(keepOrganized);
		filter.filter(*cloud_out);
		std::cout << "size after statisticalOutlierRemoval: " << cloud_out->size() << std::endl;
	}

	void rorFilter(PointCloudT::Ptr &cloud_in, PointCloudT::Ptr &cloud_out, int radiusSearch, float minNeighborsInRadius, bool negative, bool keepOrganized) {
		std::cout << "size before radiusOutlierRemoval: " << cloud_in->size() << std::endl;

		pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
		ror.setInputCloud(cloud_in);
		ror.setRadiusSearch(radiusSearch);
		ror.setMinNeighborsInRadius(minNeighborsInRadius);
		ror.setNegative(negative);
		ror.setKeepOrganized(keepOrganized);
		ror.filter(*cloud_out);

		std::cout << "size after radiusOutlierRemoval: " << cloud_out->size() << std::endl;

	}

	void passThroughFilter(PointCloudT::Ptr &cloud_in, PointCloudT::Ptr &cloud_out, const std::string &field_name, const float &limit_min, const float &limit_max, bool negative) {
		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud(cloud_in);            //设置输入点云
		pass.setFilterFieldName(field_name);         //设置过滤时所需要点云类型的Z字段
		pass.setFilterLimits(limit_min, limit_max);        //设置在过滤字段的范围
		pass.setFilterLimitsNegative (negative);   //设置保留范围内还是过滤掉范围内
		pass.filter(*cloud_out);            //执行滤波，保存过滤结果在cloud_filtered

	}

	void projectInliersFilter(PointCloudT::Ptr &cloud_in, PointCloudT::Ptr &cloud_out, pcl::ModelCoefficients::Ptr &coefficients, int modelType) {

		pcl::ProjectInliers<pcl::PointXYZ> proj;		//创建投影滤波对象
		proj.setModelType(modelType);			//设置对象对应的投影模型
		proj.setInputCloud(cloud_in);			//设置输入点云
		proj.setModelCoefficients(coefficients);		//设置模型对应的系数
		proj.filter(*cloud_out);						//执行投影滤波存储结果cloud_projected

	}

	void extractIndicesFilter(PointCloudT::Ptr &cloud_in, PointCloudT::Ptr &cloud_out, pcl::PointIndices::Ptr &indices, bool negative) {

		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(cloud_in);
		extract.setIndices(indices);
		extract.setNegative(negative);
		extract.filter(*cloud_out);
	}

	void euclideanClusterExtraction(PointCloudT::Ptr &cloud_in, std::vector<pcl::PointIndices> &cluster_indices_out, double tolerance, int min_cluster_size, int max_cluster_size) {
		// 创建用于提取搜索方法的kdtree树对象
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud(cloud_in);

		pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;		//欧式聚类对象
		ec.setClusterTolerance(tolerance);						// 设置近邻搜索的搜索半径为50mm
		ec.setMinClusterSize(min_cluster_size);						//设置一个聚类需要的最少的点数目为100
		ec.setMaxClusterSize(max_cluster_size);					//设置一个聚类需要的最大点数目为200000
		ec.setSearchMethod(tree);						//设置点云的搜索机制
		ec.setInputCloud(cloud_in);
		ec.extract(cluster_indices_out);					//从点云中提取聚类，并将点云索引保存在cluster_indices中
	}
}
 

