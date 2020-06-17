#include "pcl_filter_cloud.h"
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include "parameters.h"
#include "global.h"
#include "logger.h"
#include "Registration.h"

namespace pcl_filter_cloud {
	using namespace PCLFilterHelper;
	using namespace std;
	using namespace cv;
	using namespace fileutil;
	using namespace AluminumParameters;
	using namespace AluminumGlobal;
	using namespace logger;
	int cluster_index = 0;
	const double Tolerance = 50;
	const int MinClusterSize = 100;

	int getMaxAbsValueIndex(Eigen::Vector3f vector) {

		float v1 = abs(vector.x());
		float v2 = abs(vector.y());
		float v3 = abs(vector.z());

		if (v1 > v2) {
			if (v1 > v3) {
				return vector.x() > 0 ? 1 : -1;
			}
			else {
				return vector.z() > 0 ? 3 : -3;
			}
		}
		else if (v2 > v3) {
			return vector.y() > 0 ? 2 : -2;
		}
		else {
			return vector.z() > 0 ? 3 : -3;
		}
	}

	/*标准化包围盒找到的旋转矩阵*/
	Eigen::Affine3f standardization(Eigen::Affine3f tm_aff) {
		//原始坐标系
		Eigen::Vector3f axis_o_origin = Eigen::Vector3f(0, 0, 0);
		Eigen::Vector3f axis_x_origin = Eigen::Vector3f(1, 0, 0);
		Eigen::Vector3f axis_y_origin = Eigen::Vector3f(0, 1, 0);
		Eigen::Vector3f axis_z_origin = Eigen::Vector3f(0, 0, 1);

		//PCA转换后的坐标系
		Eigen::Vector3f axis_o_trans = tm_aff * axis_o_origin;
		Eigen::Vector3f axis_x_trans = tm_aff * axis_x_origin;
		Eigen::Vector3f axis_y_trans = tm_aff * axis_y_origin;
		Eigen::Vector3f axis_z_trans = tm_aff * axis_z_origin;
		Eigen::Vector3f axis_x_vector = axis_x_trans - axis_o_trans;
		Eigen::Vector3f axis_y_vector = axis_y_trans - axis_o_trans;
		Eigen::Vector3f axis_z_vector = axis_z_trans - axis_o_trans;
		cout << "PCA transform:" << endl;
		cout << axis_x_vector.x() << " " << axis_x_vector.y() << " " << axis_x_vector.z() << endl;
		cout << axis_y_vector.x() << " " << axis_y_vector.y() << " " << axis_y_vector.z() << endl;
		cout << axis_z_vector.x() << " " << axis_z_vector.y() << " " << axis_z_vector.z() << endl;

		//X坐标校正
		int max_index_x = getMaxAbsValueIndex(axis_x_vector);
		switch (max_index_x)
		{
		case 1:
			break;
		case -1:
			tm_aff = pcl::getTransformation(0, 0, 0, 0, M_PI, 0) * tm_aff;
			break;
		case 2:
			tm_aff = pcl::getTransformation(0, 0, 0, 0, 0, -M_PI_2) * tm_aff;
			break;
		case -2:
			tm_aff = pcl::getTransformation(0, 0, 0, 0, 0, M_PI_2) * tm_aff;
			break;
		case 3:
			tm_aff = pcl::getTransformation(0, 0, 0, 0, M_PI_2, 0) * tm_aff;
			break;
		case -3:
			tm_aff = pcl::getTransformation(0, 0, 0, 0, -M_PI_2, 0) * tm_aff;
			break;
		default:
			break;
		}
		axis_o_trans = tm_aff * axis_o_origin;
		axis_x_trans = tm_aff * axis_x_origin;
		axis_y_trans = tm_aff * axis_y_origin;
		axis_z_trans = tm_aff * axis_z_origin;
		axis_x_vector = axis_x_trans - axis_o_trans;
		axis_y_vector = axis_y_trans - axis_o_trans;
		axis_z_vector = axis_z_trans - axis_o_trans;
		cout << "x calibration:" << endl;
		cout << axis_x_vector.x() << " " << axis_x_vector.y() << " " << axis_x_vector.z() << endl;
		cout << axis_y_vector.x() << " " << axis_y_vector.y() << " " << axis_y_vector.z() << endl;
		cout << axis_z_vector.x() << " " << axis_z_vector.y() << " " << axis_z_vector.z() << endl;

		//Y坐标校正
		int max_index_y = getMaxAbsValueIndex(axis_y_vector);
		switch (max_index_y)
		{
		case 1: break;
		case -1: break;
		case 2: break;
		case -2:
			tm_aff = pcl::getTransformation(0, 0, 0, M_PI, 0, 0) * tm_aff;
			break;
		case 3:
			tm_aff = pcl::getTransformation(0, 0, 0, -M_PI_2, 0, 0) * tm_aff;
			break;
		case -3:
			tm_aff = pcl::getTransformation(0, 0, 0, M_PI_2, 0, 0) * tm_aff;
			break;
		default:
			break;
		}
		axis_o_trans = tm_aff * axis_o_origin;
		axis_x_trans = tm_aff * axis_x_origin;
		axis_y_trans = tm_aff * axis_y_origin;
		axis_z_trans = tm_aff * axis_z_origin;
		axis_x_vector = axis_x_trans - axis_o_trans;
		axis_y_vector = axis_y_trans - axis_o_trans;
		axis_z_vector = axis_z_trans - axis_o_trans;
		cout << "y calibration:" << endl;
		cout << axis_x_vector.x() << " " << axis_x_vector.y() << " " << axis_x_vector.z() << endl;
		cout << axis_y_vector.x() << " " << axis_y_vector.y() << " " << axis_y_vector.z() << endl;
		cout << axis_z_vector.x() << " " << axis_z_vector.y() << " " << axis_z_vector.z() << endl;

		return tm_aff;
	}


	/*利用PCA计算将点云转正的旋转矩阵*/
	Eigen::Affine3f getPCATransfrom(PointCloudT::Ptr& cloud) {

		PointCloudT::Ptr voxelCloud(new PointCloudT);
		voxelFilter(cloud, voxelCloud, 15, 15, 15);
		PointCloudT::Ptr sorCloud(new PointCloudT);
		sorFilter(voxelCloud, sorCloud, 50, 1);


		PointCloudT::Ptr downsample = sorCloud;

		//利用PCA计算主轴并返回沿主轴转正的仿射矩阵
		Eigen::Vector4f pcaCentroid;//质心点(4x1)
		pcl::compute3DCentroid(*downsample, pcaCentroid);

		Eigen::Matrix4f tm = Eigen::Matrix4f::Identity();

		pclgeomerty::getPCATransfrom(*downsample, pcaCentroid, tm);

		PointCloudT::Ptr transformedCloud(new PointCloudT);
		pcl::transformPointCloud(*cloud, *transformedCloud, tm);


		//初始转正的转换矩阵
		Eigen::Affine3f tm_aff(tm);
		Eigen::Affine3f tm_aff_standard = standardization(tm_aff);

		PointCloudT::Ptr transformedCloud2(new PointCloudT);
		pcl::transformPointCloud(*cloud, *transformedCloud2, tm_aff_standard);


		float rx, ry, rz, tx, ty, tz;
		pcl::getTranslationAndEulerAngles(tm_aff_standard, tx, ty, tz, rx, ry, rz);
		std::cout << "Final filter: x,y,z,rx,ry,rz = " << tx << " " << ty << " " << tz << " " << rx * 180 / M_PI << " " << ry * 180 / M_PI << " " << rz * 180 / M_PI << std::endl;

		return tm_aff_standard;
	}

	/*利用修正后的PCA将点云转正的旋转矩阵*/
	Eigen::Affine3f getPCAFitTransform(PointCloudT::Ptr& cloud, double fitStepDistance = 25) {

		//利用PCA计算主轴并返回沿主轴转正的仿射矩阵
		Eigen::Vector4f pcaCentroid1;//质心点(4x1)
		pcl::compute3DCentroid(*cloud, pcaCentroid1);

		Eigen::Matrix4f tm = Eigen::Matrix4f::Identity();
		pclgeomerty::getPCATransfrom(*cloud, pcaCentroid1, tm);

		//初始转正的转换矩阵
		Eigen::Affine3f tm_aff(tm);
		Eigen::Affine3f tm_aff_standard = standardization(tm_aff);

		PointCloudT::Ptr transformedCloud(new PointCloudT);
		pcl::transformPointCloud(*cloud, *transformedCloud, tm_aff_standard);


		/******************************************************************/
		pcl::PointXYZ min_p, max_p;
		pcl::getMinMax3D(*transformedCloud, min_p, max_p);

		double dx = max_p.x - min_p.x;	//点云x方向的长度
		int step = int(dx / fitStepDistance);
		double revisedfitStep = dx / step;

		std::vector<Eigen::Vector4f> centroidPoints;

		cout << "step-1:" << step - 1 << endl;
		for (int i = 1; i < step - 1; i++)	//去掉首尾
		{
			double minStepX = min_p.x + i * revisedfitStep;
			double maxStepX = (min_p.x + (i + 1) * revisedfitStep);

			PointCloudT::Ptr cropedCloud(new PointCloudT);
			cropFilter(transformedCloud, cropedCloud, minStepX, maxStepX, min_p.y, max_p.y, min_p.z, max_p.z);

#ifdef DEBUG
			{
				boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = getViewer();
				viewerCloud(viewer, transformedCloud, 255, 0, 0, 1, 0);
				viewerCloud(viewer, cropedCloud, 0, 0, 255, 2, 2);
				viewerStop(viewer);
			}
#endif


		//此处计算质心点
			Eigen::Vector4f pcaCentroid;//质心点(4x1)
			pcl::compute3DCentroid(*cropedCloud, pcaCentroid);
			centroidPoints.push_back(pcaCentroid);

			cout << pcaCentroid[0] << ":" << pcaCentroid[1] << ":"
				<< pcaCentroid[2] << ":" << pcaCentroid[3] << endl;
		}

		//拟合centroidPoints成一条直线
		cout << "vertical cut finish" << endl;

		vector<Point3f> fitPoints;
		for (size_t i = 0; i < centroidPoints.size(); i++)
		{
			fitPoints.push_back(Point3f(centroidPoints[i].x(), centroidPoints[i].y(), centroidPoints[i].z()));
		}
		Vec6f line;
		if (fitPoints.size() < 1)
		{
			return Eigen::Affine3f();
		}
		fitLine(fitPoints, line, CV_DIST_HUBER, 0, 1e-2, 1e-2);
		cout << line[0] << ":" << line[1] << ":" << line[2] << ":"
			<< line[3] << ":" << line[4] << ":" << line[5] << endl;
		/******************************************************************/

		//pcl::getTransformation(0,0,0,)
		Eigen::Matrix3f trans = pclgeomerty::Calculation(
			Eigen::Vector3f(line[0], line[1], line[2]),
			Eigen::Vector3f(1, 0, 0));
		float rx, ry, rz, tx, ty, tz;
		pcl::getTranslationAndEulerAngles(Eigen::Affine3f(trans), tx, ty, tz, rx, ry, rz);
		cout << tx << ":" << ty << ":" << tz << ":" << rx * 180 / M_PI << ":" << ry * 180 / M_PI << ":" << rz * 180 / M_PI << endl;

		Eigen::Affine3f final_trans = Eigen::Affine3f(trans) * tm_aff_standard;

		PointCloudT::Ptr finalCloud(new PointCloudT);
		pcl::transformPointCloud(*cloud, *finalCloud, final_trans);


		return final_trans;
	}

	Point2f convertPointXYZToPicPoint(pcl::PointXYZ point, float picWidth, pcl::PointXYZ min_p1, pcl::PointXYZ max_p1) {
		Point2f newPoint;

		float dx = abs(min_p1.x - max_p1.x);
		float dy = abs(min_p1.y - max_p1.y);
		float dz = abs(min_p1.z - max_p1.z);

		newPoint.x = point.y - min_p1.y + (picWidth - dy) / 2.;
		newPoint.y = point.z - min_p1.z + (picWidth - dz) / 2.;
		return newPoint;
	}

	pcl::PointXYZ convertPicPointToPointXYZ(Point2f point, float picWidth, pcl::PointXYZ min_p1, pcl::PointXYZ max_p1) {
		pcl::PointXYZ newPoint;

		float dx = abs(min_p1.x - max_p1.x);
		float dy = abs(min_p1.y - max_p1.y);
		float dz = abs(min_p1.z - max_p1.z);

		double x, y, z;

		x = 0;
		y = point.x + min_p1.y - (picWidth - dy) / 2.;
		z = point.y + min_p1.z - (picWidth - dz) / 2.;

		newPoint.x = x;
		newPoint.y = y;
		newPoint.y = z;

		return newPoint;
	}

	std::vector<PointCloudT> getClusters(PointCloudT::Ptr &cloud_in, double tolerance, int min_cluster_size = 200) {

		std::vector<pcl::PointIndices> cluster_indices;

		euclideanClusterExtraction(cloud_in, cluster_indices, tolerance, min_cluster_size, 100000);

		std::vector<PointCloudT> clusters;

		for (size_t i = 0; i < cluster_indices.size(); i++)
		{
			PointCloudT::Ptr cloud_cluster(new PointCloudT);

			pcl::PointIndices::Ptr inliers(new pcl::PointIndices(cluster_indices[i]));

			extractIndicesFilter(cloud_in, cloud_cluster, inliers);

			std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;


			clusters.push_back(*cloud_cluster);
#ifdef DEBUG
			{
				boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = getViewer();
				viewerCloud(viewer, cloud_in, 0, 255, 0, 1, 0);
				viewerCloud(viewer, cloud_cluster, 255, 0, 0, 2, 5);
				viewerStop(viewer);
			}
#endif
		}

		return clusters;
	}

	std::vector<PointCloudT::Ptr> getClustersCong(PointCloudT::Ptr &cloud_in, double tolerance, int min_cluster_size = 200) {

		std::vector<pcl::PointIndices> cluster_indices;

		euclideanClusterExtraction(cloud_in, cluster_indices, tolerance, min_cluster_size, 100000);

		std::vector<PointCloudT::Ptr> clusters;

		for (size_t i = 0; i < cluster_indices.size(); i++)
		{
			PointCloudT::Ptr cloud_cluster(new PointCloudT);

			pcl::PointIndices::Ptr inliers(new pcl::PointIndices(cluster_indices[i]));

			extractIndicesFilter(cloud_in, cloud_cluster, inliers);

			std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
 

			clusters.push_back(cloud_cluster);
#ifdef DEBUG
			{
				boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = getViewer();
				viewerCloud(viewer, cloud_cluster, 255, 0, 0, 1, 5);
				viewerStop(viewer);
			}
#endif
		}

		return clusters;
	}

	void groupRealCloud(const pcl::PointCloud<pcl::PointXYZ> &transformed_cloud_in, std::vector<PointCloudT> &cloud_out, double max_width) {

		double min_angle = 10;

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTmp(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::copyPointCloud(transformed_cloud_in, *cloudTmp);
		//std::vector<PointCloudT> tmpPointClouds = getClusters(cloudTmp, 35, 200);
		std::vector<PointCloudT> tmpPointClouds = getClusters(cloudTmp, Tolerance, MinClusterSize);
		std::vector<Eigen::Vector4f> tmpPcaCentroids;


		double meanDy = max_width;

		for (size_t i = 0; i < tmpPointClouds.size(); i++)
		{
			Eigen::Vector4f subPcaCentroid;//质心点(4x1)
			pcl::compute3DCentroid(tmpPointClouds[i], subPcaCentroid);//预估质心点
			tmpPcaCentroids.push_back(subPcaCentroid);
		}

		//needMerge
		vector<pcl::PointCloud<pcl::PointXYZ>> tmpNewPointClouds;

		pclgeomerty::LINE referenceLine;

		referenceLine = pclgeomerty::LINE(pcl::PointXYZ(0, 0, 0), pcl::PointXYZ(1, 0, 0));

		while (tmpPcaCentroids.size() > 0)
		{
			Eigen::Vector4f subPcaCentroid1 = tmpPcaCentroids[tmpPcaCentroids.size() - 1];

			pcl::PointCloud<pcl::PointXYZ> subCloud1 = tmpPointClouds[tmpPointClouds.size() - 1];

			tmpPcaCentroids.pop_back();
			tmpPointClouds.pop_back();

			vector<int> waitRemoveIndex;
			//此合并考虑点云被分成多段,应能比上边慢

			for (size_t i = 0; i < tmpPcaCentroids.size(); i++)
			{
				Eigen::Vector4f subPcaCentroid2 = tmpPcaCentroids[i];

				pcl::PointCloud<pcl::PointXYZ> subCloud2 = tmpPointClouds[i];

				pcl::PointXYZ point1 = pcl::PointXYZ(subPcaCentroid1(0), subPcaCentroid1(1), subPcaCentroid1(2));
				pcl::PointXYZ point2 = pcl::PointXYZ(subPcaCentroid2(0), subPcaCentroid2(1), subPcaCentroid2(2));

				pclgeomerty::LINE pcaCentroidLine = pclgeomerty::LINE(point1, point2);

				double realAngle = abs(pclgeomerty::lsangle(referenceLine, pcaCentroidLine)) * 180 / M_PI;
				if (realAngle < min_angle || (180 - realAngle) < min_angle)//说明平行 需要合并
				{
					subCloud1 = subCloud1 + subCloud2;
					waitRemoveIndex.push_back(i);
					continue;
				}

				pcl::PointXYZ min_sub_p1, max_sub_p1;
				pcl::PointXYZ min_sub_p2, max_sub_p2;

				pcl::getMinMax3D(subCloud1, min_sub_p1, max_sub_p1);
				pcl::getMinMax3D(subCloud2, min_sub_p2, max_sub_p2);

				//当不平行但是工件长度在另一个工件长度范围内说明可以合并
				if ((min_sub_p1.y<min_sub_p2.y&&max_sub_p1.y>max_sub_p2.y) || min_sub_p1.y > min_sub_p2.y&&max_sub_p1.y < max_sub_p2.y)
				{
					subCloud1 = subCloud1 + subCloud2;
					waitRemoveIndex.push_back(i);
					continue;
				}

				//工件间最小距离小于阈值进行合并
				//float merginThresh = 20;
				float merginThresh = 10;
				if (abs(subPcaCentroid1(1) - subPcaCentroid2(1)) < meanDy + merginThresh)
				{
					subCloud1 = subCloud1 + subCloud2;
					waitRemoveIndex.push_back(i);
					continue;
				}
			}
			tmpNewPointClouds.push_back(subCloud1);

			for (size_t i = 0; i < waitRemoveIndex.size(); i++)
			{
				tmpPcaCentroids.erase(tmpPcaCentroids.begin() + waitRemoveIndex[i] - i);
				tmpPointClouds.erase(tmpPointClouds.begin() + waitRemoveIndex[i] - i);
			}

		}
		cloud_out = tmpNewPointClouds;
	}
	void extractIndices(PointCloud::Ptr &cloud, const pcl::PointIndices::Ptr &indices, const bool &negative) {
		PointCloud::Ptr cloudTmp(new PointCloud);
		*cloudTmp = *cloud;
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud_p(new pcl::PointCloud<pcl::PointXYZ>);
		cloud->clear();
		extract.setInputCloud(cloudTmp);
		extract.setIndices(indices);
		extract.setNegative(negative);
		extract.filter(*cloud);
	}
	void sacSegmentation(PointCloud::Ptr &cloud, pcl::ModelCoefficients::Ptr coefficients,const int &model, const int &max_iterations, const double &threshold) {
		
		pcl::PointIndices::Ptr indices(new pcl::PointIndices);

		pcl::SACSegmentation<pcl::PointXYZ> seg;
		seg.setOptimizeCoefficients(true);	//设置对估计的模型参数进行优化处理
		seg.setModelType(model);//pcl::SACMODEL_PLANE
		seg.setMethodType(pcl::SAC_RANSAC);	// 设置用哪个随机参数估计方法
		seg.setMaxIterations(max_iterations);
		seg.setDistanceThreshold(threshold);	//设置判断是否为模型内点的距离阈值
		seg.setInputCloud(cloud);
		seg.segment(*indices, *coefficients);
		extractIndices(cloud, indices, false);
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

	string savePointCloudCrossPic(PointCloudT::Ptr &cloud_in,float min_x, float max_x, int sequenceNo, double half_project_length, int cloud_pos, float head_offset, float mid_offset, float tail_offset,string out_name) {
		std::cout << "max_x:" << max_x << std::endl;
		std::cout << "min_x:" << min_x << std::endl;
		if (cloud_pos == cloud_head) {
		min_x -= head_offset;
		max_x -= mid_offset;
	}
	else if (cloud_pos == cloud_mid) {
		min_x += mid_offset;
		max_x -= mid_offset;
	}
	else if (cloud_pos == cloud_tail) {
		min_x += mid_offset;
		max_x += tail_offset;
		if (max_x < min_x) {
			float tmp = min_x;
			max_x = min_x;
			min_x = tmp;
			if (fabs(max_x - min_x) > tail_offset) {
				max_x = min_x + tail_offset;
			}
		}

	}
	std::cout << "after max_x:" << max_x << std::endl;
	std::cout << "after min_x:" << min_x << std::endl;
	std::cout << "cloud_pos:" << cloud_pos << std::endl;
	std::cout << "head_offset:" << head_offset << std::endl;
	std::cout << "mid_offset:" << mid_offset << std::endl;
	std::cout << "tail_offset:" << tail_offset << std::endl;
#ifdef DEBUG
	{
		std::cout << "max_x:" << max_x << std::endl;
		std::cout <<"min_x:"<< min_x << std::endl;
		std::cout << "cloud_pos:" << cloud_pos << std::endl;
		std::cout << "head_offset:" << head_offset << std::endl;
		std::cout << "mid_offset:" << mid_offset << std::endl;
		std::cout << "tail_offset:" << tail_offset << std::endl;

		//PointCloudT::Ptr cloud_in_tmp(new PointCloudT);
		//*cloud_in_tmp = *cloud_in;
		//pcl::PassThrough<pcl::PointXYZ> pass;
		//pass.setInputCloud(cloud_in_tmp);
		//pass.setFilterFieldName("x");
		//pass.setFilterLimits(min_x, max_x);
		//pass.setFilterLimitsNegative(false);
		//pass.filter(*cloud_in_tmp);
		//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = getViewer();
		//viewerCloud(viewer, cloud_in, 255, 0, 0, 1, 0);
		//viewerCloud(viewer, cloud_in_tmp, 0, 255, 0, 2, 2);
		//viewerStop(viewer);

	}
#endif
	//找平面
	PointCloudT::Ptr cloud_plane(new PointCloudT);
	*cloud_plane = *cloud_in;
	pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
	sacSegmentation(cloud_plane, coefficients_plane, pcl::SACMODEL_PLANE, 1000, 0.5);
	if (coefficients_plane->values[1] < 0) {
		coefficients_plane->values[1] = -coefficients_plane->values[1];
		coefficients_plane->values[2] = -coefficients_plane->values[2];
	}
	Eigen::Vector3f v1(coefficients_plane->values[0], coefficients_plane->values[1], coefficients_plane->values[2]);

#ifdef DEBUG
	{
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = getViewer();
		viewerCloud(viewer, cloud_in, 255, 0, 0, 1, 0);
		viewerCloud(viewer, cloud_plane, 0, 255, 0, 2, 2);
		viewerStop(viewer);
	}
#endif 
	//截取铝型材中部2*half_project_length长度的点云
	pcl::PointXYZ min_p, max_p;
	pcl::getMinMax3D(*cloud_in, min_p, max_p);
	PointCloudT::Ptr cropedProjectCloud(new PointCloudT);
	cropFilter(cloud_in, cropedProjectCloud,
		(min_p.x + max_p.x) / 2 - half_project_length, (min_p.x + max_p.x) / 2 + half_project_length,
		min_p.y, max_p.y,
		min_p.z, max_p.z);
	//防抖
	cropFilter(cloud_plane, cloud_plane,
		(min_p.x + max_p.x) / 2 - (half_project_length+100), (min_p.x + max_p.x) / 2 + (half_project_length+100),
		min_p.y, max_p.y,
		min_p.z, max_p.z);
	if (cropedProjectCloud->size() == 0) {
		std::cout << "here crop head" << std::endl;
		cropFilter(cloud_in, cropedProjectCloud,
			min_p.x, min_p.x  + 2*half_project_length,
			min_p.y, max_p.y,
			min_p.z, max_p.z);
		cropFilter(cloud_plane, cloud_plane,
			min_p.x, min_p.x + 2 * (half_project_length+100),
			min_p.y, max_p.y,
			min_p.z, max_p.z);
	}
#ifdef DEBUG
	{
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = getViewer();
		//viewerCloud(viewer, cloud_in, 255, 0, 0, 1, 0);
		viewerCloud(viewer, cropedProjectCloud, 0, 255, 0, 2, 2);
		viewerStop(viewer);
	}
#endif 
	//设定投影平面为X=0(平面方程:A*X+B*Y+C*Z+D=0)
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	coefficients->values.resize(4);
	coefficients->values[0] = 1.0;	//A
	coefficients->values[1] = 0;	//B
	coefficients->values[2] = 0;	//C
	coefficients->values[3] = 0;	//D
	//执行投影
	PointCloudT::Ptr projCloud(new PointCloudT);
	projectInliersFilter(cropedProjectCloud, projCloud, coefficients, pcl::SACMODEL_PLANE);

	//对投影点云进行居中操作，便于映射到2D图像上
	pcl::PointXYZ min_proj, max_proj;
	pcl::getMinMax3D(*projCloud, min_proj, max_proj);
	int ymin = PICWIDTH_2 - (min_proj.y + max_proj.y) / 2;
	int zmin = PICWIDTH_2 - (min_proj.z + max_proj.z) / 2;
	Eigen::Affine3f transform(Eigen::Translation3f(0, ymin, zmin));
	pcl::transformPointCloud(*projCloud, *projCloud, transform);
	pcl::transformPointCloud(*cloud_plane, *cloud_plane, transform);
	pcl::PointXYZ min_proj_plane, max_proj_plane;
	pcl::getMinMax3D(*cloud_plane, min_proj_plane, max_proj_plane);
#ifdef DEBUG
	{
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = getViewer();
		viewerCloud(viewer, projCloud, 255, 0, 0, 1, 0);
		viewerCloud(viewer, cloud_plane, 0, 0, 255, 3, 2);
		viewerStop(viewer);
	}
#endif 
	Eigen::Vector3f v3(0, 1, 0);
	Eigen::Vector3f direction = v1.normalized().cross(v3);
	Point2f start;
	Point2f end;
	if (fabs(max_proj_plane.z - min_proj_plane.z) < 10) {
		start.x = (min_proj_plane.z + max_proj_plane.z) / 2.0;
		end.x = (min_proj_plane.z + max_proj_plane.z) / 2.0;
	}
	else if (direction.x() < 0) {
		start.x = min_proj_plane.z;
		end.x = max_proj_plane.z;
	}
	else {
		start.x = max_proj_plane.z;
		end.x = min_proj_plane.z;
	}
	start.y = max_proj_plane.y;
	end.y = min_proj_plane.y;
	float A = coefficients_plane->values[0];
	float B = coefficients_plane->values[1];
	float C = coefficients_plane->values[2];
	float D = coefficients_plane->values[3];

	if (C != 0) {
		end.x = -B / C * end.y + B / C * start.y + start.x;
	}
	

	Vec4f plane_line;
	plane_line[0] = start.x;
	plane_line[1] = start.y;
	plane_line[2] = end.x;
	plane_line[3] = end.y;

	if (cloud_plane->size() <= 0) {
		plane_line[0] = 0;
		plane_line[1] = 0;
		plane_line[2] = 0;
		plane_line[3] = 0;
	}
	std::cout << "plane_line[0]:" << plane_line[0] << std::endl;
	std::cout << "plane_line[1]:" << plane_line[1] << std::endl;
	std::cout << "plane_line[2]:" << plane_line[2] << std::endl;
	std::cout << "plane_line[3]:" << plane_line[3] << std::endl;

#ifdef DEBUG
	{
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = getViewer();
		//viewerCloud(viewer, cloud_tt, 0, 255, 0, 2, 10);
		viewerCloud(viewer, cloud_plane, 0, 0, 255, 3, 2);
		viewerStop(viewer);
	}
#endif 
	Eigen::Matrix4f tm = Eigen::Matrix4f::Identity();
	Eigen::Affine3f tm_aff(tm);
	Eigen::Affine3f pca_partial_transform_matrix = transform * tm_aff;
	if (projCloud->size() < 10)
	{
		return "";
	}
	//赋值2D图像
	cv::Mat imageGary = cv::Mat::zeros(PICWIDTH, PICWIDTH, CV_32FC1);
	int skipCount = 0;
	for (size_t i = 0; i < projCloud->size(); i++)
	{
		pcl::PointXYZ point = projCloud->points[i];

		int x = floor(point.z);
		int y = floor(point.y);

		if (x > 0 && x < PICWIDTH && y > 0 && y < PICWIDTH)
		{
			imageGary.at<float>(y, x) = 1.0;
		}
		else
		{
			skipCount++;
			cout << "skip x:" << x << " y:" << y << endl;
		}
	}
	if (skipCount > projCloud->size() / 2.)
	{
		return "";
	}
	imageGary = imageGary * 255;

	//生成2D图像转回3D坐标的信息
	float rx, ry, rz, tx, ty, tz;
	pcl::getTranslationAndEulerAngles(pca_partial_transform_matrix.inverse(), tx, ty, tz, rx, ry, rz);
	Eigen::Vector3f min_x_trans = pca_partial_transform_matrix * Eigen::Vector3f(min_x, 0, 0);
	Eigen::Vector3f max_x_trans = pca_partial_transform_matrix * Eigen::Vector3f(max_x, 0, 0);

	if (out_name == "") {
		if (WorkFlowPath != "") {
			out_name = WorkFlowPath + "group_" + std::to_string(sequenceNo) + "#" +
				std::to_string(tx) + "_" + std::to_string(ty) + "_" + std::to_string(tz) + "#" +
				std::to_string(rx) + "_" + std::to_string(ry) + "_" + std::to_string(rz) + "#" +
				std::to_string(max_x_trans.x()) + "_" + std::to_string(min_x_trans.x()) + ".jpg";

			string plane_path = out_name.substr(0, out_name.length() - 4) + ".key";
			FILE* stream;
			stream = fopen(plane_path.c_str(), "w");
			fprintf(stream, "%.3f%s%.3f%s%.3f%s%.3f%s", plane_line[0], " ", plane_line[1], " ", plane_line[2], " ", plane_line[3], " ");
			fclose(stream);
		}
		else {
			return "";
		}
	}
	else {
		std::cout << "save plane success" << std::endl;
		string plane_path = saveModelPath +out_name + ".key";
		FILE* stream;
		stream = fopen(plane_path.c_str(), "w");
		fprintf(stream, "%.3f%s%.3f%s%.3f%s%.3f%s", plane_line[0], " ", plane_line[1], " ", plane_line[2], " ", plane_line[3], " ");
		fclose(stream);

		out_name = saveModelImgPath + out_name + ".jpg";
	}
	//cout << out_name << endl;
	if (isnan(rx) || isnan(ry) || isnan(rz) || isnan(tx) || isnan(ty) || isnan(tz))
	{
		return "";
	}
	cv::Mat out = imageGary.clone();
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	cv::morphologyEx(imageGary, out, cv::MORPH_CLOSE, element);
#ifdef DEBUGC
	imwrite(out_name, out);
#endif
	return out_name;
}
	int complementCloud(PointCloudT::Ptr &cloud_incomplete, PointCloudT::Ptr cloud_out,PointCloudT::Ptr &cloud_complete,bool up) {
	//补全y轴上下两端缺失点云
#ifdef DEBUG
	{
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = getViewer();
		viewerCloud(viewer, cloud_incomplete, 255, 0, 0, 1, 0);
		viewerCloud(viewer, cloud_complete, 0, 255, 0, 2, 1);
		viewerStop(viewer);
	}
#endif
	PointCloudT::Ptr cloud_complete_tmp(new PointCloudT);
	*cloud_complete_tmp = *cloud_complete;
	pcl::PointXYZ min_i_p, max_i_p;
	pcl::getMinMax3D(*cloud_incomplete, min_i_p, max_i_p);

	pcl::PointXYZ min_p, max_p;
	pcl::getMinMax3D(*cloud_complete, min_p, max_p);

	float min_pass = 0;
	float max_pass = 0;
	Eigen::Matrix4f tm0 = Eigen::Matrix4f::Identity();
	
	if (up) {
		min_pass = min_p.y;
		max_pass = min_p.y+fabs(max_i_p.y- min_i_p.y);
		tm0.block<3, 1>(0, 3) << 0, min_i_p.y - min_p.y, 0, 1;
	}
	else {
		min_pass = max_p.y- fabs(max_i_p.y - min_i_p.y);
		max_pass = max_p.y;
		tm0.block<3, 1>(0, 3) << 0, max_i_p.y - max_p.y, 0, 1;
	}

	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud_complete_tmp);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(min_pass, max_pass);
	pass.setFilterLimitsNegative(false);
	pass.filter(*cloud_complete_tmp);

	pcl::getMinMax3D(*cloud_complete_tmp, min_p, max_p);

	pcl::transformPointCloud(*cloud_complete_tmp, *cloud_complete_tmp, tm0);
#ifdef DEBUG
	{
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = getViewer();
		viewerCloud(viewer, cloud_incomplete, 255, 0, 0, 1, 5);
		viewerCloud(viewer, cloud_complete, 0, 0, 255, 3, 5);
		viewerCloud(viewer, cloud_complete_tmp, 0, 255, 0, 2, 1);
		viewerStop(viewer);
	}
#endif

	double score = 0;
	Eigen::Matrix4f tm = Eigen::Matrix4f::Identity();
	//icpRegistration(cloud_complete_tmp, cloud_incomplete, score,tm);
	cloud_out->clear();
	tm = tm*tm0;
	pcl::transformPointCloud(*cloud_complete, *cloud_out, tm);
#ifdef DEBUG
	{
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = getViewer();
		viewerCloud(viewer, cloud_incomplete, 255, 0, 0, 1, 0);
		viewerCloud(viewer, cloud_out, 0, 255, 0, 2, 1);
		viewerStop(viewer);
	}
#endif
	return 0;
}
	int filterRealPointCloud(char * cloud_front_file,char * cloud_file,std::vector<PointCloudT::Ptr> &originGroupClouds, PointCloudT::Ptr &cloud_real, float &min_real_x, float &max_real_x, double cloud_length, double stepDistanceThresh ,double poleWidthThresh , int poleSizeThresh ,double poleOffesetThresh) {
		WorkFlowPath = "";
		int res = fileExist(cloud_file);
		if (res == -1) {
			LOG(ERRORL) << "cloud_file点云文件不存在";
			throw exception((std::string(__FUNCTION__) + ":cloud_file点云文件不存在").c_str());
		}
#ifdef DEBUGC
		WorkFlowPath = getFilePath(cloud_file) + "Algorithm\\";
		createFolder(WorkFlowPath);
#endif
		PointCloudT::Ptr cloud = LoadPointCloud(cloud_file);
		pcl::PointXYZ min_cloud_p, max_cloud_p;
		pcl::getMinMax3D(*cloud, min_cloud_p, max_cloud_p);
		std::cout << "**cloud_length**:" << fabs(max_cloud_p.x - min_cloud_p.x) << std::endl;
		if (cloud_length < 400) {
			LOG(ERRORL) << "cloud_length<200错误";
			throw exception((std::string(__FUNCTION__) + ":cloud_length<200错误").c_str());
		}
		if (fabs(max_cloud_p.x - min_cloud_p.x) < cloud_length-400) {
			res = fileExist(cloud_front_file);
			//if (res == -1) {
			//}
			if (res == 0) {
				PointCloudT::Ptr cloud_front = LoadPointCloud(cloud_front_file);
				cloud->clear();
				*cloud = *cloud_front;
			}
		}
		min_real_x = min_cloud_p.x;
		max_real_x = max_cloud_p.x;

		if (cloud->size() == 0) return -1;
		PointCloudT::Ptr voxel_cloud(new PointCloudT);
		voxelFilter(cloud, voxel_cloud, 20, 2, 2);
		if (voxel_cloud->size() == 0) return -1;
#ifdef DEBUG
		{
			boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = getViewer();
			viewerCloud(viewer, voxel_cloud, 255, 0, 0, 1, 0);
			viewerStop(viewer);
		}
#endif 
		if (voxel_cloud->size() == 0) return -1;
		PointCloudT::Ptr sor_cloud(new PointCloudT);
		//*sor_cloud = *voxel_cloud;
		sorFilter(voxel_cloud, sor_cloud, 20, 5);
#ifdef DEBUG
		{
			boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = getViewer();
			viewerCloud(viewer, voxel_cloud, 255, 0, 0, 1, 0);
			viewerStop(viewer);
		}
#endif 
		if (sor_cloud->size() == 0) return -1;
		//滤离群杂波
		PointCloudT::Ptr cluster_cloud(new PointCloudT);
		voxelFilter(sor_cloud, cluster_cloud, 10, 10, 10);
		//std::vector<PointCloudT::Ptr> clusters = getClustersCong(cluster_cloud, 100, 200);
		std::vector<PointCloudT::Ptr> clusters = getClustersCong(cluster_cloud, Tolerance, MinClusterSize);
		cluster_cloud->clear();
		for (int i = 0; i < clusters.size(); i++) {
			*cluster_cloud += *clusters[i];
		}
		if (cluster_cloud->size() == 0) return -1;
#ifdef DEBUG
		{
			boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = getViewer();
			viewerCloud(viewer, cluster_cloud, 255, 0, 0, 1, 0);
			viewerStop(viewer);
		}
#endif
		//滤杆子
		pcl::PointXYZ min_cluster_p, max_cluster_p;
		pcl::getMinMax3D(*cluster_cloud, min_cluster_p, max_cluster_p);
		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud(sor_cloud);
		pass.setFilterFieldName("x");
		pass.setFilterLimits(min_cluster_p.x, max_cluster_p.x);
		pass.setFilterLimitsNegative(false);
		pass.filter(*sor_cloud);
		pass.setInputCloud(sor_cloud);
		pass.setFilterFieldName("y");
		pass.setFilterLimits(min_cluster_p.y, max_cluster_p.y);
		pass.setFilterLimitsNegative(false);
		pass.filter(*sor_cloud);
		pass.setInputCloud(sor_cloud);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(min_cluster_p.z, max_cluster_p.z);
		pass.setFilterLimitsNegative(false);
		pass.filter(*sor_cloud);
		if (sor_cloud->size() == 0) return -1;
#ifdef DEBUG
		{
			boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = getViewer();
			viewerCloud(viewer, sor_cloud, 255, 0, 0, 1, 0);
			viewerStop(viewer);
		}
#endif
		PointCloudT::Ptr cloud_in(new PointCloudT);
		PointCloudT::Ptr cloud_pole(new PointCloudT);
		PointCloudT::Ptr sor_cloud_tmp(new PointCloudT);
		*sor_cloud_tmp = *sor_cloud;
		//将sor_cloud点云两端切去,保证cloud_real滤完后无两端杆子噪声
		pcl::PointXYZ min_p, max_p;
		pcl::getMinMax3D(*sor_cloud, min_p, max_p);
		pass.setInputCloud(sor_cloud);
		pass.setFilterFieldName("x");
		pass.setFilterLimits(min_p.x + 200, max_p.x - 200);
		pass.setFilterLimitsNegative(false);
		pass.filter(*sor_cloud);
#ifdef DEBUG
		{
			boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = getViewer();
			viewerCloud(viewer, sor_cloud_tmp, 255, 0, 0, 1, 2);
			viewerCloud(viewer, sor_cloud, 0, 255, 0, 2, 1);
			viewerStop(viewer);
		}
#endif
		*cloud_real = *sor_cloud;
		//pcl::PointXYZ min_p, max_p;
		pcl::getMinMax3D(*sor_cloud, min_p, max_p);
		
		double min_x = min_p.x;
		double max_x = min_x + stepDistanceThresh;
		bool flag = false;
		while (max_x != max_p.x) {
			if (flag) {
				min_x = max_x;
				max_x = min_x + stepDistanceThresh;
				if (max_x > max_p.x) {
					max_x = max_p.x;
					min_x = max_x - stepDistanceThresh;
				}
			}
			else {
				flag = true;
			}

			pass.setInputCloud(sor_cloud);
			pass.setFilterFieldName("x");
			pass.setFilterLimits(min_x, max_x);
			pass.setFilterLimitsNegative(false);
			pass.filter(*cloud_in);

#ifdef DEBUG
			{
				boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = getViewer();
				viewerCloud(viewer, sor_cloud, 255, 0, 0, 1, 0);
				viewerCloud(viewer, cloud_in, 0, 255, 0, 2, 2);
				viewerStop(viewer);
			}
#endif
			double min_sild = min_p.y;
			double max_slid = min_sild + 10;
			bool flag1 = false;

			cloud_pole->clear();
			while (max_slid != max_p.y) {
				if (flag1) {
					min_sild = max_slid;
					max_slid = min_sild + 10;
					if (max_slid > max_p.y) {
						max_slid = max_p.y;
						min_sild = max_slid - 10;
					}
				}
				else {
					flag1 = true;
				}


				PointCloud::Ptr cloudSlideBox(new PointCloud);
				pass.setInputCloud(cloud_in);
				pass.setFilterFieldName("y");
				pass.setFilterLimits(min_sild, max_slid);
				pass.setFilterLimitsNegative(false);
				pass.filter(*cloudSlideBox);


				pcl::PointXYZ min_p1, max_p1;
				pcl::getMinMax3D(*cloudSlideBox, min_p1, max_p1);
				if (fabs(max_p1.x - min_p1.x) < poleWidthThresh)
				{
					*cloud_pole += *cloudSlideBox;
				}
#ifdef DEBUG
				{
					boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = getViewer();
					viewerCloud(viewer, cloud_in, 255, 0, 0, 1, 0);
					viewerCloud(viewer, cloudSlideBox, 0, 255, 0, 2, 2);
					viewerStop(viewer);
				}
#endif
			}
			voxelFilter(cloud_pole, cloud_pole, 12, 2, 2);
			pcl::PointXYZ min_pole_p, max_pole_p;
			pcl::getMinMax3D(*cloud_pole, min_pole_p, max_pole_p);
			//if (cloud_pole->size() < poleSizeThresh || cloud_pole->size() == 0 || fabs(max_pole_p.x - min_pole_p.x) > stepDistanceThresh - 50) {
			if (cloud_pole->size() < poleSizeThresh|| cloud_pole->size()==0) {
				continue;
			}

			pass.setInputCloud(cloud_real);
			pass.setFilterFieldName("x");
			pass.setFilterLimits(min_pole_p.x- poleOffesetThresh, max_pole_p.x+ poleOffesetThresh);
			pass.setFilterLimitsNegative(true);
			pass.filter(*cloud_real);
#ifdef DEBUG
			{
				boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = getViewer();
				viewerCloud(viewer, cloud_in, 255, 0, 0, 1, 0);
				viewerCloud(viewer, cloud_pole, 0, 255, 0, 2, 2);
				viewerStop(viewer);
			}
#endif
		}
#ifdef DEBUG
		{
			boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = getViewer();
			viewerCloud(viewer, sor_cloud, 255, 0, 0, 1, 0);
			viewerCloud(viewer, cloud_real, 0, 255, 0, 2, 1);
			viewerStop(viewer);
		}
#endif
		//分割点云
		double max_width = 0;
		//点云拍平
		PointCloudT::Ptr cloud_real_tmp(new PointCloudT);
		*cloud_real_tmp = *cloud_real;
		//设定投影平面为Z=0(平面方程:A*X+B*Y+C*Z+D=0)
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
		coefficients->values.resize(4);
		coefficients->values[0] = 0;	//A
		coefficients->values[1] = 0;	//B
		coefficients->values[2] = 1.0;	//C
		coefficients->values[3] = 0;	//D
		projectInliersFilter(cloud_real_tmp, cloud_real_tmp, coefficients, pcl::SACMODEL_PLANE);
#ifdef DEBUG
		{
	
			boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = getViewer();
			viewerCloud(viewer, cloud_real, 255, 0, 0, 1, 0);
			viewerCloud(viewer, cloud_real_tmp, 0, 255, 0, 2, 2);
			viewerStop(viewer);

		}
#endif
		//std::vector<PointCloudT> cloud_widths = getClusters(cloud_real, 35, 200);
		std::vector<PointCloudT> cloud_widths = getClusters(cloud_real_tmp, 40, 100);
		//std::vector<PointCloudT> cloud_widths = getClusters(cloud_real, 30, 50);

		//if (cloud_widths.size() > 1) {
		//	std::sort(cloud_widths.begin(), cloud_widths.end(), [](PointCloudT &a, PointCloudT &b) {
		//		pcl::PointXYZ min_a_p, max_a_p;
		//		pcl::getMinMax3D(a, min_a_p, max_a_p);
		//		pcl::PointXYZ min_b_p, max_b_p;
		//		pcl::getMinMax3D(b, min_b_p, max_b_p);
		//		return max_a_p.y+ min_a_p.y/2.0 > max_b_p.y + min_b_p.y/2.0;
		//	});
		//}
		vector<double> widths;
		for (int i = 0; i < cloud_widths.size();i++) {
			pcl::PointXYZ min_width_p, max_width_p;
			pcl::getMinMax3D(cloud_widths[i], min_width_p, max_width_p);
			double width = fabs(max_width_p.y - min_width_p.y);
			widths.push_back(width);
		}
		if (widths.size() > 1) {
			std::sort(widths.begin(), widths.end(), [](const double &a, const double &b) {
				return a> b;
		});
		}
		if (widths.size() >= 1) {
			max_width = widths[0];
		}


		
		//std::vector<PointCloudT> groupClouds;
		//groupRealCloud(*cloud_real, groupClouds, max_width);
		std::vector<float> cloud_ys;
		std::vector<PointCloudT> groupClouds;
		bool skip = false;
		for (int i = 0; i < cloud_widths.size(); i++) {
			PointCloudT::Ptr cloud_group(new PointCloudT);
			pcl::PointXYZ min_width_p, max_width_p;
			pcl::getMinMax3D(cloud_widths[i], min_width_p, max_width_p);
			double width = fabs(max_width_p.y - min_width_p.y);
			Eigen::Vector4f pcaCentroid;//质心点(4x1)
			pcl::compute3DCentroid(cloud_widths[i], pcaCentroid);
			float cloud_y = pcaCentroid.y();
			skip = false;
			for (int i = 0; i < cloud_ys.size(); i++) {
				std::cout << "fabs(cloud_y - cloud_ys[i])" << fabs(cloud_y - cloud_ys[i]) << std::endl;
				if (fabs(cloud_y - cloud_ys[i]) <40) {
					skip = true;
					break;
				}
			}
			cloud_ys.push_back(cloud_y);
			if (fabs(max_width - width) < 80&& skip==false) {
				pass.setInputCloud(cloud_real);
				pass.setFilterFieldName("y");
				pass.setFilterLimits(min_width_p.y - 10, max_width_p.y + 10);
				pass.setFilterLimitsNegative(false);
				pass.filter(*cloud_group);
				originGroupClouds.push_back(cloud_group);
			}
		}

		pcl::PointXYZ min_real_p, max_real_p;
		pcl::getMinMax3D(*cloud_real, min_real_p, max_real_p);
		if (originGroupClouds.size() > 1) {
			std::sort(originGroupClouds.begin(), originGroupClouds.end(), [](const PointCloudT::Ptr &cloud1, const PointCloudT::Ptr &cloud2) {
				pcl::PointXYZ min_1, max_1;
				pcl::getMinMax3D(*(cloud1), min_1, max_1);
				pcl::PointXYZ min_2, max_2;
				pcl::getMinMax3D(*(cloud2), min_2, max_2);
				return min_1.y > min_2.y;
			});
		}
		if (originGroupClouds.size() > 3) {
			//补全y轴上下两端缺失点云
			int midIndex = floor(originGroupClouds.size()*1.0 / 2.0);
			//complementCloud(originGroupClouds[originGroupClouds.size() - 1], originGroupClouds[originGroupClouds.size() - 1], originGroupClouds[midIndex], false);
			//complementCloud(originGroupClouds[0], originGroupClouds[0], originGroupClouds[midIndex], true);
			complementCloud(originGroupClouds[originGroupClouds.size() - 1], originGroupClouds[originGroupClouds.size() - 1], originGroupClouds[originGroupClouds.size() - 2], false);
			complementCloud(originGroupClouds[0], originGroupClouds[0], originGroupClouds[1], true);
		}
#ifdef DEBUG
		{
			for (int i = 0; i < originGroupClouds.size(); i++) {
				boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = getViewer();
				viewerCloud(viewer, cloud_real, 255, 0, 0, 1, 0);
				viewerCloud(viewer, originGroupClouds[i], 0, 255, 0, 2, 2);
				viewerStop(viewer);
			}
		}
#endif
#ifdef DEBUGC
		for (int i = 0; i < originGroupClouds.size(); i++) {
			if (WorkFlowPath != "") {
				std::string n = std::to_string(i);
				pcl::io::savePLYFileBinary(WorkFlowPath + "cloud_group" + n + ".ply", *(originGroupClouds[i]));
			}
		}

#endif 
		return 0;
	}
}