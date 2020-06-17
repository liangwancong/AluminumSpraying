#include "Registration.h"

void cornerRegistration(PointCloud::Ptr &cornerCloudSource, PointCloud::Ptr &cornerCloudTarget, bool isRotation, Eigen::Matrix4f &tm) {
	//get transform
	if (isRotation == false) {
		Eigen::Vector4f pcaCentroidSource;
		pcl::compute3DCentroid(*cornerCloudSource, pcaCentroidSource);
		Eigen::Vector4f pcaCentroidTarget;
		pcl::compute3DCentroid(*cornerCloudTarget, pcaCentroidTarget);
		pcaCentroidTarget[3] = 2;
		tm = Eigen::Matrix4f::Identity();
		tm.block<4, 1>(0, 3) = pcaCentroidTarget - pcaCentroidSource;
	}
	else {
		pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> TESVD;
		pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 transform;
		TESVD.estimateRigidTransformation(*cornerCloudSource, *cornerCloudTarget, transform);
		tm = transform;
	}
}
int icpRegistration(PointCloud::Ptr& cloudSource, PointCloud::Ptr& cloudTarget,double &score,Eigen::Matrix4f &tm) {
	if (cloudSource->size() == 0 || cloudTarget->size() == 0) {
		return -1;
	}
	//kdTree 加速搜索
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1 = getKdTree(cloudSource);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2 = getKdTree(cloudTarget);

	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setSearchMethodSource(tree1);
	icp.setSearchMethodTarget(tree2);

	//设置参数
	icp.setInputSource(cloudSource);
	icp.setInputTarget(cloudTarget);
	//icp.setMaxCorrespondenceDistance(10);
	// 最大迭代次数
	icp.setMaximumIterations(100);
	// 两次变化矩阵之间的差值
	//icp.setTransformationEpsilon(1e-10);
	// 均方误差
	icp.setEuclideanFitnessEpsilon(0.01);
	//icp.setEuclideanFitnessEpsilon(0.00001);

	PointCloud::Ptr cloudIcp(new PointCloud);
	icp.align(*cloudIcp, tm);

	if (icp.hasConverged()) {
		score = icp.getFitnessScore();
		tm = icp.getFinalTransformation();
	}else{
		return -1;
	}


	return 0;
}
double getIcpScore(PointCloud::Ptr &cloudSource, PointCloud::Ptr &cloudTarget) {
	if (cloudSource->size() == 0 || cloudTarget->size() == 0) {
		return -1;
	}
	//kdTree 加速搜索
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1 = getKdTree(cloudSource);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2 = getKdTree(cloudTarget);
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setSearchMethodSource(tree1);
	icp.setSearchMethodTarget(tree2);
	icp.setInputSource(cloudSource);
	icp.setInputTarget(cloudTarget);
	icp.setMaximumIterations(0);//最大迭代次数
	icp.setMaximumIterations(1);
	PointCloud::Ptr Cloudicp(new PointCloud);
	icp.align(*Cloudicp);
	return  icp.getFitnessScore();
}