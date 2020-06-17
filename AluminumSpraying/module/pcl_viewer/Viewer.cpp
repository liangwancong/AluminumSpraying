#include "Viewer.h"
void printUsage(const char* progName)
{
	std::cout << "\n\nUsage: " << progName << " <cloud.pcd> <downsampled_cloud.pcd> <inlier_cloud.pcd> <border.pcd> [options]\n\n"
		<< "Options:\n"
		<< "-------------------------------------------\n"
		<< "-h           this help\n"
		<< "-mk          mean K\n"
		<< "-sdm         std dev mul\n"
		<< "-r           radius for border check \n"
		<< "-min_pts     minimum neighbors in radius \n"
		<< "-min_pl      minimum points in z axes \n"
		<< "-min_std     minimum stdev that is not border points \n"
		<< "-max_keep    the first keep z range contains points number \n"
		<< "\n\n";
}
std::string print4x4Matrix(const Eigen::Matrix4d & matrix)
{
	printf("Rotation matrix :\n");
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
	printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
	printf("Translation vector :\n");
	printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
	std::string res = "";
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			res += std::to_string(matrix(i, j)) + " ";
	return res;
}
void print3x3Matrix(const Eigen::Matrix3d & matrix, const char * name)
{
	printf("Rotation matrix %s:\n", name);
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
	printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> getViewer() {
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer);
	viewer->addCoordinateSystem(200);
	viewer->setBackgroundColor(1.0, 1.0, 1.0);
	return (viewer);
}
void viewerCloud(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, PointCloud::Ptr &cloud, const int &R, const int &G, const int &B, const int &num, const int &size) {
	if (cloud->size() > 0) {
		pcl::visualization::PointCloudColorHandlerCustom<PointT> color_handler(cloud, R, G, B); //转换到原点的点云相关
		std::string cloudNum = std::to_string(num);
		viewer->addPointCloud(cloud, color_handler, "cloud" + cloudNum);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "cloud" + cloudNum);
	}
}
void viewerCloud(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, PointCloud &cloud, const int &R, const int &G, const int &B, const int &num, const int &size) {
	PointCloud::Ptr cloudTmp(new PointCloud);
	*cloudTmp = cloud;
	if (cloudTmp->size() > 0) {
		pcl::visualization::PointCloudColorHandlerCustom<PointT> color_handler(cloudTmp, R, G, B); //转换到原点的点云相关
		std::string cloudNum = std::to_string(num);
		viewer->addPointCloud(cloudTmp, color_handler, "cloud" + cloudNum);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "cloud" + cloudNum);
	}
}
void viewerArrow(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, const PointT &xp, const PointT &yp, const PointT &zp, const PointT &op, const int &color_mode) {
	if (color_mode == 0) {
		viewer->addArrow(xp, op, 1.0, 0.0, 0.0, false, "arrow_x");
		viewer->addArrow(yp, op, 0.0, 1.0, 0.0, false, "arrow_y");
		viewer->addArrow(zp, op, 0.0, 0.0, 1.0, false, "arrow_z");
	}
	else {
		viewer->addArrow(xp, op, 1.0, 1.0, 0.0, false, "arrow_x");
		viewer->addArrow(yp, op, 0.0, 1.0, 1.0, false, "arrow_y");
		viewer->addArrow(zp, op, 1.0, 0.0, 1.0, false, "arrow_z");
	}
}
void viewerArrow(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, const Eigen::Vector3f &v_x, const Eigen::Vector3f &v_y, const Eigen::Vector3f &v_z, const Eigen::Vector3f &oop, const int & color_mode) {
	PointT op;
	op.x = oop.x();
	op.y = oop.y();
	op.z = oop.z();
	PointT xp;
	double len = 100;
	xp.x = len * v_x.x() + op.x;
	xp.y = len * v_x.y() + op.y;
	xp.z = len * v_x.z() + op.z;
	PointT yp;
	yp.x = len * v_y.x() + op.x;
	yp.y = len * v_y.y() + op.y;
	yp.z = len * v_y.z() + op.z;
	PointT zp;
	zp.x = len * v_z.x() + op.x;
	zp.y = len * v_z.y() + op.y;
	zp.z = len * v_z.z() + op.z;
	if (color_mode == 0) {
		viewer->addArrow(xp, op, 1.0, 0.0, 0.0, false, "arrow_x");
		viewer->addArrow(yp, op, 0.0, 1.0, 0.0, false, "arrow_y");
		viewer->addArrow(zp, op, 0.0, 0.0, 1.0, false, "arrow_z");
	}
	else {
		viewer->addArrow(xp, op, 1.0, 0.0, 1.0, false, "arrow_x");//粉
		viewer->addArrow(yp, op, 1.0, 1.0, 0.0, false, "arrow_y");//黄
		viewer->addArrow(zp, op, 0.0, 1.0, 1.0, false, "arrow_z");//蓝绿
	}
}
void viewBox(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, const Eigen::Vector4f &tm, const Eigen::Vector3f &center, const Eigen::Vector3f &whd, const int &num) {
	const Eigen::Quaternionf bboxQ1(tm.block<3, 3>(0, 0));
	const Eigen::Vector3f    bboxT1(center);
	std::string boxNum = std::to_string(num);
	viewer->addCube(bboxT1, bboxQ1, whd(0), whd(1), whd(2), "bbox" + boxNum);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "bbox" + boxNum);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "bbox" + boxNum);
}
void viewerStop(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer) {
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
	}
}

