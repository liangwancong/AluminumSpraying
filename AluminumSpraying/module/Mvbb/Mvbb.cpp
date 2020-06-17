#include "Mvbb.h"

namespace G3CppMvbb {
	int getCornerPointCloud(PointCloud::Ptr &cloud, PointCloud::Ptr &cornerPointsCloud) {
		if (cloud->size() == 0) {
			return -1;
		}
		ApproxMVBB::Matrix3Dyn points(3, cloud->size());
		for (int i = 0; i < cloud->size(); i++) {
			points.col(i) = ApproxMVBB::Vector3((*cloud)[i].x, (*cloud)[i].y, (*cloud)[i].z);
		}
		ApproxMVBB::OOBB oobb = ApproxMVBB::approximateMVBB(points,
			0.001,
			500,
			4, /*increasing the grid size decreases speed */
			0,
			5);

		// To make all points inside the OOBB:
		ApproxMVBB::Matrix33 A_KI = oobb.m_q_KI.matrix().transpose();  // faster to store the transformation matrix first
		auto size = points.cols();
		for (unsigned int i = 0; i < size; ++i)
		{
			oobb.unite(A_KI * points.col(i));
		}
		oobb.expandToMinExtentAbsolute(0.1);
		ApproxMVBB::OOBB::Vector3List corners = oobb.getCornerPoints();

		//±£´æ½Çµã
		cornerPointsCloud->clear();
		for (int i = 0; i < corners.size(); i++)
		{
			pcl::PointXYZ p;
			p.x = corners[i].x();
			p.y = corners[i].y();
			p.z = corners[i].z();

			cornerPointsCloud->push_back(p);
		}
		return 0;
	}
	bool sortFunz(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2)
	{
		return p1.z < p2.z;//ÉýÐòÅÅÁÐ  
	}
	bool sortFunx(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2)
	{
		return p1.x < p2.x;//ÉýÐòÅÅÁÐ  
	}
	bool sortFuny(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2)
	{
		return p1.y < p2.y;//ÉýÐòÅÅÁÐ  
	}
	void sortCornerPoint(PointCloud::Ptr &cornerPointsCloud) {
		std::vector<pcl::PointXYZ> point;
		for (int i = 0; i < (*cornerPointsCloud).size(); i++) {
			point.push_back((*cornerPointsCloud)[i]);
		}

		std::sort(point.begin(), point.end(), sortFunx);
		std::sort(point.begin(), point.end() - 4, sortFuny);
		std::sort(point.begin() + 4, point.end(), sortFuny);
		std::sort(point.begin(), point.end() - 6, sortFunz);
		std::sort(point.begin() + 2, point.end() - 4, sortFunz);
		std::sort(point.begin() + 4, point.end() - 2, sortFunz);
		std::sort(point.begin() + 6, point.end() - 0, sortFunz);
		for (int i = 0; i < point.size(); i++) {
			(*cornerPointsCloud)[i] = point[i];
		}
	}
	void sortCornerPointX(PointCloud::Ptr &cornerPointsCloud) {
		std::vector<pcl::PointXYZ> point;
		for (int i = 0; i < (*cornerPointsCloud).size(); i++) {
			point.push_back((*cornerPointsCloud)[i]);
		}

		std::sort(point.begin(), point.end(), sortFunx);
		std::sort(point.begin(), point.end() - 4, sortFuny);
		std::sort(point.begin() + 4, point.end(), sortFuny);
		std::sort(point.begin(), point.end() - 6, sortFunz);
		std::sort(point.begin() + 2, point.end() - 4, sortFunz);
		std::sort(point.begin() + 4, point.end() - 2, sortFunz);
		std::sort(point.begin() + 6, point.end() - 0, sortFunz);
		for (int i = 0; i < point.size(); i++) {
			(*cornerPointsCloud)[i] = point[i];
		}
	}
	void sortCornerPointY(PointCloud::Ptr &cornerPointsCloud) {
		std::vector<pcl::PointXYZ> point;
		for (int i = 0; i < (*cornerPointsCloud).size(); i++) {
			point.push_back((*cornerPointsCloud)[i]);
		}
		std::sort(point.begin(), point.end(), sortFuny);
		std::sort(point.begin(), point.end() - 4, sortFunx);
		std::sort(point.begin() + 4, point.end(), sortFunx);
		std::sort(point.begin(), point.end() - 6, sortFunz);
		std::sort(point.begin() + 2, point.end() - 4, sortFunz);
		std::sort(point.begin() + 4, point.end() - 2, sortFunz);
		std::sort(point.begin() + 6, point.end() - 0, sortFunz);
		for (int i = 0; i < point.size(); i++) {
			(*cornerPointsCloud)[i] = point[i];
		}
	}
	int parseCornerCloud(PointCloud::Ptr &cloud, PointCloud::Ptr &cornerCloud) {
		/*
		pcl::visualization::PCLVisualizer viewer;
		viewer.addCoordinateSystem(0.5f * 200);
		viewer.setBackgroundColor(1.0, 1.0, 1.0);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cornerCloud_h(cloud, 0, 255, 0);
		viewer.addPointCloud(cloud, cornerCloud_h, "cornerCloud");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "cornerCloud");

		while (!viewer.wasStopped())
		{
			viewer.spinOnce(100);
		}*/
		const int cornerNum = 11;
		const int headNum = 3;
		int headCount = 0;
		PointT point(0, 0, 0);
		if (cloud->size() < cornerNum) {
			std::cout << "parse size error" << std::endl;
			return -1;
		}
		for (int i = 0; i < headNum; i++) {
			if ((*cloud)[i].x == point.x && (*cloud)[i].y == point.y && (*cloud)[i].z == point.z) {
				headCount++;
			}
		}
		//std::cout << "headCount:" << headCount << std::endl;
		cornerCloud->clear();
		if (headCount == headNum) {
			//std::cout << "parse" << std::endl;
			for (int i = 0; i < cornerNum; i++) {
				if (i > 2)
					cornerCloud->push_back((*cloud)[0]);
				pcl::PointCloud<pcl::PointXYZ>::iterator index = cloud->begin();
				cloud->erase(index);
			}
		}
		else {
			//std::cout << "get" << std::endl;
			int error = getCornerPointCloud(cloud, cornerCloud);
			if (error == -1) {
				return -1;
			}
		}
		return 0;
	}
	int parseCornerCloud(PointCloud::Ptr &cloud) {
		const int cornerNum = 11;
		const int headNum = 3;
		int headCount = 0;
		PointT point(0, 0, 0);
		if (cloud->size() < cornerNum) {
			std::cout << "parse size error" << std::endl;
			return -1;
		}
		for (int i = 0; i < headNum; i++) {
			if ((*cloud)[i].x == point.x && (*cloud)[i].y == point.y && (*cloud)[i].z == point.z) {
				headCount++;
			}
		}

		if (headCount == headNum) {
			//std::cout << "parse" << std::endl;
			for (int i = 0; i < cornerNum; i++) {
				pcl::PointCloud<pcl::PointXYZ>::iterator index = cloud->begin();
				cloud->erase(index);
			}
		}
		return 0;
	}
	int makeCornerCloud(PointCloud::Ptr &cloud_in, PointCloud::Ptr &cloud_out) {
		if (cloud_in->size() == 0) {
			return -1;
		}
		PointCloud::Ptr headerCloud(new PointCloud);
		PointCloud::Ptr cornerCloud(new PointCloud);
		const int headNum = 3;
		int headCount = 0;
		PointT point(0, 0, 0);

		for (int i = 0; i < headNum; i++) {
			if ((*cloud_in)[i].x == point.x && (*cloud_in)[i].y == point.y && (*cloud_in)[i].z == point.z) {
				headCount++;
			}
		}
		if (headCount == headNum) {
			return 0;
		}
		cloud_out->clear();
		for (int i = 0; i < headNum; i++) {
			headerCloud->push_back(point);
		}
		*cloud_out += *headerCloud;

		int error = getCornerPointCloud(cloud_in, cornerCloud);
		if (error == -1) {
			return -1;
		}
		*cloud_out += *cornerCloud;
		*cloud_out += *cloud_in;
		/*
		pcl::visualization::PCLVisualizer viewer;
		viewer.addCoordinateSystem(0.5f*200);
		viewer.setBackgroundColor(1.0, 1.0, 1.0);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cornerCloud_h(cloudOut, 0, 255, 0);
		viewer.addPointCloud(cloudOut, cornerCloud_h, "cornerCloud");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "cornerCloud");
		while (!viewer.wasStopped())
		{
			viewer.spinOnce(100);
		}*/
		return 0;
	}
}
