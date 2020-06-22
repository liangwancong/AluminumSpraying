#include "pcl_filter_cloud.h"
namespace pcl_filter_cloud {
	int cluster_index = 0;
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

	string savePointCloudCrossPic(PointCloudT::Ptr &cloud_in, float min_x, float max_x, Vec4f &plane_line, Eigen::Affine3f &pca_partial_transform_matrix,int sequenceNo, double half_project_length, int cloud_pos, float head_offset, float mid_offset, float tail_offset, string out_name) {
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
			std::cout << "min_x:" << min_x << std::endl;
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
			(min_p.x + max_p.x) / 2 - (half_project_length + 100), (min_p.x + max_p.x) / 2 + (half_project_length + 100),
			min_p.y, max_p.y,
			min_p.z, max_p.z);
		if (cropedProjectCloud->size() == 0) {
			std::cout << "here crop head" << std::endl;
			cropFilter(cloud_in, cropedProjectCloud,
				min_p.x, min_p.x + 2 * half_project_length,
				min_p.y, max_p.y,
				min_p.z, max_p.z);
			cropFilter(cloud_plane, cloud_plane,
				min_p.x, min_p.x + 2 * (half_project_length + 100),
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
		//Eigen::Affine3f pca_partial_transform_matrix = transform * tm_aff;
		pca_partial_transform_matrix = transform * tm_aff;
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
			string plane_path = saveModelPath + out_name + ".key";
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
		std::vector<PointCloudT::Ptr> clusters = getClustersCong(cluster_cloud, 50, 100);
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


