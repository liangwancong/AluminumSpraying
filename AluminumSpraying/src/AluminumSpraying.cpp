// AluminumSpraying.cpp: 定义控制台应用程序的入口点。
//

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
#include "pcl_filter_cloud.h"
#include "CRMath.h"
#include "json_model_parse.h"
#include "parameters.h"
#include "global.h"
#include "logger.h"
#include "cv_search_feature.h"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
using namespace std;
using namespace cv;
using namespace pcl_filter_cloud;
using namespace CRMath;
using namespace json2model;
using namespace AluminumParameters;
using namespace AluminumGlobal;
using namespace logger;
using namespace cv_search_feature;


int readModelLine(char* file, Vec4f &plane_line) {
	int error = fileExist(file);
	if (error == -1) {
		std::cout << "load plane_line error" << std::endl;
		return -1;
	}
	FILE* stream;
	stream = fopen(file, "r");
	if (stream == NULL) {
		std::cout << "load plane_line error" << std::endl;
		return -1;
	}
	fscanf(stream, "%f %f %f %f  ", &plane_line[0], &plane_line[1], &plane_line[2], &plane_line[3]);
	fclose(stream);
	return 0;
}
string getMatchMat(Mat &M) {
	string matchMat;
	if (M.cols > 0 && M.rows > 0)
	{
		//M.at<double>(0, 0) = 1;
		//M.at<double>(0, 1) = 2;
		//M.at<double>(0, 2) = 3;
		//M.at<double>(1, 0) = 4;
		//M.at<double>(1, 1) = 5;
		//M.at<double>(1, 2) = 6;
		//std::cout << M << std::endl;
		matchMat = std::to_string(M.at<double>(0, 0))+","+ std::to_string(M.at<double>(0, 1)) + "," + std::to_string(M.at<double>(0, 2))+"_"+
							std::to_string(M.at<double>(1, 0)) + "," + std::to_string(M.at<double>(1, 1)) + "," + std::to_string(M.at<double>(1, 2));
	}
	else {
		matchMat = "";
	}
	return matchMat;
}

int getMatchMsg(char* output_result_json, char * cloud_front_file, char * cloud_file, char* model_file, double cloud_length = CloudLength, int cloud_pos = CloudPos, float head_offset = HeadOffset, float mid_offset = MidOffset, float tail_offset = TailOffset) {
	std::vector<PointCloudT::Ptr> originGroupClouds;
	PointCloudT::Ptr cloud_real(new PointCloudT);
	float min_x;
	float max_x;
	double stepDistanceThresh = 300;
	double poleWidthThresh = 100;
	double poleSizeThresh = 200;
	double poleOffesetThresh = 20;
	//点云滤波
	int res = filterRealPointCloud(cloud_front_file, cloud_file, originGroupClouds, cloud_real, min_x, max_x, cloud_length, stepDistanceThresh, poleWidthThresh, poleSizeThresh, poleOffesetThresh);
	if (res == -1) {
		LOG(ERRORL) << "点云滤波错误";
		throw exception((std::string(__FUNCTION__) + ":点云滤波错误").c_str());
	}
	if (originGroupClouds.size() == 0) {
		LOG(ERRORL) << "点云为空错误";
		throw exception((std::string(__FUNCTION__) + ":点云为空错误").c_str());
	}
	//取拟合直线点
	vector<Point2f> line_points;
	vector<Eigen::Affine3f> tms;
	vector<PointCloudT::Ptr > clouds;
	vector<PointCloudT::Ptr > originGroupCloud_tmps;
	vector<string > pic_names;
	for (size_t i = 0; i < originGroupClouds.size(); i++)
	{
		Vec4f plane_line;
		Eigen::Affine3f tm = Eigen::Affine3f::Identity();
		string imagePath = savePointCloudCrossPic(originGroupClouds[i], min_x, max_x, plane_line, tm, i, 60, cloud_pos, head_offset, mid_offset, tail_offset);
		if (imagePath == "") {
			continue;
		}
		//一一对应
		originGroupCloud_tmps.push_back(originGroupClouds[i]);
		tms.push_back(tm);
		pic_names.push_back(imagePath);
		cvgeomerty::LINE LineSEG;
		searchLineSEG(imagePath, plane_line, LineSEG, tm, 10, 8, 5);
		Point2f p_2f = LineSEG.start;
		if (LineSEG.start.x > LineSEG.end.x) {
			p_2f = LineSEG.end;
		}
		PointCloudT::Ptr cloud(new PointCloudT);
		cloud->push_back(PointT(0, p_2f.y, p_2f.x));
		pcl::transformPointCloud(*cloud, *cloud, tm.inverse());
		clouds.push_back(cloud);
		Point2f p((*cloud)[0].z, (*cloud)[0].y);
		line_points.push_back(p);
#ifdef DEBUG
		{
			//(*cloud)[0].z = 0;
			boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = getViewer();
			viewerCloud(viewer, cloud_real, 255, 0, 0, 1, 0);
			viewerCloud(viewer, originGroupClouds[i], 0, 255, 0, 2, 2);
			viewerCloud(viewer, cloud, 0, 0, 255, 3, 10);
			viewerStop(viewer);

		}
#endif
	}
	//拟合直线
	Vec4f line;
	if (line_points.size() <= 0) {
		LOG(ERRORL) << "line_points为空错误";
		throw exception((std::string(__FUNCTION__) + ":line_points为空错误").c_str());
	}
	else if (line_points.size() == 1) {
		line[0] = line_points[0].x;
		line[1] = line_points[0].y + 1000;
		line[2] = line_points[0].x;
		line[3] = line_points[0].y - 1000;
	}
	else {
		fitLine(line_points, line, DIST_HUBER, 0, 1e-2, 1e-2);
	}
	int l_x1 = line[2];
	int l_y1 = line[3];
	int l_x2 = l_x1 + 1000 * line[0];
	int l_y2 = l_y1 + 1000 * line[1];
	l_x1 = l_x1 - 1000 * line[0];
	l_y1 = l_y1 - 1000 * line[1];
	if (line_points.size() == 1) {
		l_x1 = line[0];
		l_y1 = line[1];
		l_x2 = line[2];
		l_y2 = line[3];
	}
	cvgeomerty::LINE cv_line = cvgeomerty::LINE(Vec4f(l_x1, l_y1, l_x2, l_y2));
	cvgeomerty::LINE cv_line_show = cvgeomerty::LINE(Vec4f(l_x1, l_y1, l_x2, l_y2));
	//投影各个直线
	originGroupClouds.clear();
	originGroupClouds = originGroupCloud_tmps;
	vector<MatchMsg> match_msgs;
	for (size_t i = 0; i < originGroupClouds.size(); i++) {
		pcl::PointXYZ min_p, max_p;
		pcl::getMinMax3D(*originGroupClouds[i], min_p, max_p);
		float k = cv_line.k;
		float c = cv_line.c;
		if (k == 0) {
			LOG(ERRORL) << "k=0错误";
			throw exception((std::string(__FUNCTION__) + ":k=0错误").c_str());
		}
		if (isinf(k)) {
			float y1 = max_p.y;
			float x1 = cv_line.start.x;
			float y2 = min_p.y;
			float x2 = cv_line.end.x;
			cv_line = cvgeomerty::LINE(Vec4f(x1, y1, x2, y2));
		}
		else {
			float y1 = max_p.y;
			float x1 = (y1 - c) / k;
			float y2 = min_p.y;
			float x2 = (y2 - c) / k;
			cv_line = cvgeomerty::LINE(Vec4f(x1, y1, x2, y2));
		}
		PointCloudT::Ptr cloud(new PointCloudT);
		cloud->push_back(PointT(0, cv_line.start.y, cv_line.start.x));
		cloud->push_back(PointT(0, cv_line.end.y, cv_line.end.x));
		pcl::transformPointCloud(*cloud, *cloud, tms[i]);
		cvgeomerty::LINE line_match = cvgeomerty::LINE(Vec4f((*cloud)[0].z, (*cloud)[0].y, (*cloud)[1].z, (*cloud)[1].y));
		//计算旋转矩阵,伸缩值
		res = readModelLine(model_file, line);
		if (res == -1)
		{
			LOG(ERRORL) << "model line 文件不存在错误";
			throw exception((std::string(__FUNCTION__) + ":model line 文件不存在错误").c_str());
		}
		cvgeomerty::LINE line_model = cvgeomerty::LINE(line);

		if (line_model.start.y < line_model.end.y) {
			line_model = cvgeomerty::LINE(Vec4f(line_model.end.x, line_model.end.y, line_model.start.x, line_model.start.y));
		}
		if (line_match.start.y < line_match.end.y) {
			line_match = cvgeomerty::LINE(Vec4f(line_match.end.x, line_match.end.y, line_match.start.x, line_match.start.y));
		}

		double matchAngle = 0;
		Point stable_point_model(0, 0);
		Point stable_point_match(0, 0);
		if (!_isnanf(line_match.k)) {
			Eigen::Vector3f model_vec_line(line_model.start.x - line_model.end.x, line_model.start.y - line_model.end.y, 0);
			Eigen::Vector3f image_vec_line(line_match.start.x - line_match.end.x, line_match.start.y - line_match.end.y, 0);
			Eigen::Vector3f x_vec(1, 0, 0);
			double model_angel = getV1V2Angle(x_vec, model_vec_line)*180.0 / M_PI;
			double image_angel = getV1V2Angle(x_vec, image_vec_line)*180.0 / M_PI;
			Eigen::Vector3f mode_z_vec = x_vec.cross(model_vec_line);
			Eigen::Vector3f image_z_vec = x_vec.cross(image_vec_line);
			if (mode_z_vec.z() < 0) {
				model_angel = -model_angel;
			}
			if (image_z_vec.z() < 0) {
				image_angel = -image_angel;
			}
			matchAngle = image_angel - model_angel;
			Point model_mid_point = cvgeomerty::getMidPoint(line_model.start, line_model.end);
			Point image_mid_point = cvgeomerty::getMidPoint(line_match.start, line_match.end);

			stable_point_model = Point((int)model_mid_point.x, (int)model_mid_point.y);
			stable_point_match = Point((int)image_mid_point.x, (int)image_mid_point.y);
		}
		Mat M = getRotationMatrix2D(stable_point_model, -matchAngle, 1);
		M.at<double>(0, 2) = M.at<double>(0, 2) + stable_point_match.x - stable_point_model.x;
		M.at<double>(1, 2) = M.at<double>(1, 2) + stable_point_match.y - stable_point_model.y;


		double distance_model = cvgeomerty::distance(line_model.start, line_model.end);
		double distance_match = cvgeomerty::distance(line_match.start, line_match.end);
		double scale = distance_match / distance_model;
		std::cout << "scale:" << scale << std::endl;
		std::cout << "M:" << M << std::endl;
		std::cout << "matchAngle:" << matchAngle << std::endl;
		string match_mat = getMatchMat(M);
		match_msgs.push_back(MatchMsg(pic_names[i], scale, match_mat, matchAngle));
#ifdef DEBUG		
		//std::cout<<"cv_line.k:" << cv_line.k << std::endl;
		//PointCloudT::Ptr cloud_line(new PointCloudT);
		//cloud_line->push_back(PointT(0, cv_line.start.y, cv_line.start.x));
		//cloud_line->push_back(PointT(0, cv_line.end.y, cv_line.end.x));
		//{
		//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = getViewer();
		//	viewerCloud(viewer, cloud_real, 255, 0, 0, 1, 0);
		//	viewerCloud(viewer, originGroupClouds[i], 0, 255, 0, 2, 2);
		//	viewerCloud(viewer, cloud_line, 255, 0, 255, 4, 10);
		//	for(int j=0;j< clouds.size();j++)
		//	viewerCloud(viewer, clouds[j], 255, 255, 0, j+8, 10);
		//	viewer->addLine<pcl::PointXYZ>(PointT(0, cv_line_show.start.y, cv_line_show.start.x), PointT(0, cv_line_show.end.y, cv_line_show.end.x), 255, 0, 0,"5");
		//	viewerStop(viewer);
		//}

		//模板直线旋转
		Mat p1(2, 1, CV_32FC1);
		p1.at<float>(0, 0) = line_model.start.x;
		p1.at<float>(1, 0) = line_model.start.y;
		Mat p2(2, 1, CV_32FC1);
		p2.at<float>(0, 0) = line_model.end.x;
		p2.at<float>(1, 0) = line_model.end.y;
		Mat M_r(2, 2, CV_32FC1);
		M_r.at<float>(0, 0) = M.at<double>(0, 0);
		M_r.at<float>(0, 1) = M.at<double>(0, 1);
		M_r.at<float>(1, 0) = M.at<double>(1, 0);
		M_r.at<float>(1, 1) = M.at<double>(1, 1);
		std::cout << "M_r:" << M_r << std::endl;
		Mat p11 = M_r * p1;
		Mat p22 = M_r * p2;
		//模板直线平移
		std::cout << "M.at<double>(0, 2):" << M.at<double>(0, 2) << std::endl;
		std::cout << "M.at<double>(1, 2):" << M.at<double>(1, 2) << std::endl;
		p11.at<float>(0, 0) = p11.at<float>(0, 0) + M.at<double>(0, 2);
		p11.at<float>(1, 0) = p11.at<float>(1, 0) + M.at<double>(1, 2);
		p22.at<float>(0, 0) = p22.at<float>(0, 0) + M.at<double>(0, 2);
		p22.at<float>(1, 0) = p22.at<float>(1, 0) + M.at<double>(1, 2);
		//模板直线距离计算
		double distance_model_r = cvgeomerty::distance(Point2f(p11.at<float>(0, 0), p11.at<float>(1, 0)), Point2f(p22.at<float>(0, 0), p22.at<float>(1, 0)));
		std::cout << "distance_model:" << distance_model << std::endl;
		std::cout << "distance_match:" << distance_match << std::endl;
		std::cout << "distance_model_r:" << distance_model_r << std::endl;
		cvgeomerty::LINE line_model_r = cvgeomerty::LINE(Vec4f(p11.at<float>(0, 0), p11.at<float>(1, 0), p22.at<float>(0, 0), p22.at<float>(1, 0)));
		//喷涂点
		Point2f paint1 = line_model.start;
		Point2f paint2 = line_model.end;
		//伸缩喷涂点
		Point2f paint3 = line_model.end;
		Point2f paint4 = line_model.end;
		double distance_paint3 = scale * cvgeomerty::distance(paint1, line_model.end);
		double distance_paint4 = scale * cvgeomerty::distance(paint2, line_model.end);
		std::cout << "*****distance_paint3:" << distance_paint3 << std::endl;
		std::cout << "*****distance_paint4:" << distance_paint4 << std::endl;
		float angle = line_model.angle / 180.0*M_PI;
		paint3.x += distance_paint3 * cos(angle);
		paint3.y += distance_paint3 * sin(angle);
		paint4.x += distance_paint4 * cos(angle);
		paint4.y += distance_paint4 * sin(angle);
		std::cout << "**paint3:" << paint3 << std::endl;
		std::cout << "**paint4:" << paint4 << std::endl;
		std::cout << "stable_point_model.y:" << stable_point_model.y << std::endl;
		std::cout << " (paint3.y + paint4.y) / 2.0:****" << (paint3.y + paint4.y) / 2.0 << std::endl;
		float transx = stable_point_model.x - (paint3.x + paint4.x) / 2.0;
		float transy = stable_point_model.y - (paint3.y + paint4.y) / 2.0;
		paint3.x += transx;
		paint4.x += transx;
		paint3.y += transy;
		paint4.y += transy;
		std::cout << "transx:" << transx << std::endl;
		std::cout << "transy:" << transy << std::endl;
		std::cout << "paint3:" << paint3 << std::endl;
		std::cout << "paint4:" << paint4 << std::endl;
		//旋转平移喷涂点
		Mat p3(2, 1, CV_32FC1);
		p3.at<float>(0, 0) = paint3.x;
		p3.at<float>(1, 0) = paint3.y;
		Mat p4(2, 1, CV_32FC1);
		p4.at<float>(0, 0) = paint4.x;
		p4.at<float>(1, 0) = paint4.y;
		Mat p33 = M_r * p3;
		Mat p44 = M_r * p4;
		p33.at<float>(0, 0) = p33.at<float>(0, 0) + M.at<double>(0, 2);
		p33.at<float>(1, 0) = p33.at<float>(1, 0) + M.at<double>(1, 2);
		p44.at<float>(0, 0) = p44.at<float>(0, 0) + M.at<double>(0, 2);
		p44.at<float>(1, 0) = p44.at<float>(1, 0) + M.at<double>(1, 2);
		paint3.x = p33.at<float>(0, 0);
		paint3.y = p33.at<float>(1, 0);
		paint4.x = p44.at<float>(0, 0);
		paint4.y = p44.at<float>(1, 0);

		Vec4f plane_line;
		Eigen::Affine3f tm;
		string imagePath = savePointCloudCrossPic(originGroupClouds[i], min_x, max_x, plane_line, tm, i, LeftNoRPThresh, CloudPos, HeadOffset, MidOffset, TailOffset);
		if (imagePath == "") {
			continue;
		}
		cv::Mat src2 = cv::imread(imagePath, cv::IMREAD_COLOR);
		cv::line(src2, line_match.start, line_match.end, Scalar(rand() % 255, rand() % 255, rand() % 255), 2, LINE_AA);
		cv::line(src2, line_model.start, line_model.end, Scalar(rand() % 255, rand() % 255, rand() % 255), 2, LINE_AA);
		cv::line(src2, line_model_r.start, line_model_r.end, Scalar(rand() % 255, rand() % 255, rand() % 255), 2, LINE_AA);
		circle(src2, paint1, 5, Scalar(0, 0, 255), 2, 8, 0);
		circle(src2, paint2, 5, Scalar(0, 0, 255), 2, 8, 0);
		circle(src2, paint3, 5, Scalar(0, 255, 0), 2, 8, 0);
		circle(src2, paint4, 5, Scalar(0, 255, 0), 2, 8, 0);
		double arrowWidth = 30;
		arrowedLine(src2, paint1, Point2f(paint1.x - arrowWidth, paint1.y - (arrowWidth)* tan((line_model.angle - 90) / 180 * CV_PI)), Scalar(0, 0, 255), 2, LINE_AA, 0, 0.3);
		arrowedLine(src2, paint2, Point2f(paint2.x - arrowWidth, paint2.y - (arrowWidth)* tan((line_model.angle - 90) / 180 * CV_PI)), Scalar(0, 0, 255), 2, LINE_AA, 0, 0.3);
		arrowedLine(src2, paint3, Point2f(paint3.x - arrowWidth, paint3.y - (arrowWidth)* tan((line_match.angle - 90) / 180 * CV_PI)), Scalar(0, 0, 255), 2, LINE_AA, 0, 0.3);
		arrowedLine(src2, paint4, Point2f(paint4.x - arrowWidth, paint4.y - (arrowWidth)* tan((line_match.angle - 90) / 180 * CV_PI)), Scalar(0, 0, 255), 2, LINE_AA, 0, 0.3);
		imshow("line", src2);
		waitKey(0);
#endif			
	}
	if (match_msgs.size() <= 0) {
		LOG(ERRORL) << "match_msgs.size() <= 0错误";
		throw exception((std::string(__FUNCTION__) + ":match_msgs.size() <= 0错误").c_str());
	}
	string result_json = createMsgsToJSON(match_msgs);
	std::cout <<"result_json:"<< result_json << std::endl;
	std::memcpy(output_result_json, result_json.c_str(), result_json.length());
	return result_json.length() < 2 ? -1 : result_json.length();
}
/**/
int main()
{
	while (1) {
		std::string mode;
		std::cout << "intput mode:" << std::endl;
		std::cout << "0.filter:" << std::endl;
		std::cin >> mode;
		if (mode == "0") {
			std::string cloud_file_str;
			std::cout << "intput cloud_file_str:" << std::endl;
			std::cin >> cloud_file_str;
			char* cloud_file = (char*)cloud_file_str.data();
			char * model_file = "E:\\project\\AluminumProfileSpraying\\RangeImage\\build\\Release\\aluminum\\trainModel\\model.key";
			char* json_result = new char[10000];
			getMatchMsg(json_result,cloud_file, cloud_file, model_file);

		}


	}
	int i;
	cin >> i;
	return 0;
}

