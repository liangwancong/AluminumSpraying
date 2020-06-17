// RangeImage.cpp: 定义控制台应用程序的入口点。
//
#include "pcl_filter_cloud.h"
#include "cvgeomerty.h"
#include "string_util.h"
#include "file_util.h"

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
#include "Registration.h"
#ifdef DEBUG
#include <pcl/filters/passthrough.h>
#include "Viewer.h"
#endif
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include "pcl_filter_helper.h"
#include "pclgeomerty.h"
#include "file_util.h"
#include "paint_model.h"
#include "json_model_parse.h"

#include "cv_search_points.h"
#include "cv_match_points.h"
#include "cvskeleton.h"
#include "parameters.h"
#include <direct.h>
#include "global.h"
#include "logger.h"
#include "CRMath.h"
#include "Mvbb.h"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

using namespace std;
using namespace cv;
using namespace stringutil;
using namespace json2model;
using namespace cvskeleton;

using namespace CVSearchPoints;
using namespace CVMatchPoints;
using namespace fileutil;
using namespace pcl_filter_cloud;
using namespace AluminumParameters;
using namespace AluminumGlobal;
using namespace logger;
using namespace CRMath;
using namespace G3CppMvbb;

int readPlaneLine(char* file, Vec4f &plane_line) {
	int error = fileExist(file);
	if (error == -1) {
		std::cout << "load plane_line error" << std::endl;
		return -1;
	}
	FILE* stream;
	stream = fopen(file, "r");
	if (stream == NULL) {
		std::cout << "load Tm error" << std::endl;
		return -1;
	}
	fscanf(stream, "%f %f %f %f  ", &plane_line[0], &plane_line[1], &plane_line[2], &plane_line[3]);
	fclose(stream);
	return 0;
}
int writePlaneLine(char* file, Vec4f &plane_line) {
	FILE* stream;
	stream = fopen(file, "w");
	fprintf(stream, "%.3f%s%.3f%s%.3f%s%.3f%s", plane_line[0], " ", plane_line[1], " ", plane_line[2], " ", plane_line[3], " ");
	fclose(stream);
	return 0;
}
void img2PointCloud(cv::Mat& img, PointCloudT::Ptr &cloud) {
	cv::Mat img_binary;
	cv::threshold(img, img_binary, 128, 1, cv::THRESH_BINARY);
	for (int row = 0; row < img_binary.rows; row++) {
		for (int col = 0; col < img_binary.cols; col++) {
			uchar c = img_binary.at<uchar>(row, col);
			if (c != 0) {
				PointT p;
				p.x = col;
				p.y = row;
				p.z = 0;
				cloud->push_back(p);
			}
		}
	}
}
int parseMatchKey(const std::string &matchName, MatchModel &maxMatches) {
	int status = 0;

	if (matchName.length()>1) {
		std::string jsonString = "";
		try
		{
			ifstream infile(saveModelPath + matchName, ios::binary);
			char *buffer = new char[99999];
			while (!infile.eof())
			{
				infile >> buffer;

				jsonString = jsonString + std::string(buffer);

			}
			infile.close();
			delete[] buffer;
		}
		catch (const std::exception&)
		{

		}

		status = parsingJSONToKeyPoints(jsonString.c_str(), maxMatches);
		if (status == 0) {
			return -1;
		}
	}
	return 0;
}
int getMatchMat(const Point &model_stable, const Point &src_stable,Mat &affineMat, string &matchMat) {
	if (affineMat.cols > 0 && affineMat.rows > 0)
	{
		for (size_t i = 0; i < affineMat.rows; i++)
		{
			vector<double> b;
			for (size_t j = 0; j < affineMat.cols; j++)
			{
				double s = affineMat.at<double>(i, j);

				if (i == 0 && j == 2)
				{
					if ((src_stable.x == 0 && src_stable.y == 0) || (model_stable.x == 0 && model_stable.y == 0))
					{
						matchMat = matchMat + "," + to_string(s);
					}
					else {
						cout << src_stable << endl;
						matchMat = matchMat + "," + to_string(s + src_stable.x - model_stable.x);
					}
				}
				else if (i == 1 && j == 2) {
					if ((src_stable.x == 0 && src_stable.y == 0) || (model_stable.x == 0 && model_stable.y == 0))
					{
						matchMat = matchMat + "," + to_string(s);
					}
					else {
						matchMat = matchMat + "," + to_string(s + src_stable.y - model_stable.y);
					}
				}
				else {
					matchMat = matchMat + (j > 0 ? "," : "") + to_string(s);
				}
			}
			if (i != affineMat.rows - 1)
			{
				matchMat = matchMat + "_";
			}
		}
	}
	else {
		return -1;
	}
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
int findMidMode(std::vector<PointCloudT::Ptr> &originGroupClouds,float &min_x, float &max_x, double &matchVal,string &matchName, bool &no_plane, int &midIndex, vector<PaintPoint> &corners,double minAngleThresh, double minIntervalThresh, double minDistanceThresh, double planeWidthThresh, double novangle , double minPointDistanceThresh , double leftNoRPThresh , bool skeletonImage, int cloud_pos, float head_offset, float mid_offset, float tail_offset) {
	matchName = "";
	string matchMat = "";
	double matchAngle = 0;
	//double matchVal = 0;
	no_plane = false;
	Point stable_point;
	double picwidth_thresh = 30;

	midIndex = floor(originGroupClouds.size()*1.0 / 2.0);
	int midIndex_tmp = floor(originGroupClouds.size()*1.0 / 2.0);
	bool plus_flag = false;
	bool plus_stop = false;
	bool sub_stop = false;
	int index_count = 1;
	vector<int> indexs;
	indexs.push_back(midIndex);
startReset:
	if (midIndex >= originGroupClouds.size())
	{
		LOG(ERRORL) << "midIndex错误";
		throw exception((std::string(__FUNCTION__) + ":midIndex错误").c_str());
		return -1;
	}
	string imagePath = savePointCloudCrossPic(originGroupClouds[midIndex], min_x, max_x, midIndex, half_project_length, cloud_pos, head_offset, mid_offset, tail_offset);
	std::cout << "***********************************************************************************************************************************************midIndex**"<< midIndex << std::endl;
	if (imagePath.length() < 2)
	{
		LOG(ERRORL) << "保存图片错误";
		throw exception((std::string(__FUNCTION__) + ":保存图片错误").c_str());
		return -1;
	}
	corners = matchKeyPointToCorners(imagePath, matchName, matchVal, matchAngle, matchMat, no_plane, &stable_point, minAngleThresh, minIntervalThresh, minDistanceThresh, planeWidthThresh, novangle, minPointDistanceThresh, leftNoRPThresh, skeletonImage);
	//std::tuple<double, cv::Point> t = get2DPCATransfromTuple(imagePath);
	//double img_angle = get<0>(t);
	//Point ing_stable_point = get<1>(t);
	if ((stable_point.x < picwidth_thresh || stable_point.y < picwidth_thresh || stable_point.x > PICWIDTH - picwidth_thresh || stable_point.y > PICWIDTH - picwidth_thresh) || (matchName == ""))
	{
	//find_index:
	//	if (plus_flag == true && (!plus_stop)) {
	//		plus_flag = false;
	//		midIndex = midIndex_tmp + index_count;
	//	}
	//	else if (plus_flag == false && (!sub_stop)) {
	//		plus_flag = true;
	//		midIndex = midIndex_tmp - index_count;
	//		index_count++;

	//	}
	//	if (midIndex > originGroupClouds.size() - 1) {
	//		midIndex = originGroupClouds.size() - 1;
	//		plus_stop = true;
	//	}
	//	else if (midIndex < 0) {
	//		sub_stop = true;
	//		midIndex = 0;
	//	}

	//	if (((plus_flag == true && sub_stop == true) || (plus_flag == false && sub_stop == true)) && (plus_stop != true && sub_stop != true)) {
	//		goto find_index;
	//	}
	//	if (plus_stop != true && sub_stop != true) {
	//		matchName = "";
	//		goto startReset;
	//	}

		
		if (plus_flag == false) {
			plus_flag = true;
			midIndex = midIndex_tmp - index_count;
		}else if (plus_flag == true) {
			plus_flag = false;
			midIndex = midIndex_tmp + index_count;
			index_count++;
		}
		if (midIndex > originGroupClouds.size() - 1) {
			midIndex = originGroupClouds.size() - 1;
			plus_stop = true;
		}
		else if (midIndex < 0) {
			sub_stop = true;
			midIndex = 0;
		}
		vector<int>::iterator result = find(indexs.begin(), indexs.end(), midIndex); //查找midIndex
		if (result == indexs.end()) //没找到
		{
			indexs.push_back(midIndex);
		//	if (plus_stop != true && sub_stop != true)
				goto startReset;
		}
	}
	if (matchName.length() < 1) {
		midIndex = floor(originGroupClouds.size()*1.0 / 2.0);
		string imagePath = savePointCloudCrossPic(originGroupClouds[midIndex], min_x, max_x, midIndex, half_project_length, cloud_pos, head_offset, mid_offset, tail_offset);
		corners = matchKeyPointToCorners(imagePath, matchName, matchVal, matchAngle, matchMat, no_plane, &stable_point, minAngleThresh, minIntervalThresh, minDistanceThresh, planeWidthThresh, novangle, minPointDistanceThresh, leftNoRPThresh, skeletonImage);
	}
	return 0;
}
int miniWorkpiece(std::vector<PointCloudT::Ptr> &originGroupClouds, PointCloudT::Ptr &cloudReal, float &min_x, float &max_x, string model_name, vector<PaintPoints> &groupPaintPoints, double planeWidthThresh, int cloud_pos, float head_offset, float mid_offset, float tail_offset) {
	int midIndex = floor(originGroupClouds.size()*1.0 / 2.0);
	string imagePath = savePointCloudCrossPic(originGroupClouds[midIndex], min_x, max_x, midIndex, half_project_length, cloud_pos, head_offset, mid_offset, tail_offset);
	if (WorkFlowPath != "") {
		//保存小工件图片
		Mat src = cv::imread(imagePath, cv::IMREAD_COLOR);
		std::string scan_path = WorkFlowPath.substr(0, WorkFlowPath.length() - 10) + "image.jpeg";
		cv::imwrite(scan_path, src);
		//保存小工件plane
		Vec4f plane_line(0, 0, 0, 0);
		string r_plane_path = imagePath.substr(0, imagePath.length() - 3) + "key";
		writePlaneLine((char*)r_plane_path.c_str(), plane_line);
		string w_plane_path = WorkFlowPath.substr(0, WorkFlowPath.length() - 10) + "image.key";
		writePlaneLine((char*)w_plane_path.c_str(), plane_line);
	}
	vector<PaintPoint> corners;
	Point2f center = Point2f(PICWIDTH_2, PICWIDTH_2);
	corners.push_back(PaintPoint(center, 0, false));

#ifdef DEBUGC
	{
		Mat src = cv::imread(imagePath, cv::IMREAD_COLOR);
		circle(src, center, 5, Scalar(0, 255, 0), 2, 8, 0);
		double arrowWidth = 30;
		arrowedLine(src, center, Point2f(center.x - arrowWidth, center.y), Scalar(0, 0, 255), 2, LINE_AA, 0, 0.3);
		if (WorkFlowPath != "") {
			string file_name = getFileName(imagePath);
			imwrite(WorkFlowPath + file_name.substr(0, file_name.length() - 4) + "_mini.jpg", src);
		}
	}
#endif
	double matchAngle = 0;
	Point stable_point(Point(0,0));//小工件不存在稳定点
	bool no_plane = true;
	double matchVal = 0;

	PaintPoints paintPoints = PaintPoints();
	paintPoints.points = corners;
	paintPoints.picName = imagePath;
	paintPoints.havePlane = !no_plane;
	paintPoints.matchAngle = matchAngle;
	paintPoints.matchMat = "";
	paintPoints.matchName = "";
	paintPoints.stablePoint = stable_point;
	paintPoints.matchVal = matchVal;
	paintPoints.maxLineStart = cv::Point2f(0, 0);
	paintPoints.maxLineEnd = cv::Point2f(0, 0);
	paintPoints.maxLineContentPoints = std::vector<PaintPoint>(0);
	groupPaintPoints.push_back(paintPoints);
	return 0;
}

double  getV1V2Angle1(const Eigen::Vector3f &V1, const Eigen::Vector3f &V2) {
	double tem = V1.dot(V2);
	double tep = sqrt(V1.dot(V1) * V2.dot(V2));
	double angle = acos(tem / tep);
	if (isnan(angle))
	{
		std::cout << "angle error" << std::endl;
		return DBL_MAX;
	}
	return angle;
};
int getV12V2Tm1(const Eigen::Vector3f &v1, const Eigen::Vector3f &v2, Eigen::Matrix4f &tm) {
	//v1面-->v2面
	//两个平面法向量的夹角
	double angle = getV1V2Angle1(v1, v2);
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
int useFixedModel(std::vector<PointCloudT::Ptr> &originGroupClouds, float &min_x, float &max_x, string model_name,vector<PaintPoints> &groupPaintPoints, double minAngleThresh , double minIntervalThresh , double minDistanceThresh , double planeWidthThresh , double novangle, double minPointDistanceThresh, double leftNoRPThresh, bool skeletonImage, int cloud_pos, float head_offset, float mid_offset, float tail_offset) {
	//加载模板图片
	string modelPath = saveModelImgPath + model_name + ".jpg";
	std::cout <<"modelPath:"<< modelPath << std::endl;
	int res = fileExist((char*)modelPath.c_str());
	if (res == -1) { 
		LOG(ERRORL) << "加载图片错误";
		throw exception((std::string(__FUNCTION__)+":加载图片错误").c_str());
	}
	for (size_t i = 0; i < originGroupClouds.size(); i++)
	{
		string imagePath = savePointCloudCrossPic(originGroupClouds[i], min_x, max_x, i, leftNoRPThresh, cloud_pos, head_offset, mid_offset, tail_offset);

		if (i == floor(originGroupClouds.size()*1.0 / 2.0)) {
			//保存中间工件图片
			Mat src = cv::imread(imagePath, cv::IMREAD_COLOR);
			std::string scan_path = WorkFlowPath.substr(0, WorkFlowPath.length() - 10) + "image.jpeg";
			cv::imwrite(scan_path, src);
			//保存小工件plane
			Vec4f plane_line(0, 0, 0, 0);
			string r_plane_path = imagePath.substr(0, imagePath.length() - 3) + "key";
			res = readPlaneLine((char*)r_plane_path.c_str(), plane_line);
			if (res == -1) {
				LOG(ERRORL) << "readPlaneLine错误";
				throw exception((std::string(__FUNCTION__) + ":readPlaneLine错误").c_str());
			}
			string w_plane_path = WorkFlowPath.substr(0, WorkFlowPath.length() - 10) + "image.key";
			writePlaneLine((char*)w_plane_path.c_str(), plane_line);
		}
		
		if (imagePath.length() < 2)
		{
			continue;
		}
		bool load_line = true;
		//加载model直线
		string model_plane_path = saveModelPath + model_name + ".key";
		Vec4f model_plane;
		res = readPlaneLine((char*)model_plane_path.c_str(), model_plane);
		if (res == -1) {
			load_line = false;
		}
		//加载image直线
		string image_plane_path = imagePath.substr(0, imagePath.length() - 3) + "key";
		Vec4f image_plane;
		res = readPlaneLine((char*)image_plane_path.c_str(), image_plane);
		if (res == -1) {
			load_line = false;
		}
		cvgeomerty::LINE model_line(Vec4f(0,0,0,0));
		cvgeomerty::LINE image_line(Vec4f(0, 0, 0, 0));
		double matchAngle = 0;
		Point model_stable_point(0, 0);
		Point image_stable_point(0, 0);
		if (load_line == true) {
			model_line = cvgeomerty::LINE(model_plane);
			image_line = cvgeomerty::LINE(image_plane);
			if (!_isnanf(model_line.k) && !_isnanf(image_line.k)) {
				Eigen::Vector3f model_vec_line(model_line.start.x - model_line.end.x, model_line.start.y - model_line.end.y, 0);
				Eigen::Vector3f image_vec_line(image_line.start.x - image_line.end.x, image_line.start.y - image_line.end.y, 0);
				Eigen::Vector3f x_vec(1,0,0);
				double model_angel = getV1V2Angle1(x_vec, model_vec_line)*180.0/M_PI;
				double image_angel = getV1V2Angle1(x_vec, image_vec_line)*180.0 / M_PI;
				Eigen::Vector3f mode_z_vec = x_vec.cross(model_vec_line);
				Eigen::Vector3f image_z_vec = x_vec.cross(image_vec_line);
				if (mode_z_vec.z() < 0) {
					model_angel = -model_angel;
				}
				if (image_z_vec.z()<0) {
					image_angel = -image_angel;
				}
				matchAngle = image_angel - model_angel;
				Point model_mid_point = cvgeomerty::getMidPoint(model_line.start, model_line.end);
				Point image_mid_point = cvgeomerty::getMidPoint(image_line.start, image_line.end);

				model_stable_point = Point((int)model_mid_point.x, (int)model_mid_point.y);
				image_stable_point = Point((int)image_mid_point.x, (int)image_mid_point.y);
			}
		}
		Mat model = cv::imread(modelPath, cv::IMREAD_GRAYSCALE);
		Mat image = cv::imread(imagePath, cv::IMREAD_GRAYSCALE);
		PointCloudT::Ptr cloud_model(new PointCloudT);
		PointCloudT::Ptr cloud_image(new PointCloudT);
		PointCloudT::Ptr cloud_model_line(new PointCloudT);
		PointCloudT::Ptr cloud_model_icp(new PointCloudT);
		img2PointCloud(model, cloud_model);
		img2PointCloud(image, cloud_image);
		*cloud_model_line = *cloud_model;
		*cloud_model_icp = *cloud_model;

		Eigen::Matrix4f tm_line = Eigen::Matrix4f::Identity();

		Mat M_line = getRotationMatrix2D(model_stable_point, -matchAngle, 1);
		M_line.at<double>(0, 2, 0) = M_line.at<double>(0, 2, 0) + image_stable_point.x - model_stable_point.x;
		M_line.at<double>(1, 2, 0) = M_line.at<double>(1, 2, 0) + image_stable_point.y - model_stable_point.y;
		tm_line(0, 0) = M_line.at<double>(0, 0);
		tm_line(0, 1) = M_line.at<double>(0, 1);
		tm_line(0, 3) = M_line.at<double>(0, 2);
		tm_line(1, 0) = M_line.at<double>(1, 0);
		tm_line(1, 1) = M_line.at<double>(1, 1);
		tm_line(1, 3) = M_line.at<double>(1, 2);
		pcl::transformPointCloud(*cloud_model_line, *cloud_model_line, tm_line);


		double score = 0;
		Eigen::Matrix4f tm_icp = Eigen::Matrix4f::Identity();
		icpRegistration(cloud_model, cloud_image, score, tm_icp);
		pcl::transformPointCloud(*cloud_model_icp, *cloud_model_icp, tm_icp);
		Mat M_icp = getRotationMatrix2D(Point2f(0, 0), 0, 1);
		M_icp.at<double>(0, 0) = tm_icp(0, 0);
		M_icp.at<double>(0, 1) = tm_icp(0, 1);
		M_icp.at<double>(0, 2) = tm_icp(0, 3);
		M_icp.at<double>(1, 0) = tm_icp(1, 0);
		M_icp.at<double>(1, 1) = tm_icp(1, 1);
		M_icp.at<double>(1, 2) = tm_icp(1, 3);

		double score_line = getIcpScore(cloud_model_line, cloud_image);
		double score_icp = getIcpScore(cloud_model_icp, cloud_image);

		Eigen::Matrix4d tm_lined = Eigen::Matrix4d::Identity();
		Eigen::Matrix4d tm_icpd = Eigen::Matrix4d::Identity();
		tm_lined(0, 0) = M_line.at<double>(0, 0);
		tm_lined(0, 1) = M_line.at<double>(0, 1);
		tm_lined(0, 3) = M_line.at<double>(0, 2);
		tm_lined(1, 0) = M_line.at<double>(1, 0);
		tm_lined(1, 1) = M_line.at<double>(1, 1);
		tm_lined(1, 3) = M_line.at<double>(1, 2);

		tm_icpd(0, 0) = M_icp.at<double>(0, 0);
		tm_icpd(0, 1) = M_icp.at<double>(0, 1);
		tm_icpd(0, 3) = M_icp.at<double>(0, 2);
		tm_icpd(1, 0) = M_icp.at<double>(1, 0);
		tm_icpd(1, 1) = M_icp.at<double>(1, 1);
		tm_icpd(1, 3) = M_icp.at<double>(1, 2);
		Vector6d  line_rz = Matrix4ToEuler(tm_lined);
		Vector6d  icp_rz = Matrix4ToEuler(tm_icpd);

		std::cout << "matchAngle:" << matchAngle << std::endl;
		std::cout << "line_rz:" << line_rz(0,5) << std::endl;
		std::cout << "icp_rz:" << icp_rz(0, 5) << std::endl;
		std::cout << "score_line:" << score_line << std::endl;
		std::cout << "score_icp:" << score_icp << std::endl;
		Mat M;
		M = M_icp;
		matchAngle = icp_rz(0, 5);
		if (score_line < score_icp) {
			M = M_line;
			matchAngle = line_rz(0, 5);
			std::cout << "Line" << std::endl;
		}
		if (matchAngle < -90)
		{
			matchAngle = matchAngle + 180;
			AngleAxisd v(matchAngle*M_PI/180.0, Eigen::Vector3d(0,0,1));
			Matrix3d roatation = Matrix3d::Identity();
			roatation = v.toRotationMatrix();
			M.at<double>(0, 0) = roatation(0, 0);
			M.at<double>(0, 1) = roatation(0, 1);
			M.at<double>(1, 0) = roatation(1, 0);
			M.at<double>(1, 1) = roatation(1, 1);
			std::cout <<"roatation:"<< roatation << std::endl;
		}
		else if (matchAngle > 90)
		{
			matchAngle = matchAngle - 180;
			AngleAxisd v(matchAngle*M_PI / 180.0, Eigen::Vector3d(0, 0, 1));
			Matrix3d roatation = Matrix3d::Identity();
			roatation = v.toRotationMatrix();
			M.at<double>(0, 0) = roatation(0, 0);
			M.at<double>(0, 1) = roatation(0, 1);
			M.at<double>(1, 0) = roatation(1, 0);
			M.at<double>(1, 1) = roatation(1, 1);
			std::cout << "roatation:" << roatation << std::endl;
		}
		//std::cout << M << std::endl;
		//AngleAxisd v(matchAngle*M_PI / 180.0, Eigen::Vector3d(0, 0, 1));
		//Matrix3d roatation = Matrix3d::Identity();
		//roatation = v.toRotationMatrix();
		//M.at<double>(0, 0) = roatation(0, 0);
		//M.at<double>(0, 1) = roatation(0, 1);
		//M.at<double>(1, 0) = roatation(1, 0);
		//M.at<double>(1, 1) = roatation(1, 1);
		//std::cout << "roatation:" << roatation << std::endl;
#ifdef DEBUG
		{
			std::cout << M << std::endl;
			Mat model = cv::imread(modelPath, cv::IMREAD_COLOR);
			Mat image = cv::imread(imagePath, cv::IMREAD_COLOR);
			line(image, model_line.start, model_line.end, Scalar(255, 0, 0), 2, LINE_AA);
			line(image, image_line.start, image_line.end, Scalar(0, 255, 0), 2, LINE_AA);
			model_line.start.x -= 20;
			circle(model, model_line.start, 2, Scalar(255, 0, 0), 5, 8, 0);
			//imshow("model", model);
			warpAffine(model, model, M, model.size());
			imshow("model_R", model);
			imshow("image", image);
			waitKey();

			boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = getViewer();
			viewerCloud(viewer, cloud_model, 255, 255, 0, 4, 1);
			viewerCloud(viewer, cloud_model_line, 255, 0, 0, 1, 1);
			viewerCloud(viewer, cloud_model_icp, 0, 0, 255, 2, 1);
			viewerCloud(viewer, cloud_image, 0, 255, 0, 3, 0);
			viewerStop(viewer);
		}
#endif 

#ifdef DEBUG
		{
			Mat model = cv::imread(modelPath, cv::IMREAD_COLOR);
			Mat image = cv::imread(imagePath, cv::IMREAD_COLOR);
			line(model, model_line.start, model_line.end, Scalar(255, 0, 0), 2, LINE_AA);
			line(image, image_line.start, image_line.end, Scalar(0, 255, 0), 2, LINE_AA);
			imshow("model", model);
			imshow("image", image);
			warpAffine(model, model, M, model.size());
			line(model, image_line.start, image_line.end, Scalar(0, 0, 255), 2, LINE_AA);
			imshow("model2image", model);
			waitKey();
		}
#endif 

		
		vector<PaintPoint> corners;
		Point2f center = Point2f(PICWIDTH_2, PICWIDTH_2);
		corners.push_back(PaintPoint(center, 0, false));
		string matchName = model_name+".key";
		bool no_plane = false;
		double matchVal = 0;
		PaintPoints paintPoints = PaintPoints();
		paintPoints.points = corners;
		paintPoints.picName = imagePath;
		paintPoints.havePlane = !no_plane;
		paintPoints.matchAngle = matchAngle;
		paintPoints.matchMat = getMatchMat(M);
		paintPoints.matchName = matchName;
		paintPoints.stablePoint = model_stable_point;
		paintPoints.matchVal = matchVal;
		paintPoints.maxLineStart = image_line.start;
		paintPoints.maxLineEnd = image_line.end;
		paintPoints.maxLineContentPoints = std::vector<PaintPoint>(0);
		groupPaintPoints.push_back(paintPoints);
	}
	return 0;
}
int useMidModel(std::vector<PointCloudT::Ptr> &originGroupClouds,float &min_x, float &max_x, vector<PaintPoints> &groupPaintPoints,double minAngleThresh , double minIntervalThresh , double minDistanceThresh , double planeWidthThresh , double novangle, double minPointDistanceThresh, double leftNoRPThresh , bool skeletonImage, int cloud_pos, float head_offset, float mid_offset, float tail_offset) {
	vector<PaintPoint> corners;
	double matchVal = 0;
	string matchName = "";
	int midIndex = 0;
	bool no_plane = false;
	//获取最适模板
	//得到matchName,midIndex,corners
	int res = findMidMode(originGroupClouds,min_x, max_x, matchVal,matchName, no_plane,midIndex, corners,minAngleThresh, minIntervalThresh,  minDistanceThresh, planeWidthThresh, novangle,  minPointDistanceThresh,  leftNoRPThresh ,skeletonImage, cloud_pos, head_offset, mid_offset, tail_offset);
	if (res == -1) {
		LOG(ERRORL) << "findMidMode error";
		throw exception((std::string(__FUNCTION__) + ":findMidMode").c_str());
		return -1;
	}
	if (corners.size() == 0) {
		LOG(ERRORL) << "corners.size=0";
		throw exception((std::string(__FUNCTION__) + ":corners.size=0").c_str());
		return -1;
	}
	//解析匹配模板
	MatchModel maxMatches;
	int status = parseMatchKey(matchName,maxMatches);
	//最适模板与其余工件进行对应
	string imagePath = savePointCloudCrossPic(originGroupClouds[midIndex], min_x, max_x, midIndex, half_project_length, cloud_pos, head_offset, mid_offset, tail_offset);
	//程序执行时未找到模板,存储自动规划点,供后续工件使用
	if (matchName.length()<1) {
		saveDAMatchModelWithFilePath(imagePath,"tmp",  minAngleThresh,  minIntervalThresh,  minDistanceThresh,  planeWidthThresh,  novangle,  minPointDistanceThresh,  leftNoRPThresh,  skeletonImage);
		//matchName = "tmp.key";
		//cv::Mat img = cv::imread(imagePath);
		//cv::imwrite(saveModelImgPath+"tmp.jpg", img);
	}

	std::tuple<double, cv::Point, cvgeomerty::LINE> t = get2DPCATransfromTuple(imagePath, "", minAngleThresh, minIntervalThresh, minDistanceThresh, novangle);
	double angle = get<0>(t);
	Point stable_point = get<1>(t);
	vector<Point2f> modelSrcCorners;
	for (size_t i = 0; i < corners.size(); i++)
	{
		modelSrcCorners.push_back(Point2f(corners[i].point));
	}
	vector<Point2f> modelCorners;
	Mat M2 = getRotationMatrix2D(stable_point, angle - 90, 1);//计算旋转加缩放的变换矩阵 
	cv::transform(modelSrcCorners, modelCorners, M2);
	for (size_t i = 0; i < originGroupClouds.size(); i++)
	{
		PaintPoints paintPoints = PaintPoints();
		string imagePath = savePointCloudCrossPic(originGroupClouds[i], min_x, max_x, i, leftNoRPThresh, cloud_pos, head_offset, mid_offset, tail_offset);
		if (imagePath.length() < 2)
		{
			continue;
		}
		std::tuple<double, cv::Point, cvgeomerty::LINE> t = get2DPCATransfromTuple(imagePath, "", minAngleThresh, minIntervalThresh, minDistanceThresh, novangle, no_plane);

		double changeAngle = get<0>(t);
		Point changeStablePoint = get<1>(t);
		vector<Point2f> currentCorners;
		M2 = getRotationMatrix2D(changeStablePoint, 90 - changeAngle, 1);//计算旋转加缩放的变换矩阵 
		M2.at<double>(0, 2, 0) = M2.at<double>(0, 2, 0) + changeStablePoint.x - stable_point.x;
		M2.at<double>(1, 2, 0) = M2.at<double>(1, 2, 0) + changeStablePoint.y - stable_point.y;
		cv::transform(modelCorners, currentCorners, M2);
		
#ifdef DEBUG
			{
				cv::Mat src1 = cv::imread(imagePath, cv::IMREAD_COLOR);
				for (int i = 0; i < modelSrcCorners.size(); i++) {
					circle(src1, modelSrcCorners[i], 2, Scalar(0, 0, 255), -1, 8, 0);
				}
				for (int i = 0; i < modelCorners.size(); i++) {
					circle(src1, modelCorners[i], 2, Scalar(255, 0, 0), -1, 8, 0);
				}
				for (int i = 0; i < currentCorners.size(); i++) {
					circle(src1, currentCorners[i], 2, Scalar(0, 255, 0), -1, 8, 0);
				}
				circle(src1, stable_point, 2, Scalar(0, 0, 255), 2, 8, 0);
				circle(src1, changeStablePoint, 2, Scalar(0, 255, 255), 2, 8, 0);
				cv::imshow("src1", src1);
				cv::waitKey();
			}
#endif
			//保存最适图片
			Mat img = cv::imread(imagePath, cv::IMREAD_COLOR);
			if (i == floor(originGroupClouds.size()*1.0 / 2.0)) {
				std::string scan_path = saveScanPath + "image_cross.jpeg";
				cv::imwrite(scan_path, img);
				//长直线
				cvgeomerty::LINE maxline = get<2>(t);
				paintPoints.maxLineStart = maxline.start;
				paintPoints.maxLineEnd = maxline.end;
			}
			

			//保存工件喷涂示意图
			vector<PaintPoint> newCurrentCorners;
			for (size_t j = 0; j < corners.size(); j++)
			{
				newCurrentCorners.push_back(PaintPoint(currentCorners[j], corners[j].angle + changeAngle - 90, corners[j].needChange));
				if (i == floor(originGroupClouds.size()*1.0 / 2.0)) {
					//长直线上包含的点
					bool on = onsegment(paintPoints.maxLineStart, paintPoints.maxLineEnd, newCurrentCorners[newCurrentCorners.size()-1].point);
					cvgeomerty::LINE maxLine = cvgeomerty::LINE(Vec4f(paintPoints.maxLineStart.x, paintPoints.maxLineStart.y, paintPoints.maxLineEnd.x, paintPoints.maxLineEnd.y));
					float distance = cvgeomerty::pointToLineDistance(newCurrentCorners[newCurrentCorners.size() - 1].point, maxLine);
					if (on&&distance<10) {
						paintPoints.maxLineContentPoints.push_back(newCurrentCorners[newCurrentCorners.size() - 1]);
					}
				}
				if (corners[j].needChange)
				{
					circle(img, currentCorners[j], 5, Scalar(0, 255, 0), 2, 8, 0);
				}
				else {
					circle(img, currentCorners[j], 5, Scalar(0, 0, 255), 2, 8, 0);
				}
				double arrowWidth = 30;
				arrowedLine(img, currentCorners[j], Point2f(currentCorners[j].x - arrowWidth, currentCorners[j].y - (arrowWidth)* tan((corners[j].angle + changeAngle - 90) / 180 * M_PI)), Scalar(0, 0, 255), 2, LINE_AA, 0, 0.3);
			}
#ifdef DEBUGC
			//string file_path_w = getFileName(imagePath);
			//file_path_w = saveGroupPath + file_path_w;
			//imwrite(file_path_w.substr(0, file_path_w.length() - 4) + "_mid_pre.jpg", img);
			if (WorkFlowPath != "") {
				string file_name = getFileName(imagePath);
				imwrite(WorkFlowPath + file_name.substr(0, file_name.length() - 4) + "_mid.jpg", img);
			}
#endif 
			paintPoints.picName = imagePath;
			paintPoints.points = newCurrentCorners;
			paintPoints.matchName = matchName;
			paintPoints.havePlane = !no_plane;
			paintPoints.stablePoint = changeStablePoint;
			paintPoints.matchVal = matchVal;
			//计算matchAngle,matchMat
			double matchAngle = 0;
			matchAngle = changeAngle - 90;
			paintPoints.matchAngle = matchAngle;
			Mat affineMat;
			affineMat = getRotationMatrix2D(changeStablePoint, -matchAngle, 1);
			string matchMat = "";
			if (matchName.length() >1) {
				int res = getMatchMat(maxMatches.stable_point, changeStablePoint, affineMat, matchMat);
				if (res == -1) {
					return -1;
				}
			}
			else {
				//程序执行时未找到模板,存储自动规划点,供后续工件使用
				affineMat = M2;
				int res = getMatchMat(stable_point, changeStablePoint, affineMat, matchMat);
				if (res == -1) {
					return -1;
				}
			}
			paintPoints.matchMat = matchMat;
			groupPaintPoints.push_back(paintPoints);
			
		}
		return 0;
}
int useAutoModel(std::vector<PointCloudT::Ptr> &originGroupClouds, float &min_x, float &max_x,vector<PaintPoints> &groupPaintPoints, double minAngleThresh, double minIntervalThresh, double minDistanceThresh,double novangle,double leftNoRPThresh, bool skeletonImage, int cloud_pos, float head_offset, float mid_offset, float tail_offset) {

	for (size_t i = 0; i < originGroupClouds.size(); i++)
	{
		string imagePath = savePointCloudCrossPic(originGroupClouds[i], min_x, max_x, i, leftNoRPThresh, cloud_pos, head_offset, mid_offset, tail_offset);
		if (imagePath.length() < 2)
		{
			continue;
		}
		cvgeomerty::LINE maxDistanceLine;
		vector<PaintPoint> paintCorners;
		vector<PaintPoint>contentPaintCorners;
		getPointCloudCrossPaintCorners(imagePath, paintCorners, contentPaintCorners, maxDistanceLine, minAngleThresh, minIntervalThresh, minDistanceThresh, novangle, skeletonImage);
		double matchAngle = 0;
		Point stable_point(Point(0, 0));//自动点不提供匹配
		bool no_plane = true;
		double matchVal = 0;

		PaintPoints paintPoints = PaintPoints();
		paintPoints.points = paintCorners;
		paintPoints.picName = imagePath;
		paintPoints.havePlane = !no_plane;
		paintPoints.matchAngle = matchAngle;
		paintPoints.matchMat = "";
		paintPoints.matchName = "";
		paintPoints.stablePoint = stable_point;
		paintPoints.matchVal = matchVal;
		paintPoints.maxLineStart = maxDistanceLine.start;
		paintPoints.maxLineEnd = maxDistanceLine.end;
		paintPoints.maxLineContentPoints = contentPaintCorners;
		groupPaintPoints.push_back(paintPoints);
	}
	return 0;
}
int onlyMidMatch(std::vector<PointCloudT::Ptr> &originGroupClouds, PointCloudT::Ptr &cloud_real, float &min_x, float &max_x, string model_name, vector<PaintPoints> &groupPaintPoints, double minAngleThresh, double minIntervalThresh, double minDistanceThresh , double planeWidthThresh, double novangle, double minPointDistanceThresh , double leftNoRPThresh, bool skeletonImage,int cloud_pos, float head_offset, float mid_offset, float tail_offset) {
	if (originGroupClouds.size() == 0) {
		LOG(ERRORL) << "点云为空错误";
		throw exception((std::string(__FUNCTION__) + ":点云为空错误").c_str());
	}
	int midIndex = floor(originGroupClouds.size()*1.0 / 2.0);
	pcl::PointXYZ min_op, max_op;
	pcl::getMinMax3D(*(originGroupClouds[midIndex]), min_op, max_op);
	double ody = abs(min_op.y - max_op.y);
	double odz = abs(min_op.z - max_op.z);
	//小工件
	int ody_thresh = 65;
	int odz_thresh = 60;
	std::cout << "****************************************ody" << ody << std::endl;
	std::cout <<"****************************************odz"<< odz << std::endl;
	if (ody < ody_thresh) {
		int res = miniWorkpiece(originGroupClouds, cloud_real, min_x, max_x, model_name, groupPaintPoints, planeWidthThresh, cloud_pos, head_offset, mid_offset, tail_offset);
		if (res) {
			LOG(ERRORL) << "小工件规划错误";
			throw exception((std::string(__FUNCTION__) + ":小工件规划错误").c_str());
		}
	}
	else {
		//固定模板
		if (model_name.length() > 1) {
			int res = useFixedModel(originGroupClouds, min_x, max_x, model_name, groupPaintPoints, minAngleThresh, minIntervalThresh, minDistanceThresh, planeWidthThresh, novangle, minPointDistanceThresh, leftNoRPThresh, skeletonImage, cloud_pos, head_offset, mid_offset, tail_offset);
			if (res) {
				LOG(ERRORL) << "使用固定模板错误";
				throw exception((std::string(__FUNCTION__) + ":使用固定模板错误").c_str());
			}
		}
		else {
			//得到groupPaintPoints
			int res = useMidModel(originGroupClouds, min_x, max_x, groupPaintPoints, minAngleThresh, minIntervalThresh, minDistanceThresh, planeWidthThresh, novangle, minPointDistanceThresh, leftNoRPThresh, skeletonImage, cloud_pos, head_offset, mid_offset, tail_offset);
			if (res == -1) {
				LOG(ERRORL) << "使用最适模板错误";
				throw exception((std::string(__FUNCTION__) + ":使用最适模板错误").c_str());
			}
		}
	}
	return 0;
}

string getMatchMessage(std::vector<PointCloudT::Ptr> &originGroupClouds, PointCloudT::Ptr &cloud_real, float &min_x, float &max_x, string match_name, bool need_only_use_mid, double minAngleThresh, double minIntervalThresh, double minDistanceThresh , double planeWidthThresh, double novangle, double minPointDistanceThresh, double leftNoRPThresh, bool skeletonImage, int cloud_pos, float head_offset, float mid_offset, float tail_offset) {

	vector<PaintPoints> groupPaintPoints;
	if (need_only_use_mid)
	{
		//得到groupPaintPoints
		int res = onlyMidMatch(originGroupClouds, cloud_real, min_x, max_x, match_name, groupPaintPoints, minAngleThresh, minIntervalThresh, minDistanceThresh, planeWidthThresh, novangle, minPointDistanceThresh, leftNoRPThresh, skeletonImage, cloud_pos,  head_offset,  mid_offset,  tail_offset);
		if (res == -1)return"";
	}

	if (groupPaintPoints.size()<2)
	{
		if (groupPaintPoints.size()>0)
		{
			PaintPoints paintPoints = groupPaintPoints[0];
			if (paintPoints.points.size()<1)
			{
				return "";
			}
		}
		else {
			return "";
		}
	}

	char * jsonString = createLinesToJSON(groupPaintPoints);
	cout << jsonString << endl;

	return jsonString;
}


void creatFolderInit() {
	//createFolder(savePath);
	//createFolder(saveCloudPath);
	//createFolder(saveGroupPath);
	createFolder(saveModelImgPath);
	createFolder(saveModelPath);
	createFolder(saveScanPath);
	createFolder(logger::saveLoggerPath);
}
int AluminumPointCloudResolve(char* output_result_json, char * cloud_front_file, char* cloud_file, char* match_name = "", double cloud_length= CloudLength, double stepDistanceThresh = StepDistanceThresh, double poleWidthThresh = PoleWidthThresh, int poleSizeThresh = PoleSizeThresh, double poleOffesetThresh = PoleOffesetThresh, bool need_only_use_mid = NeedOnlyUseMid, double minAngleThresh = MinAngleThresh, double minIntervalThresh = MinIntervalThresh, double minDistanceThresh = MinDistanceThresh, double planeWidthThresh = PlaneWidthThresh, double novangle = Novangle, double minPointDistanceThresh = MinPointDistanceThresh, double leftNoRPThresh = LeftNoRPThresh, bool skeletonImage = SkeletonImage,int cloud_pos = CloudPos, float head_offset = HeadOffset, float mid_offset = MidOffset, float tail_offset = TailOffset)
{	
#ifdef DEBUGC
	//创建所需文件夹
	creatFolderInit();
#endif 
	//初始化参数
	WorkFlowPath = "";
	std::vector<PointCloudT::Ptr> originGroupClouds;
	PointCloudT::Ptr cloud_real(new PointCloudT);
	float min_x;
	float max_x;
	vector<PaintPoints> groupPaintPoints;
	string model_name = match_name;
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
	//判断是否为小工件
	int midIndex = floor(originGroupClouds.size()*1.0 / 2.0);
	pcl::PointXYZ min_op, max_op;
	pcl::getMinMax3D(*(originGroupClouds[midIndex]), min_op, max_op);
	double ody = abs(min_op.y - max_op.y);
	double odz = abs(min_op.z - max_op.z);
	int ody_thresh = 65;
	int odz_thresh = 60;
	std::cout << "ody:" << ody << std::endl;
	std::cout << "odz:" << odz << std::endl;
	if (ody < ody_thresh) {
		int res = miniWorkpiece(originGroupClouds, cloud_real, min_x, max_x, model_name, groupPaintPoints, planeWidthThresh, cloud_pos, head_offset, mid_offset, tail_offset);
		if (res) {
			LOG(ERRORL) << "小工件规划错误";
			throw exception((std::string(__FUNCTION__) + ":小工件规划错误").c_str());
		}
	}
	else if (need_only_use_mid)	//使用中间最适模板
	{
		if (model_name.length() > 1) {
			int res = useFixedModel(originGroupClouds, min_x, max_x, model_name, groupPaintPoints, minAngleThresh, minIntervalThresh, minDistanceThresh, planeWidthThresh, novangle, minPointDistanceThresh, leftNoRPThresh, skeletonImage, cloud_pos, head_offset, mid_offset, tail_offset);
			if (res) {
				LOG(ERRORL) << "使用固定模板错误";
				throw exception((std::string(__FUNCTION__) + ":使用固定模板错误").c_str());
			}
		}
		else {
			//LOG(ERRORL) << "自动未规划错误";
			//throw exception((std::string(__FUNCTION__) + ":自动未规划错误").c_str());
			int res = useAutoModel(originGroupClouds, min_x, max_x, groupPaintPoints,  minAngleThresh,  minIntervalThresh,  minDistanceThresh,  novangle,  leftNoRPThresh,  skeletonImage,  cloud_pos,  head_offset,  mid_offset,  tail_offset);
			if (res) {
				LOG(ERRORL) << "使用自动规划错误";
				throw exception((std::string(__FUNCTION__) + ":使用自动规划错误").c_str());
			}
		}
	}
	else {
		LOG(ERRORL) << "请将need_only_use_mid置为true";
		throw exception((std::string(__FUNCTION__) + ":请将need_only_use_mid置为true").c_str());
	}

	if (groupPaintPoints.size() < 2)
	{
		if (groupPaintPoints.size() > 0)
		{
			PaintPoints paintPoints = groupPaintPoints[0];
			if (paintPoints.points.size() < 1)
			{
				LOG(ERRORL) << "生成paintPoints错误";
				throw exception((std::string(__FUNCTION__) + ":生成paintPoints错误").c_str());
			}
		}
		else {
			LOG(ERRORL) << "生成groupPaintPoints错误";
			throw exception((std::string(__FUNCTION__) + ":生成groupPaintPoints错误").c_str());
		}
	}

	char * jsonString = createLinesToJSON(groupPaintPoints);
	cout << jsonString << endl;

	string result_json_path = WorkFlowPath.length() > 0 ? (WorkFlowPath + "result_json" + ".key") : (savePath + "result_json" + ".key");
	try
	{
		ofstream outfile(result_json_path, ios::binary);
		outfile << jsonString;
		outfile.close();
	}
	catch (const std::exception&)
	{
		result_json_path = "";
	}

	std::string result_json = jsonString;
	std::memcpy(output_result_json, result_json.c_str(), result_json.length());
	return result_json.length() < 2 ? -1 : result_json.length();
}
int SaveModel(char* cloud_file, char* out_name = "", double cloud_length = CloudLength, double stepDistanceThresh = StepDistanceThresh, double poleWidthThresh = PoleWidthThresh, int poleSizeThresh = PoleSizeThresh, double poleOffesetThresh = PoleOffesetThresh)
{
#ifdef DEBUGC
	//创建所需文件夹
	creatFolderInit();
#endif 
	//初始化参数
	WorkFlowPath = "";
	std::vector<PointCloudT::Ptr> originGroupClouds;
	PointCloudT::Ptr cloud_real(new PointCloudT);
	float min_x = 0;
	float max_x = 0;

	string cloud_front_file_s;
	string cloud_file_s;
	if (string(cloud_file).substr(string(cloud_file).length()-3, string(cloud_file).length())=="pcd"|| string(cloud_file).substr(string(cloud_file).length() - 3, string(cloud_file).length()) == "ply") {
		cloud_front_file_s = cloud_file;
		cloud_file_s = cloud_file;
	}
	else {
		string file_path = getFilePath(cloud_file);
		string  file_name = getFileName(file_path.substr(0, file_path.length() - 1));
		if (file_name == "Algorithm") {
			file_path = getFilePath(file_path.substr(0, file_path.length() - 1));
		}
		//std::cout << "file_path" << file_path << std::endl;
		//std::cout << "file_name" << file_name << std::endl;
		cloud_front_file_s = file_path + "scan_save_fast_1.pcd";
		cloud_file_s = file_path + "scan_save_fast_0.pcd";
	}


	std::cout << "out_name:" << out_name << std::endl;
	std::cout << "cloud_front_file:" << cloud_front_file_s << std::endl;
	std::cout << "cloud_file:" << cloud_file_s << std::endl;

	//点云滤波
	int res = filterRealPointCloud((char*)cloud_front_file_s.c_str(), (char*)cloud_file_s.c_str(), originGroupClouds, cloud_real, min_x, max_x, cloud_length, stepDistanceThresh, poleWidthThresh, poleSizeThresh, poleOffesetThresh);
	if (res == -1) {
		LOG(ERRORL) << "点云滤波错误";
		throw exception((std::string(__FUNCTION__) + ":点云滤波错误").c_str());
	}
	if (originGroupClouds.size() == 0) {
		LOG(ERRORL) << "点云为空错误";
		throw exception((std::string(__FUNCTION__) + ":点云为空错误").c_str());
	}
	int midIndex = floor(originGroupClouds.size()*1.0 / 2.0);
	if (out_name == "") {
		LOG(ERRORL) << "out_name为空错误";
		throw exception((std::string(__FUNCTION__) + ":out_name为空错误").c_str());
	}
	string imagePath = savePointCloudCrossPic(originGroupClouds[midIndex], min_x, max_x, midIndex, half_project_length, 0, 0, 0, 0, string(out_name));
	return 0;
}
int SaveDistanceAngleMatchModelWithImageFilePath(const char* file_path, const char* out_path = "", double minAngleThresh = MinAngleThresh, double minIntervalThresh = MinIntervalThresh, double minDistanceThresh = MinDistanceThresh, double planeWidthThresh = PlaneWidthThresh, double novangle = Novangle, double minPointDistanceThresh = MinPointDistanceThresh, double leftNoRPThresh = LeftNoRPThresh, bool skeletonImage = SkeletonImage, int cloud_pos = CloudPos, float head_offset = HeadOffset, float mid_offset = MidOffset, float tail_offset = TailOffset) {
	WorkFlowPath = "";
#ifdef DEBUGC
	creatFolderInit();
#endif 

	string newpath = saveDAMatchModelWithFilePath(file_path, out_path, minAngleThresh, minIntervalThresh, minDistanceThresh, planeWidthThresh, novangle, minPointDistanceThresh, leftNoRPThresh, skeletonImage);
	return newpath.length() < 2 ? -1 : 1;
}
int AluminumImageResolve(char* output_result_json, char* image_file, double minAngleThresh = MinAngleThresh, double minIntervalThresh = MinIntervalThresh, double minDistanceThresh = MinDistanceThresh, double planeWidthThresh = PlaneWidthThresh, double novangle = Novangle, double minPointDistanceThresh = MinPointDistanceThresh, double leftNoRPThresh = LeftNoRPThresh, bool skeletonImage = SkeletonImage, int cloud_pos = CloudPos, float head_offset = HeadOffset, float mid_offset = MidOffset, float tail_offset = TailOffset)
{
	WorkFlowPath = "";
#ifdef DEBUGC
	creatFolderInit();
#endif 

	bool no_plane;
	string matchName = "";
	string matchMat = "";
	double matchAngle = 0;
	double matchVal = 0;

	PaintPoints paintPoints = PaintPoints();

	Point tmpStablePoint;

	//vector<PaintPoint> corners = matchKeyPointToCorners(image_file, matchName, matchVal, matchAngle, matchMat, no_plane, &tmpStablePoint, minAngleThresh, minIntervalThresh, minDistanceThresh, planeWidthThresh, novangle, minPointDistanceThresh, leftNoRPThresh, skeletonImage);
	vector<PaintPoint> corners;
	Point2f center = Point2f(PICWIDTH_2, PICWIDTH_2);
	corners.push_back(PaintPoint(center, 0, false));

	paintPoints.picName = image_file;
	paintPoints.points = corners;
	paintPoints.matchName = matchName;
	paintPoints.matchMat = matchMat;
	paintPoints.matchAngle = matchAngle;
	paintPoints.matchVal = matchVal;
	paintPoints.havePlane = !no_plane;
	paintPoints.stablePoint = tmpStablePoint;

	vector<PaintPoints> groupPaintPoints;

	groupPaintPoints.push_back(paintPoints);

	string result_json;
	if (groupPaintPoints.size() < 2)
	{
		if (groupPaintPoints.size() > 0)
		{
			PaintPoints paintPoints = groupPaintPoints[0];
			if (paintPoints.points.size() < 1)
			{
				result_json = "";
			}
		}
		else {
			result_json = "";
		}
	}

	result_json = createLinesToJSON(groupPaintPoints);

	std::memcpy(output_result_json, result_json.c_str(), result_json.length());
	return result_json.length() < 2 ? -1 : result_json.length();
}
void ModelShow(char* json_result) {
	std::vector<MatchModelC> v_matches;
	int status = parsingJSONToKeyPoints(json_result, v_matches);

	for (int i = 0; i < v_matches.size(); i++) {
		std::cout << "************************************************:" << std::endl;
		std::string matchKey = v_matches[i].match_name;
		Point stablePoint_src = v_matches[i].stable_point;
		double angle = v_matches[i].match_angle;
		std::cout << "*********v_matches[i].match_angle" << v_matches[i].match_angle << std::endl;
		std::cout << "matchKey:" << matchKey << std::endl;
		if (matchKey == "") {
			cv::Mat m = cv::imread(v_matches[i].pic_name);
			cv::imshow("no model", m);
			cv::waitKey(0);
			continue;
		}
		{
			TCHAR szLongPathName[_MAX_PATH];
			GetModuleFileName(NULL, szLongPathName, _MAX_PATH);
			string runRealPath(szLongPathName);
			int pos = runRealPath.find_last_of('\\');
			runRealPath = runRealPath.substr(0, pos);

			std::string model_img_path = saveModelImgPath + matchKey.substr(0, matchKey.size() - 3) + "jpg";
			std::cout << "model img path:" << model_img_path << std::endl;
			std::cout << "src img path:" << v_matches[i].pic_name << std::endl;
			cv::Mat model = cv::imread(model_img_path);
			cv::Mat src = cv::imread(v_matches[i].pic_name);



			//解析model corner************start*******************************************
			std::string match_dir = saveModelPath;
			string fileName = matchKey;
			string filePath = match_dir + fileName;
			std::cout << "model key path:" << filePath << std::endl;


			string jsonString = "";

			try
			{
				ifstream infile(filePath, ios::binary);

				char *buffer = new char[99999];

				while (!infile.eof())
				{
					infile >> buffer;

					jsonString = jsonString + string(buffer);

				}
				infile.close();
				delete[] buffer;
			}
			catch (const std::exception&)
			{

			}
			std::cout << "jsonString:" << jsonString << std::endl;
			MatchModel matches;
			vector<PaintPoint> modelKey;
			vector<Point> corners;

			int status = parsingJSONToKeyPoints(jsonString.c_str(), matches);
			std::cout << "status:" << status << std::endl;

			corners.clear();
			modelKey.clear();
			modelKey = matches.points;
			for (size_t i = 0; i < modelKey.size(); i++) {
				corners.push_back(modelKey[i].point);
			}

			for (size_t i = 0; i < corners.size(); i++)
			{

				std::cout << "modelKey angle:" << modelKey[i].angle << std::endl;

				if (modelKey[i].needChange)
				{
					std::cout << "needChange" << corners[i] << std::endl;

					circle(model, corners[i], 5, Scalar(0, 255, 0), 2, 8, 0);
				}
				else {
					std::cout << "no needChange" << corners[i] << std::endl;
					circle(model, corners[i], 5, Scalar(0, 0, 255), 2, 8, 0);
				}
				double arrowWidth = 30;
				//arrowedLine(model, corners[i], Point2f(corners[i].x - arrowWidth, corners[i].y - (arrowWidth)* tan((matches.points[0].angle) / 180 * CV_PI)), Scalar(0, 0, 255), 2, LINE_AA, 0, 0.3);
			}

			//解析model corner************end*******************************************

			//解析mat************start*******************************************
			std::cout << "matchMat" << v_matches[i].match_mat << std::endl;
			std::string mat = v_matches[i].match_mat;
			char *substr0 = strtok((char*)(char*)mat.c_str(), "_");
			char charlist0[50][50] = { "" };
			int i = 0;
			while (substr0 != NULL) {
				strcpy(charlist0[i], substr0);
				i++;
				substr0 = strtok(NULL, "_");
			}
			char *substr1 = strtok((char*)(char*)charlist0[0], ",");
			char charlist1[50][50] = { "" };
			int j = 0;
			while (substr1 != NULL) {
				strcpy(charlist1[j], substr1);
				j++;
				substr1 = strtok(NULL, ",");
			}

			substr1 = strtok((char*)(char*)charlist0[1], ",");
			while (substr1 != NULL) {
				strcpy(charlist1[j], substr1);
				j++;
				substr1 = strtok(NULL, ",");
			}

			Mat affineMat(2, 3, CV_32FC1);
			int k = 0;
			for (int i = 0; i < affineMat.rows; i++)
			{
				for (int j = 0; j < affineMat.cols; j++)
				{

					affineMat.at<float>(i, j) = atof(charlist1[k]);
					k++;
				}
			}
			std::cout << "affineMat:" << affineMat << std::endl;
			//解析mat************end*******************************************

			cv::transform(corners, corners, affineMat);


			Point stablePoint_model = matches.stable_point;
			Point tt = stablePoint_model - stablePoint_src;
			std::cout << "stablePoint_src:" << stablePoint_src.x << "$$" << stablePoint_src.y << std::endl;
			std::cout << "stablePoint_model:" << stablePoint_model.x << "$$" << stablePoint_model.y << std::endl;
			std::cout << "************tt:" << tt.x << "$$" << tt.y << std::endl;
			if (status == 1)
			{
				for (size_t j = 0; j < corners.size(); j++)
				{


					//corners[j] = corners[j]+(stablePoint_model- stablePoint_src);
					//if (v_matches[i].points[j].needChange)
					if (modelKey[j].needChange)
					{
						std::cout << "needChange" << corners[j] << std::endl;

						circle(src, corners[j], 5, Scalar(0, 255, 0), 2, 8, 0);
					}
					else {
						std::cout << "no needChange" << corners[j] << std::endl;
						circle(src, corners[j], 5, Scalar(0, 0, 255), 2, 8, 0);
					}
					double arrowWidth = 30;

					//arrowedLine(src, corners[j], Point2f(corners[j].x - arrowWidth, corners[j].y - (arrowWidth)* tan((angle) / 180 * CV_PI)), Scalar(0, 0, 255), 2, LINE_AA, 0, 0.3);
				}
			}
			//cv::imshow(matchKey.substr(0, matchKey.size() - 4), model);
			//cv::imshow("src", src);

			cv::Mat out(Size(1000, 500), src.type(), Scalar(0, 0, 0));

			for (int row = 0; row < src.rows; row++) {
				for (int col = 0; col < src.cols; col++) {
					Vec3b rgb_model = model.at<Vec3b>(row, col);
					Vec3b rgb_src = src.at<Vec3b>(row, col);
					out.at<Vec3b>(row, col) = rgb_model;
					out.at<Vec3b>(row, col + 500) = rgb_src;
				}
			}
			cv::imshow(matchKey.substr(0, matchKey.size() - 4), out);
			cv::waitKey(0);
		}
	}


}


int main()
{
	//double xyzrxryrz[6] = { 0,0,1,30,0,0 };
	//getRotation_x(xyzrxryrz,30);
	//return 0;

	//double xyzrxryrz1[6] = { 0,0,0,0,0,0 };
	//double xyzrxryrz2[6] = { 1,0,0,45,0,0 };
	//double angle_spling = 30;
	//double rx;
	//double ry;
	//double rz;
	//CRMath::getSplingRxRyRz(xyzrxryrz1, xyzrxryrz2, angle_spling, rx, ry, rz);
	////double rxryrz[3] = { 166,-33,88 };
	////double rxryrz_spling[3] = { 0,30,0 };
	////double rxryrz_out[3] = {0,0,0};
	////getSplingRxRyRz(rxryrz, rxryrz_spling, rxryrz_out);
	//std::cout << "rx: ry :rz:" << rx <<"  "<< ry << "  " << rz << std::endl;
	//system("pause");
	//return 0;

//	Mat img1 = imread("E:\\project\\test\\opencv\\OpencvCmake\\pic\\2.jpg", IMREAD_GRAYSCALE);    //右图
//	Mat img2 = imread("E:\\project\\test\\opencv\\OpencvCmake\\pic\\3.jpg", IMREAD_GRAYSCALE);    //左图
//	//imshow("img1", img1);
//	//imshow("img2", img2);
//	//waitKey();
//	PointCloudT::Ptr cloud_img1(new PointCloudT);
//	PointCloudT::Ptr cloud_img2(new PointCloudT);
//	img2PointCloud(img1, cloud_img1);
//	img2PointCloud(img2, cloud_img2);
//	double score = 0;
//	Eigen::Matrix4f tm = Eigen::Matrix4f::Identity();
//	icpRegistration(cloud_img1, cloud_img2, score,tm);
//	pcl::transformPointCloud(*cloud_img1, *cloud_img1, tm);
//#ifdef DEBUGC
//	{
//		//tm = tm.inverse();
//		Mat M = getRotationMatrix2D(Point2f(0,0), 0, 1);
//		float K[2][3] = { tm(0, 0),tm(0, 1), tm(0, 3), tm(1, 0),tm(1, 1), tm(1, 3) };
//		M = cv::Mat(2, 3, CV_32FC1, K);
//
//		std::cout << "tm:" << tm << std::endl;
//		std::cout <<"M:"<< M << std::endl;
//		warpAffine(img1, img1, M, img1.size());
//		//imshow("img1", img1);
//		//imshow("img2", img2);
//		//waitKey();
//		//vector<Point2f> transformPoints;
//		//Mat M2 = getRotationMatrix2D(stable_point, angle - 90, 1);//计算旋转加缩放的变换矩阵 
//		//cv::transform(modelCorners, transformPoints, M2);
//		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = getViewer();
//		viewerCloud(viewer, cloud_img1, 255, 0, 0, 1, 0);
//		viewerCloud(viewer, cloud_img2, 0, 255, 0, 2, 1);
//		viewerStop(viewer);
//	}
//#endif
//	//imshow("img1", img1);
//	//imshow("img2", img2);
//	//waitKey();
//	return 0;
	while (1) {
		std::string mode;
		std::cout << "intput mode:" << std::endl;
		std::cout << "file" << std::endl;
		std::cout << "0.image match" << std::endl;
		std::cout << "1.make temp" << std::endl;
		std::cout << "2.match" << std::endl;
		std::cin >> mode;
		if (mode == "file") {
			std::string dirString;
			std::cout << "intput match dir**:" << std::endl;
			std::cin >> dirString;
			vector<std::string> fileNames;
			fileutil::getFiles(dirString, fileNames);
			char* json_result = new char[10000];
			for (int i = 0; i < fileNames.size(); i++) {
				string name = fileutil::getFileName(fileNames[i]);
				if (name == "scan_save_fast_0.pcd") {
					char * fileName = (char*)fileNames[i].data();
					char * fileFCloud = (char*)(fileNames[i].substr(0, fileNames[i].length()-5)+"1.pcd").data();
					try {
						clock_t start = clock();
						//AluminumPointCloudResolve(json_result, (char *)(fileFCloud), (char *)(fileCloud), "A1");
						AluminumPointCloudResolve(json_result, (char *)(fileFCloud), (char *)(fileName), "");
						clock_t end = clock();
						cout << "AluminumPointCloudResolve time" << (double)(end - start) / CLOCKS_PER_SEC << endl;

					}
					catch (exception &e)
					{
						cout << " 捕获了异常对象 " << e.what() << endl;
					}
#ifdef DEBUG
					ModelShow(json_result);
#endif
				}
			}
		}
		else if (mode == "0") {
			std::string image_fileString;
			std::cout << "intput match fileNameString**:" << std::endl;
			std::cin >> image_fileString;
			char * image_file = (char*)image_fileString.data();
			char* json_result = new char[10000];
			AluminumImageResolve(json_result, image_file);
			std::cout << json_result << std::endl;
		}
		else if (mode == "1") {
			std::string fileNameString;
			std::cout << "intput match fileNameString**:" << std::endl;
			std::cin >> fileNameString;
			std::string modelNameString;
			std::cout << "intput modelName**:" << std::endl;
			std::cin >> modelNameString;
			char * modelName = (char*)modelNameString.data();
			char * fileName = (char*)fileNameString.data();
			try {
				//SaveDistanceAngleMatchModelWithImageFilePath(fileName, modelName);
				SaveModel(fileName, modelName);

			}
			catch (exception &e)
			{
				cout << " 捕获了异常对象 " << e.what() << endl;
			}


		}
		else {
			std::string fileFCloudString;
			std::cout << "intput fileFrontCloudString**:" << std::endl;
			std::cin >> fileFCloudString;
			char * fileFCloud = (char*)fileFCloudString.data();
			std::string fileCloudString;
			std::cout << "intput fileCloudString**:" << std::endl;
			std::cin >> fileCloudString;
			char * fileCloud = (char*)fileCloudString.data();

			char* json_result = new char[10000];

			try {
				clock_t start = clock();
				AluminumPointCloudResolve(json_result, (char *)(fileFCloud), (char *)(fileCloud), (char*)mode.data());
				//int len = AluminumPointCloudResolve(json_result, (char *)(fileFCloud),(char *)(fileCloud), "");
				//AluminumPointCloudResolve(json_result, (char *)(fileFCloud), (char *)(fileCloud), "", CloudLength, StepDistanceThresh, PoleWidthThresh, PoleSizeThresh, PoleOffesetThresh, NeedOnlyUseMid, MinAngleThresh, MinIntervalThresh, MinDistanceThresh, PlaneWidthThresh, Novangle, MinPointDistanceThresh, LeftNoRPThresh, SkeletonImage, 0, 100, 100, 100);
				string json(json_result);
				clock_t end = clock();
				cout << "AluminumPointCloudResolve time" << (double)(end - start) / CLOCKS_PER_SEC << endl;
				
			}
			catch (exception &e)
			{
				cout << " 捕获了异常对象 " << e.what() << endl;
			}
#ifdef DEBUG
			ModelShow(json_result);
#endif
		}

	}

	int i;
	cin >> i;

	return 0;
}

