#pragma once

#include "cvgeomerty.h"
#include "paint_model.h"

namespace CVSearchPoints {
	using namespace std;
	using namespace cv;
	//拟合一组线段
	vector<Vec4f> fitGroupLine(vector<Vec4f> lines, double min_angle, double min_distance, double min_interval);

	//邻近点做ignore，保留中点
	vector<Point3f> ignorePoints(vector<Point3f> pointin, double min_distance, bool ignore_angle);

	//基于平面纵向igonre邻近点，保留距离平面远的点
	vector<Point3f> ignoreVertaicalSamePoints(vector<Point3f> pointin, double min_distance, cvgeomerty::LINE planeLine);

	vector<PaintPoint> ignoreFinalVertaicalSamePoints(vector<PaintPoint> pointin, double min_distance, cvgeomerty::LINE planeLine);
	int findCorner(cv::Mat &skeleton, std::vector<cv::Point2f> &corners);
	bool onsegment(cv::Point2f pi, cv::Point2f pj, cv::Point2f Q);
	/*
	原始角点获取

	file_path:截面图片路径
	minAngleThresh:两直线夹角最小角度阈值
	minIntervalThresh:两直线距离最小阈值
	minDistanceThresh:线段距离最小阈值
	planeWidthThresh:平面偏移量，代表找出来的最长线段左后扩展的数值
	novangle:最大直线斜角超过30度说明不是平面且图像转正了表明没有平面
	minPointDistanceThresh:控制点与点之间间隔的阈值，小于这个值滤掉取中点
	skeletonImage:对图片进行骨骼化设置
	*/
	int getPointCloudCrossOriginCorners(vector<Point3f> &newCorners, bool &no_plane, cvgeomerty::LINE &maxDistanceLine, const std::string &file_path, double minAngleThresh, double minIntervalThresh, double minDistanceThresh, double planeWidthThresh, double novangle, double minPointDistanceThresh, bool skeletonImage);

	/*
	喷涂角点获取

	newCorners:输入原始角点
	no_plane:是否没有虚拟平面
	maxDistanceLine:虚拟平面线
	file_path:截面图片路径
	minAngleThresh:两直线夹角最小角度阈值
	minIntervalThresh:两直线距离最小阈值
	minDistanceThresh:线段距离最小阈值
	planeWidthThresh:平面偏移量，代表找出来的最长线段左后扩展的数值
	novangle:最大直线斜角超过30度说明不是平面且图像转正了表明没有平面
	minPointDistanceThresh:控制点与点之间间隔的阈值，小于这个值滤掉取中点
	leftNoRPThresh:控制平面点和右边的点在左边的点纵向距离阈值内忽略
	skeletonImage:对图片进行骨骼化设置
	*/
	//vector<PaintPoint> getPointCloudCrossPaintCorners( vector<Point3f> &newCorners, bool &no_plane, cvgeomerty::LINE &maxDistanceLine, const std::string &file_path, double minDistanceThresh, double planeWidthThresh, double novangle, double minPointDistanceThresh, double leftNoRPThresh, bool skeletonImage);
	vector<PaintPoint> getPointCloudCrossPaintCorners(vector<PaintPoint> &finalCorners, bool &no_plane, cvgeomerty::LINE &maxDistanceLine, const std::string &file_path, double minDistanceThresh, double planeWidthThresh, double novangle, double minPointDistanceThresh, double leftNoRPThresh);
	int getPointCloudCrossOriginCornersPre(vector<Point3f> &newCorners, bool &no_plane, cvgeomerty::LINE &maxDistanceLine, const std::string &file_path, double minAngleThresh, double minIntervalThresh, double minDistanceThresh, double novangle, bool skeletonImage);
	int getPointCloudCrossPaintCorners(const std::string &file_path, vector<PaintPoint>&paintCorners, vector<PaintPoint>&contentPaintCorners, cvgeomerty::LINE &maxDistanceLine, double minAngleThresh, double minIntervalThresh, double minDistanceThresh, double novangle, bool skeletonImage);
}