#pragma once
#include "paint_model.h"
#include "cv_search_points.h"
#include "json_model_parse.h"

#include "file_util.h"
#include "string_util.h"

#include "Win32\OniPlatformWin32.h"
#include <direct.h>
#include "cvskeleton.h"

namespace CVMatchPoints {
	using namespace std;
	using namespace cv;
	const int NoPlaneS = 3;
	//std::tuple<double, cv::Point> get2DPCATransfromTuple(const string &file_path, const char * out_path = "", double minAngleThresh = 10, double minIntervalThresh = 20, double minDistanceThresh = 5, double novangle = 20);
	//double get2DPCATransfrom(const string &file_path, const char * out_path = "", double minAngleThresh = 10, double minIntervalThresh = 20, double minDistanceThresh = 5, double novangle = 20);
	std::tuple<double, cv::Point, cvgeomerty::LINE> get2DPCATransfromTuple(const string &file_path, const char *out_path, double minAngleThresh, double minIntervalThresh, double minDistanceThresh, double novangle, int no_plane_s = NoPlaneS);
	string saveMatchModelWithFilePath(const string &file_path);

	string saveDAMatchModelWithFilePath(const string file_path, const string out_path = "", double minAngleThresh = 10, double minIntervalThresh = 8, double minDistanceThresh = 5, double planeWidthThresh = 15, double novangle = 10, double minPointDistanceThresh = 10, double leftNoRPThresh = 60, bool skeletonImage = true);

	string odaMatchKeyPoint(const string &file_path, vector<PaintPoint> corners, const string &match_dir, double &outMatchVal, double &outMatchAngle, string &outMatchMat, cv::Point *outStablePoint);

	vector<PaintPoint> matchKeyPointToCorners(const string &file_path, string &matchName, double &matchVal,double &matchAngle, string &matchMat,bool &no_plane, cv::Point *outStablePoint, double minAngleThresh = 10, double minIntervalThresh = 8, double minDistanceThresh = 5, double planeWidthThresh = 15, double novangle = 10, double minPointDistanceThresh = 10, double leftNoRPThresh = 60, bool skeletonImage = true);
	vector<PaintPoint>getPointCloudCrossMatchCorners(vector<Point3f> &newCorners, bool &no_plane, cvgeomerty::LINE &maxDistanceLine, const string file_path, double minAngleThresh, double minIntervalThresh, double minDistanceThresh, double planeWidthThresh, double novangle, double minPointDistanceThresh, double leftNoRPThresh);
}
