#pragma once

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include "pcl_filter_cloud.h"
#include "CRMath.h"
#include "parameters.h"
#include "global.h"
#include "logger.h"
#include "pcl_filter_helper.h"
#include "file_util.h"
#include "pclgeomerty.h"
#include "cvskeleton.h"

namespace cv_search_feature{
	using namespace std;
	using namespace cv;
	using namespace pcl_filter_cloud;
	using namespace CRMath;
	using namespace AluminumParameters;
	using namespace AluminumGlobal;
	using namespace logger;
	using namespace PCLFilterHelper;
	using namespace fileutil;
	using namespace cvgeomerty;
	int searchLineSEG(const std::string &file_path, Vec4f &plane_line, cvgeomerty::LINE &LineSEG, Eigen::Affine3f &transform_matrix, double minAngleThresh, double minIntervalThresh, double minDistanceThresh);
}