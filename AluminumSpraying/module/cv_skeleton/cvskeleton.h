#pragma once
#include <opencv2/opencv.hpp>

namespace cvskeleton {
	cv::Mat thinImage(const cv::Mat & src, const int maxIterations = -1);
}
