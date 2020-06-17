#include "cv_search_points.h"
#include "cvskeleton.h"
#include "file_util.h"
#include "global.h"
#include "logger.h"

namespace CVSearchPoints {
	using namespace std;
	using namespace cv;
	using namespace fileutil;
	using namespace AluminumGlobal;
	using namespace logger;

	int getPointCloudCrossclassifyCorners(vector<Point3f> &newCorners, bool &no_plane, cvgeomerty::LINE &maxDistanceLine, const std::string &file_path, double minDistanceThresh, double planeWidthThresh, double novangle, double minPointDistanceThresh,  bool skeletonImage);
	//拟合一组线段
	vector<Vec4f> fitGroupLine(vector<Vec4f> lines, double min_angle, double min_distance, double min_interval) {
		vector<Vec4f> fitGroupLines;

		vector<cvgeomerty::LINE> cvLines;
		for (size_t i = 0; i < lines.size(); i++)
		{
			cvLines.push_back(lines[i]);
		}

		vector<cvgeomerty::LINE> cvNewLines;

		bool sucess = cvgeomerty::fitLineSeg(cvLines, min_distance, min_interval, min_angle, cvNewLines);
		if (sucess)
		{
			for (size_t i = 0; i < cvNewLines.size(); i++)
			{
				fitGroupLines.push_back(cvNewLines[i].line);
			}
		}

		return fitGroupLines;
	}

	//邻近点做ignore，保留中点
	vector<Point3f> ignorePoints(vector<Point3f> pointin, double min_distance, bool ignore_angle) {
		vector<Point3f> finalCorners;

		while (pointin.size() > 0)//邻近点分组
		{
			Point3f tmpP1 = pointin[pointin.size() - 1];

			pointin.pop_back();

			double errorAngle = 90;
			double errorAngleThresh = 28;

			if (abs(tmpP1.z) < (errorAngle + errorAngleThresh) && abs(tmpP1.z) > (errorAngle - errorAngleThresh) && !ignore_angle)
			{
				continue;
			}

			vector<Point3f> groupCorner;
			groupCorner.push_back(tmpP1);

			vector<Point3f>::iterator it = pointin.begin();

			while (it != pointin.end())
			{
				Point3f tmpP2 = (*it);
				if (cvgeomerty::distance(Point2f(tmpP1.x, tmpP1.y), Point2f(tmpP2.x, tmpP2.y)) < min_distance)
				{
					groupCorner.push_back(tmpP2);
					it = pointin.erase(it);
					continue;
				}
				it++;
			}

			Point3f tmpPoint1 = groupCorner[0];

			for (size_t i = 0; i < groupCorner.size(); i++)
			{
				Point3f tmpPoint2 = groupCorner[i];
				tmpPoint1 = Point3f((tmpPoint1.x + tmpPoint2.x) / 2., (tmpPoint1.y + tmpPoint2.y) / 2., (tmpPoint1.z + tmpPoint2.z) / 2.);
			}

			while (it != pointin.end())
			{
				Point3f tmpP2 = (*it);
				if (cvgeomerty::distance(Point2f(tmpPoint1.x, tmpPoint1.y), Point2f(tmpP2.x, tmpP2.y)) < min_distance)
				{
					groupCorner.push_back(tmpP2);
					it = pointin.erase(it);
					continue;
				}
				it++;
			}

			finalCorners.push_back(tmpPoint1);
		}
		return finalCorners;
	}

	//基于平面纵向igonre邻近点，保留距离平面远的点
	vector<Point3f> ignoreVertaicalSamePoints(vector<Point3f> pointin, double min_distance, cvgeomerty::LINE planeLine) {
		vector<Point3f> finalCorners;

		while (pointin.size() > 0)
		{
			Point3f tmpP1 = pointin[pointin.size() - 1];

			pointin.pop_back();

			double errorAngle = 90;
			double errorAngleThresh = 28;

			Point2f tmpP1Perpendicular = cvgeomerty::perpendicular(Point2f(tmpP1.x, tmpP1.y), planeLine);

			vector<Point3f>::iterator it = pointin.begin();

			cvgeomerty::LINE vertaicalLine = cvgeomerty::LINE(Vec4f(tmpP1.x, tmpP1.y, tmpP1Perpendicular.x, tmpP1Perpendicular.y));

			while (it != pointin.end())
			{
				Point3f tmpP2 = (*it);

				double tmpDistance = cvgeomerty::pointToLineDistance(Point2f(tmpP2.x, tmpP2.y), vertaicalLine);
				if (tmpDistance < min_distance)
				{

					if (tmpP1.x > tmpP2.x)
					{
						tmpP1 = tmpP2;
					}

					pointin.erase(it);
					it = pointin.begin();
					continue;
				}

				it++;
			}
			finalCorners.push_back(tmpP1);
		}
		return finalCorners;
	}

	vector<PaintPoint> ignoreFinalVertaicalSamePoints(vector<PaintPoint> pointin, double min_distance, cvgeomerty::LINE planeLine) {
		vector<PaintPoint> finalCorners;

		while (pointin.size() > 0)
		{
			PaintPoint tmpP1 = pointin[pointin.size() - 1];

			pointin.pop_back();

			double errorAngle = 90;
			double errorAngleThresh = 28;

			if (abs(tmpP1.angle) < (errorAngle + errorAngleThresh) && abs(tmpP1.angle) > (errorAngle - errorAngleThresh))
			{
				continue;
			}

			Point2f tmpP1Perpendicular = cvgeomerty::perpendicular(Point2f(tmpP1.point.x, tmpP1.point.y), planeLine);

			cvgeomerty::LINE vertaicalLine1 = cvgeomerty::LINE(Vec4f(tmpP1.point.x, tmpP1.point.y, tmpP1Perpendicular.x, tmpP1Perpendicular.y));

			if (cvgeomerty::distance(tmpP1Perpendicular, tmpP1.point) < 1)
			{
				vertaicalLine1 = cvgeomerty::LINE(Vec4f(tmpP1.point.x - 10, tmpP1.point.y, tmpP1Perpendicular.x, tmpP1Perpendicular.y));
			}

			vector<PaintPoint>::iterator it = pointin.begin();

			while (it != pointin.end())
			{
				PaintPoint tmpP2 = (*it);

				Point2f tmpP2Perpendicular = cvgeomerty::perpendicular(Point2f(tmpP2.point.x, tmpP2.point.y), planeLine);

				cvgeomerty::LINE vertaicalLine2 = cvgeomerty::LINE(Vec4f(tmpP2.point.x, tmpP2.point.y, tmpP2Perpendicular.x, tmpP2Perpendicular.y));

				if (cvgeomerty::distance(tmpP2Perpendicular, tmpP2.point) < 1)
				{
					vertaicalLine2 = cvgeomerty::LINE(Vec4f(tmpP2.point.x - 10, tmpP2.point.y, tmpP2Perpendicular.x, tmpP2Perpendicular.y));
				}

				double tmpDistance = cvgeomerty::pointToLineDistance(Point2f(tmpP2.point.x, tmpP2.point.y), vertaicalLine1);

				if (isnan(tmpDistance))//说明tmpP1在平面上
				{
					tmpDistance = cvgeomerty::pointToLineDistance(Point2f(tmpP1.point.x, tmpP1.point.y), vertaicalLine2);
					if (isnan(tmpDistance))//说明tmpP1，tmpP2都在平面上
					{
						tmpDistance = cvgeomerty::distance(tmpP1.point, tmpP2.point);
					}
				}

				if (tmpDistance < min_distance)
				{
					if (tmpP1.point.x > tmpP2.point.x)
					{
						tmpP1 = tmpP2;

						tmpP1Perpendicular = tmpP2Perpendicular;
						vertaicalLine1 = vertaicalLine2;

					}

					pointin.erase(it);
					it = pointin.begin();
					continue;
				}

				it++;
			}
			finalCorners.push_back(tmpP1);
		}
		return finalCorners;
	}
	int findCorner(cv::Mat &skeleton, std::vector<cv::Point2f> &corners) {
		//cv::Mat img_corner = skeleton.clone();
		//cv::cvtColor(img_corner, img_corner,cv::COLOR_GRAY2BGR);
		cv::Mat img_filter = cv::Mat::zeros(skeleton.rows, skeleton.cols, CV_8UC1);
		for (int row = 0; row < skeleton.rows; row++) {
			for (int col = 0; col < skeleton.cols; col++) {
				uchar c = skeleton.at<uchar>(row, col);
				if (c == 255) {
					uchar up0h = skeleton.at<uchar>(row - 2, col);
					uchar dp0h = skeleton.at<uchar>(row + 2, col);
					uchar up1h = skeleton.at<uchar>(row - 1, col - 1);
					uchar dp1h = skeleton.at<uchar>(row + 1, col - 1);
					uchar up2h = skeleton.at<uchar>(row - 1, col + 1);
					uchar dp2h = skeleton.at<uchar>(row + 1, col + 1);

					if ((up0h == 255 || dp0h == 255) && (up1h != 255 || dp1h != 255) && (up2h != 255 || dp2h != 255)) {
						//cv::circle(img_corner, cv::Point2f(col, row), 2, cv::Scalar(0, 0, 255), -1, 8, 0);
						img_filter.at<uchar>(row, col) = 255;
					}
				}
			}
		}
		cv::Mat structureElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(-1, -1));
		dilate(img_filter, img_filter, structureElement, cv::Point(-1, -1), 1);
		std::vector<std::vector<cv::Point>> contours;
		findContours(img_filter, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE, cv::Point(0, 0));
		for (size_t i = 0; i < contours.size(); i++) {
			//drawContours(skeleton_show, contours, -1, Scalar(255,0,0), 2);
			//std::cout << "point size:" << contours[i].size() << std::endl;
			std::vector<cv::Point> pts = contours[i];
			int size = static_cast<int>(pts.size());
			cv::Mat data_pts = cv::Mat(size, 2, CV_64FC1);
			for (int i = 0; i < size; i++) {
				data_pts.at<double>(i, 0) = pts[i].x;
				data_pts.at<double>(i, 1) = pts[i].y;
			}
			cv::PCA pca_analysis(data_pts, cv::Mat(), cv::PCA::DATA_AS_ROW);
			cv::Point cnt = cv::Point(static_cast<int>(pca_analysis.mean.at<double>(0, 0)), static_cast<int>(pca_analysis.mean.at<double>(0, 1)));
			corners.push_back(cnt);
			//circle(img_corner, cnt, 2, cv::Scalar(0, 255, 0), 2, 8, 0);
		}
		//std::cout << "contours size:" << contours.size() << std::endl;
		//cv::imshow("img_feature", img_corner);

		cv::waitKey(0);
		return 0;
	}

	bool onsegment(cv::Point2f pi, cv::Point2f pj, cv::Point2f Q)
	{
		//if ((Q.x - pi.x)*(pj.y - pi.y) == (pj.x - pi.x)*(Q.y - pi.y) && min(pi.x, pj.x) <= Q.x&&Q.x <= max(pi.x, pj.x) && min(pi.y, pj.y) <= Q.y&&Q.y <= max(pi.y, pj.y)) {
		if (min(pi.y, pj.y) < (Q.y - 3) && (Q.y + 3) < max(pi.y, pj.y)) {
			return true;
		}
		else {
			return false;
		}
	}

	bool onsegmentX(cv::Point2f pi, cv::Point2f pj, cv::Point2f Q)
	{
		//if ((Q.x - pi.x)*(pj.y - pi.y) == (pj.x - pi.x)*(Q.y - pi.y) && min(pi.x, pj.x) <= Q.x&&Q.x <= max(pi.x, pj.x) && min(pi.y, pj.y) <= Q.y&&Q.y <= max(pi.y, pj.y)) {
		//if (min(pi.y, pj.y) < (Q.y - 3) && (Q.y + 3) < max(pi.y, pj.y)) {
		if ( (min(pi.x, pj.x) < (Q.x - 10) && (Q.x + 10) < max(pi.x, pj.x))) {
			return true;
		}
		else {
			return false;
		}
	}
	
	int modifyCorner(const std::string &file_path, cvgeomerty::LINE &maxDistanceLine, cv::Point2f &point) {
		cv::Mat src_gray = cv::imread(file_path, cv::IMREAD_GRAYSCALE);
		cv::Mat structureElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 1), cv::Point(-1, -1));
		dilate(src_gray, src_gray, structureElement, cv::Point(-1, -1), 1);
		uchar c = src_gray.at<uchar>(int(point.y), int(point.x));
		threshold(src_gray, src_gray, 0, 255, THRESH_OTSU | THRESH_BINARY);

		//cv::Mat src1 = src_gray.clone();
		//cv::cvtColor(src1, src1, cv::COLOR_GRAY2BGR);
		//circle(src1, tmpPerpendicular, 2, Scalar(255, 0, 0), -1, 8, 0);
		//Point pp;
		int count = 0;
		float tmp_y = 0;
		float tmp_x = 0;
		int gray_level = 50;
		while (c > gray_level) {
			if (count > gray_level) {
				break;
			}
			count++;
			float tmp_y10 = point.y - count;
			float tmp_x10 = (-maxDistanceLine.c - tmp_y10 * maxDistanceLine.b)*1.0 / (maxDistanceLine.a*1.0);
			/**********************/
			//pp.x = tmp_x10;
			//pp.y = tmp_y10;
			//circle(src1, pp, 2, Scalar(0, 0, 255), -1, 8, 0);
			uchar c10 = src_gray.at<uchar>(int(tmp_y10), int(tmp_x10));
			if (c10 < gray_level) {
				float tmp_y11 = point.y - count - 1;
				float tmp_x11 = (-maxDistanceLine.c - tmp_y11 * maxDistanceLine.b)*1.0 / (maxDistanceLine.a*1.0);
				/**********************/
				//pp.x = tmp_x11;
				//pp.y = tmp_y11;
				//circle(src1, pp, 2, Scalar(0, 0, 255), -1, 8, 0);
				uchar c11 = src_gray.at<uchar>(int(tmp_y11), int(tmp_x11));
				float tmp_y12 = point.y - count - 2;
				float tmp_x12 = (-maxDistanceLine.c - tmp_y12 * maxDistanceLine.b)*1.0 / (maxDistanceLine.a*1.0);
				/**********************/
				//pp.x = tmp_x12;
				//pp.y = tmp_y12;
				//circle(src1, pp, 2, Scalar(0, 0, 255), -1, 8, 0);
				uchar c12 = src_gray.at<uchar>(int(tmp_y12), int(tmp_x12));
				float tmp_y13 = point.y - count - 3;
				float tmp_x13 = (-maxDistanceLine.c - tmp_y13 * maxDistanceLine.b)*1.0 / (maxDistanceLine.a*1.0);
				/**********************/
				//pp.x = tmp_x13;
				//pp.y = tmp_y13;
				//circle(src1, pp, 2, Scalar(0, 0, 255), -1, 8, 0);
				uchar c13 = src_gray.at<uchar>(int(tmp_y13), int(tmp_x13));
				//if (c11 == 0 && c12 == 0 && c13 == 0) {
				if (c11 <= gray_level && c12 <= gray_level && c13 <= gray_level) {
					tmp_y = tmp_y11;
					tmp_x = tmp_x11;
					break;
				}
			}

			float tmp_y20 = point.y + count;
			float tmp_x20 = (-maxDistanceLine.c - tmp_y20 * maxDistanceLine.b)*1.0 / (maxDistanceLine.a*1.0);
			/**********************/
			//pp.x = tmp_x20;
			//pp.y = tmp_y20;
			//circle(src1, pp, 2, Scalar(0, 255, 0), -1, 8, 0);
			uchar c20 = src_gray.at<uchar>(int(tmp_y20), int(tmp_x20));
			if (c20 < gray_level) {
				float tmp_y21 = point.y + count + 1;
				float tmp_x21 = (-maxDistanceLine.c - tmp_y21 * maxDistanceLine.b)*1.0 / (maxDistanceLine.a*1.0);
				/**********************/
				//pp.x = tmp_x21;
				//pp.y = tmp_y21;
				//circle(src1, pp, 2, Scalar(0, 255, 0), -1, 8, 0);
				uchar c21 = src_gray.at<uchar>(int(tmp_y21), int(tmp_x21));
				float tmp_y22 = point.y + count + 2;
				float tmp_x22 = (-maxDistanceLine.c - tmp_y22 * maxDistanceLine.b)*1.0 / (maxDistanceLine.a*1.0);
				/**********************/
				//pp.x = tmp_x22;
				//pp.y = tmp_y22;
				//circle(src1, pp, 2, Scalar(0, 255, 0), -1, 8, 0);
				uchar c22 = src_gray.at<uchar>(int(tmp_y22), int(tmp_x22));
				float tmp_y23 = point.y + count + 3;
				float tmp_x23 = (-maxDistanceLine.c - tmp_y23 * maxDistanceLine.b)*1.0 / (maxDistanceLine.a*1.0);
				uchar c23 = src_gray.at<uchar>(int(tmp_y23), int(tmp_x23));
				/**********************/
				//pp.x = tmp_x23;
				//pp.y = tmp_y23;
				//circle(src1, pp, 2, Scalar(0, 255, 0), -1, 8, 0);
				if (c21 <= gray_level && c22 <= gray_level && c23 <= gray_level) {
					tmp_y = tmp_y21;
					tmp_x = tmp_x21;
					break;
				}
			}

		}
		if (!(tmp_x == 0 && tmp_y == 0)) {
			point.x = tmp_x;
			point.y = tmp_y;
		}



#ifdef DEBUG
		//cv::imshow("23", src1);
		//cv::waitKey();
		{
			/*cv::Mat src1 = src_gray.clone();*/
			//cv::cvtColor(src1, src1, cv::COLOR_GRAY2BGR);
			cv::Mat src1 = cv::imread(file_path, cv::IMREAD_COLOR);
			circle(src1, point, 2, Scalar(255, 0, 0), -1, 8, 0);
			cv::imshow("src1", src1);
			cv::waitKey();
		}
#endif
		return 0;
	}
	int modifyLine(const std::string &file_path, cvgeomerty::LINE &Line) {
#ifdef DEBUG
		{
			cv::Mat src1 = cv::imread(file_path, cv::IMREAD_COLOR);
			circle(src1, Line.start, 5, Scalar(255, 0, 0), -1, 8, 0);
			circle(src1, Line.end, 5, Scalar(255, 255, 0), -1, 8, 0);
			cv::imshow("src1", src1);
			cv::waitKey();
		}
#endif
		//处理图像
		cv::Mat src_gray = cv::imread(file_path, cv::IMREAD_GRAYSCALE);
		cv::Mat structureElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(50, 3), cv::Point(-1, -1));
		dilate(src_gray, src_gray, structureElement, cv::Point(-1, -1), 1);
		threshold(src_gray, src_gray, 0, 255, THRESH_OTSU | THRESH_BINARY);

		cvgeomerty::LINE  Line_tmp = Vec4f(Line.start.x, Line.start.y, Line.end.x, Line.end.y);
		int count = 0;
		int gray_level = 50;
		cv::Point2f point = Line.start;
		uchar c = src_gray.at<uchar>(int(point.y), int(point.x));
		while (c < gray_level) {
			count++;
			float tmp_y = point.y - count;
			float tmp_x = (-Line.c - tmp_y * Line.b)*1.0 / (Line.a*1.0);
			if (tmp_x > 500 || tmp_x < 0 || tmp_y > 500 || tmp_y < 0) {
				break;
			}
			uchar tmp_c = src_gray.at<uchar>(int(tmp_y), int(tmp_x));
			if (tmp_c > gray_level) {
				Line.start.x = tmp_x;
				Line.start.y = tmp_y;
				break;
			}
		}

		point = Line.end;
		c = src_gray.at<uchar>(int(point.y), int(point.x));
		while (c < gray_level) {
			count--;
			float tmp_y = point.y - count;
			float tmp_x = (-Line.c - tmp_y * Line.b)*1.0 / (Line.a*1.0);
			if (tmp_x > 500 || tmp_x < 0 || tmp_y > 500 || tmp_y < 0) {
				break;
			}
			uchar tmp_c = src_gray.at<uchar>(int(tmp_y), int(tmp_x));
			if (tmp_c > gray_level) {
				Line.end.x = tmp_x;
				Line.end.y = tmp_y;
				break;
			}
		}
		Line = Vec4f(Line.start.x, Line.start.y, Line.end.x, Line.end.y);
		std::cout << "cvgeomerty::distance(Line.start, Line.end):" << cvgeomerty::distance(Line.start, Line.end) << std::endl;
		if (cvgeomerty::distance(Line.start, Line.end)<20) {
			Line = Vec4f(Line_tmp.start.x, Line_tmp.start.y, Line_tmp.end.x, Line_tmp.end.y);
		}
#ifdef DEBUG
		{
			cv::Mat src1 = cv::imread(file_path, cv::IMREAD_COLOR);
			circle(src1, Line.start, 5, Scalar(255, 0, 0), -1, 8, 0);
			circle(src1, Line.end, 5, Scalar(255, 255, 0), -1, 8, 0);
			cv::imshow("src1", src1);
			cv::waitKey();
		}
#endif
		return 0;
	}
	int findMaxLine() {
		////锯齿状的进行膨胀
		//cv::Mat structureElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 7), cv::Point(-1, -1));
		//erode(src, src, structureElement, cv::Point(-1, -1), 1);
		//dilate(src, src, structureElement, cv::Point(-1, -1), 1);
		//imshow("src", src);
		//waitKey();
		return 0;
	}
	int getPointCloudCrossOriginCornersPre(vector<Point3f> &newCorners, bool &no_plane, cvgeomerty::LINE &maxDistanceLine, const std::string &file_path, double minAngleThresh, double minIntervalThresh, double minDistanceThresh, double novangle, bool skeletonImage) {
		int res = fileExist((char*)file_path.c_str());
		if (res == -1) {
			LOG(ERRORL) << "加载图片错误";
			throw exception((std::string(__FUNCTION__) + ":加载图片错误").c_str());
		}

		Mat src = cv::imread(file_path, cv::IMREAD_GRAYSCALE);

		//提取骨骼
		Mat skeleton = src.clone();
		if (skeletonImage)
		{
			cv::threshold(src, skeleton, 128, 1, cv::THRESH_BINARY);
			skeleton = cvskeleton::thinImage(skeleton);
			skeleton = skeleton * 255;
		}
		//拟合线段
		vector<Vec4f>  lines; //存放检测到的线段的两个端点坐标
		double _scale = 0.8;
		double _sigma_scale = 0.6;
		double _quant = 2.0;
		double _ang_th = 22.5;
		double _log_eps = 0;
		double _density_th = 0.7;
		int _n_bins = 1024;
		Ptr<LineSegmentDetector> ls = createLineSegmentDetector(LSD_REFINE_NONE, _scale, _sigma_scale, _quant, _ang_th, _log_eps, _density_th, _n_bins);
		ls->detect(skeleton, lines);
		//vector<Vec4f>  newlines = fitGroupLine(lines, minAngleThresh, minDistanceThresh, minIntervalThresh);
		vector<Vec4f>  newlines = fitGroupLine(lines, minAngleThresh, 5, 8);
		/*
		if (newlines.size() > 1) {
			std::sort(newlines.begin(), newlines.end(), [](Vec4f &a, const Vec4f &b) {
				cvgeomerty::LINE newa = cvgeomerty::LINE(a);
				cvgeomerty::LINE newb = cvgeomerty::LINE(b);
				double distace1 = cvgeomerty::distance(newa.start, newa.end);
				double distace2 = cvgeomerty::distance(newb.start, newb.end);
				return distace1 > distace2;
			});
		}
		vector<Vec4f>  newlines_tmp(newlines);
		double long_d = 0;
		double long_vangle = 0;
		if (newlines_tmp.size() > 0) {
			vector<Vec4f>::iterator it = newlines_tmp.begin();
			Vec4f long_line = (*it);
			cvgeomerty::LINE long_cv_line = cvgeomerty::LINE(long_line);
			Point2f long_p1 = long_cv_line.start;
			Point2f long_p2 = long_cv_line.end;
			long_d = cvgeomerty::distance(long_p1, long_p2);
			long_vangle = cvgeomerty::lvangle(long_cv_line) * 180 / CV_PI;
			it++;
			std::cout << "************long_vangle:" << long_vangle << std::endl;
			while (it != newlines_tmp.end() && abs(long_vangle) < novangle)
			{
				std::cout << "************long_vangle:" << long_vangle << std::endl;
				Vec4f currentLine = (*it);
				cvgeomerty::LINE currentCVLine = cvgeomerty::LINE(currentLine);

				Point2f p1 = currentCVLine.start;
				Point2f p2 = currentCVLine.end;
				double vangle = cvgeomerty::lvangle(currentCVLine) * 180 / CV_PI;
				double current_d = cvgeomerty::distance(p1, p2);
				std::cout << "*************long_d" << long_d << std::endl;
				std::cout << "*************current_d" << current_d << std::endl;
				std::cout << "*************fabs(long_d- current_d) " << fabs(long_d - current_d) << std::endl;
				if (fabs(long_d- current_d) > 45 && abs(vangle) < 30) {
					it = newlines_tmp.erase(it);
					continue;
				}
				it++;

			}
		}
		vector<Vec4f>  newmaxlines = fitGroupLine(newlines_tmp, 10, 10, 60);
		if (newmaxlines.size() == 0|| long_d < 50) {
			newmaxlines = fitGroupLine(lines, 10, 10, 40);
		}*/
		vector<Vec4f>  newmaxlines = fitGroupLine(lines, 10, 10, 60);

#ifdef DEBUG
		{
			cv::Mat src1 = cv::imread(file_path, cv::IMREAD_COLOR);
			Point2f p1 = plane_line.start;
			Point2f p2 = plane_line.end;
			line(src1, p1, p2, Scalar(rand() % 255, rand() % 255, rand() % 255), 2, LINE_AA);
			imshow("line", src1);
			waitKey(0);
		}

		//Mat drawnNewLines(skeleton);
		//ls->drawSegments(drawnNewLines, lines);
		//imshow("drawnNewLines", drawnNewLines);
		//cv::waitKey();
		//{
		//	cv::Mat src1 = cv::imread(file_path, cv::IMREAD_COLOR);
		//	for (int i = 0; i < newlines.size(); i++) {
		//		Vec4f currentLine = newlines[i];
		//		cvgeomerty::LINE currentCVLine = cvgeomerty::LINE(currentLine);
		//		Point2f p1 = currentCVLine.start;
		//		Point2f p2 = currentCVLine.end;
		//		line(src1, p1, p2, Scalar(rand() % 255, rand() % 255, rand() % 255), 2, LINE_AA);

		//	}
		//	imshow("line", src1);
		//	waitKey(0);
		//}

		//cv::Mat src2 = cv::imread(file_path, cv::IMREAD_COLOR);
		//for (int i = 0; i < newmaxlines.size(); i++) {
		//	Vec4f currentLine = newmaxlines[i];
		//	cvgeomerty::LINE currentCVLine = cvgeomerty::LINE(currentLine);
		//	Point2f p1 = currentCVLine.start;
		//	Point2f p2 = currentCVLine.end;
		//	line(src2, p1, p2, Scalar(rand() % 255, rand() % 255, rand() % 255), 2, LINE_AA);
		//	imshow("line", src2);
		//	waitKey(0);
		//}
#endif
		/*
		cvgeomerty::LINE planeCVLine = cvgeomerty::LINE(plane_line);
		float plane_line_distance = cvgeomerty::distance(planeCVLine.start, planeCVLine.end);
		float plane_line_angle = fabs(cvgeomerty::lvangle(planeCVLine) * 180 / CV_PI);
		vector<Vec4f> newmaxlines_tmp;
		std::cout << "*****************************************************plane_line_distance:" << plane_line_distance << std::endl;
		std::cout << "*****************************************************plane_line_angle:" << plane_line_angle << std::endl;
		//平面直线倾斜,看看是否有竖直直线,且竖直直线大于等于平面直线
		for (int i = 0; i < newmaxlines.size(); i++) {
			Vec4f currentLine = newlines[i];
			cvgeomerty::LINE currentCVLine = cvgeomerty::LINE(currentLine);
			float distance = cvgeomerty::distance(currentCVLine.start, currentCVLine.end);
			float angle = fabs(cvgeomerty::lvangle(currentCVLine) * 180 / CV_PI);
			std::cout << "*****************************************************distance:" << distance << std::endl;
			std::cout << "*****************************************************angle:" << angle << std::endl;
			if (plane_line_angle > novangle&&(fabs(plane_line_distance - distance) < 60|| distance> plane_line_distance)&& angle < novangle) {
				newmaxlines_tmp.push_back(currentLine);
			}
		}
		//若上一步无结果,则平面直线为最长直线
		cvgeomerty::LINE line_tmp;
		if (newmaxlines_tmp.size() == 0) {
			line_tmp = planeCVLine;
		}
		else {
			//若有结果,取x最大的直线
			std::sort(newmaxlines_tmp.begin(), newmaxlines_tmp.end(), [](Vec4f &a, const Vec4f &b) {
				cvgeomerty::LINE newa = cvgeomerty::LINE(a);
				cvgeomerty::LINE newb = cvgeomerty::LINE(b);
				double distace1 = cvgeomerty::distance(newa.start, newa.end);
				double distace2 = cvgeomerty::distance(newb.start, newb.end);
				return newa.start.x > newb.start.x;
			});
			line_tmp = newmaxlines_tmp[0];
		}		
		if (line_tmp.start.y < line_tmp.end.y)
		{
			maxDistanceLine = cvgeomerty::LINE(Vec4f(line_tmp.end.x, line_tmp.end.y, line_tmp.start.x, line_tmp.start.y));
		}
		else {
			maxDistanceLine = line_tmp;
		}*/

		if (newmaxlines.size() > 1) {
			std::sort(newmaxlines.begin(), newmaxlines.end(), [](Vec4f &a, const Vec4f &b) {
				cvgeomerty::LINE newa = cvgeomerty::LINE(a);
				cvgeomerty::LINE newb = cvgeomerty::LINE(b);
				double distace1 = cvgeomerty::distance(newa.start, newa.end);
				double distace2 = cvgeomerty::distance(newb.start, newb.end);
				return distace1 > distace2;
			});
		}
		Vec4f plane_line;
		cvgeomerty::LINE planeCVLine = cvgeomerty::LINE(plane_line);
		if (planeCVLine.start.y < planeCVLine.end.y)
		{
			maxDistanceLine = cvgeomerty::LINE(Vec4f(planeCVLine.end.x, planeCVLine.end.y, planeCVLine.start.x, planeCVLine.start.y));
		}
		else {
			maxDistanceLine = planeCVLine;
		}
		if (planeCVLine.start.x < 5 || planeCVLine.start.y < 5) {
			if (newmaxlines.size() > 0) {
				cvgeomerty::LINE line_tmp = cvgeomerty::LINE(newmaxlines[0]);
				if (line_tmp.start.y < line_tmp.end.y)
				{
					maxDistanceLine = cvgeomerty::LINE(Vec4f(line_tmp.end.x, line_tmp.end.y, line_tmp.start.x, line_tmp.start.y));
				}
				else {
					maxDistanceLine = line_tmp;
				}
			}
			else {
				LOG(ERRORL) << "未找到最长线段";
				throw exception((std::string(__FUNCTION__) + ":未找到最长线段").c_str());
			}
		}
		modifyCorner(file_path, maxDistanceLine, maxDistanceLine.start);
		modifyCorner(file_path, maxDistanceLine, maxDistanceLine.end);

		cvgeomerty::LINE maxDistanceLine2 = maxDistanceLine;
		if (newmaxlines.size() > 1) {
			//找出不是最长直线,最长直线
			for (int i = 0; i < newmaxlines.size(); i++) {
				cvgeomerty::LINE line_tmp = cvgeomerty::LINE(newmaxlines[i]);
				if (line_tmp.start.y < line_tmp.end.y)
				{
					line_tmp = cvgeomerty::LINE(Vec4f(line_tmp.end.x, line_tmp.end.y, line_tmp.start.x, line_tmp.start.y));
				}
				if (cvgeomerty::distance(line_tmp.start, maxDistanceLine.start) >20|| cvgeomerty::distance(line_tmp.end, maxDistanceLine.end) > 20) {
					maxDistanceLine2 = line_tmp;
					break;
				}
			}
		}
#ifdef DEBUG
		//对称直线
		//cvgeomerty::LINE maxDistanceLine2;
		//maxDistanceLine2.start = maxDistanceLine.start;
		//if (plane_line_angle >= 0) {
		//	maxDistanceLine2.end.y = maxDistanceLine.end.y + 2*fabs(maxDistanceLine.start.y - maxDistanceLine.end.y);
		//}
		//else {
		//	maxDistanceLine2.end.y = maxDistanceLine.end.y + 2*fabs(maxDistanceLine.start.y + maxDistanceLine.end.y);
		//}
		//maxDistanceLine2.end.x = maxDistanceLine.end.x;

		//Mat drawnNewLines(skeleton);
		////ls->drawSegments(drawnNewLines, lines);
		//ls->drawSegments(drawnNewLines, newmaxlines);
		//imshow("drawnNewLines", drawnNewLines);
		//cv::waitKey();
		cv::Mat src1 = cv::imread(file_path, cv::IMREAD_COLOR);
		for (int i = 0; i < newlines.size(); i++) {
			Vec4f currentLine = newlines[i];
			cvgeomerty::LINE currentCVLine = cvgeomerty::LINE(currentLine);
			Point2f p1 = currentCVLine.start;
			Point2f p2 = currentCVLine.end;
			line(src1, p1, p2, Scalar(rand() % 255, rand() % 255, rand() % 255), 2, LINE_AA);
		}
		//line(src1, planeCVLine.start, planeCVLine.end, Scalar(0,0, 255), 2, LINE_AA);
		line(src1, maxDistanceLine.start, maxDistanceLine.end, Scalar(0, 0, 255), 2, LINE_AA);
		//line(src1, maxDistanceLine2.start, maxDistanceLine2.end, Scalar(0,255, 0), 2, LINE_AA);
		
		imshow("line", src1);
		waitKey(0);
		//cv::Mat src1 = cv::imread(file_path, cv::IMREAD_COLOR);
		//for (int i = 0; i < newmaxlines.size(); i++) {
		//	Vec4f currentLine = newmaxlines[i];
		//	cvgeomerty::LINE currentCVLine = cvgeomerty::LINE(currentLine);
		//	Point2f p1 = currentCVLine.start;
		//	Point2f p2 = currentCVLine.end;
		//	line(src1, p1, p2, Scalar(rand() % 255, rand() % 255, rand() % 255), 2, LINE_AA);
		//	imshow("line", src1);
		//	waitKey(0);
		//}
		//cv::Mat src1 = cv::imread(file_path, cv::IMREAD_COLOR);
		//Point2f p1 = maxDistanceLine.start;
		//Point2f p2 = maxDistanceLine.end;
		//line(src1, p1, p2, Scalar(rand() % 255, rand() % 255, rand() % 255), 2, LINE_AA);
		//imshow("line", src1);
		//waitKey(0);
#endif 
		double maxlvangle = cvgeomerty::lvangle(maxDistanceLine) * 180 / CV_PI;
		//角点检测
		vector<Point2f> corners;
		vector<Point2f> corners0;
		int max_corners = 100;
		double quality_level = 0.01;
		double min_distance = 10;
		int block_size = 3;
		bool use_harris = false;
		double k = 0.04;
		//cv角点检测	
		cv::goodFeaturesToTrack(src,
			corners0,
			max_corners,
			quality_level,
			min_distance,
			cv::Mat(),
			block_size,
			use_harris,
			k);
		//米字角点检测	
		vector<Point2f> corners1;
		findCorner(skeleton, corners1);


		src = cv::imread(file_path, cv::IMREAD_COLOR);

		newCorners.push_back(Point3f(maxDistanceLine.start.x, maxDistanceLine.start.y, maxlvangle));
		newCorners.push_back(Point3f(maxDistanceLine.end.x, maxDistanceLine.end.y, maxlvangle));
		//过滤角点
		//去除newlines上的角点
		for (int i = 0; i < newlines.size(); ++i)
		{
			Vec4f currentLine = newlines[i];
			cvgeomerty::LINE currentCVLine = cvgeomerty::LINE(currentLine);

			Point2f p1 = currentCVLine.start;
			Point2f p2 = currentCVLine.end;
			double vangle = cvgeomerty::lvangle(currentCVLine) * 180 / CV_PI;

			newCorners.push_back(Point3f(p1.x, p1.y, vangle));
			newCorners.push_back(Point3f(p2.x, p2.y, vangle));

			vector<Point2f>::iterator it0 = corners0.begin();

			while (it0 != corners0.end())
			{
				Point2f tmpPoint = (*it0);
				double tmpDistance = cvgeomerty::pointToLineDistance(tmpPoint, cvgeomerty::LINE(currentLine));
				bool on = onsegmentX(p1, p2, tmpPoint);
				//if (tmpDistance < minIntervalThresh)
				if (tmpDistance < 5)
				//if (on&&tmpDistance < 10)
				{
					it0 = corners0.erase(it0);
					continue;
				}
				it0++;
			}
			vector<Point2f>::iterator it1 = corners1.begin();

			while (it1 != corners1.end())
			{
				Point2f tmpPoint = (*it1);
				bool on = onsegment(p1, p2, tmpPoint);
				double tmpDistance1 = cvgeomerty::pointToLineDistance(tmpPoint, currentLine);
				bool onmax = onsegment(maxDistanceLine.start, maxDistanceLine.end, tmpPoint);
				double tmpDistance = cvgeomerty::pointToLineDistance(tmpPoint, maxDistanceLine);
				double currentLine_d = cvgeomerty::distance(p1,p2);
				if (currentLine_d > 15) {
					if ((on&&tmpDistance1 < 5) || (onmax&&tmpDistance < 5))
					{
						it1 = corners1.erase(it1);
						continue;
					}
				}

				it1++;
			}
			cv::line(src, p1, p2, Scalar(rand() % 255, rand() % 255, rand() % 255), 2, LINE_AA);
		}


		//去除直线上的杂小线段
		for (int i = 0; i < newlines.size(); ++i)//判断角点与线段重合
		{
			Vec4f currentLine = newlines[i];
			cvgeomerty::LINE currentCVLine = cvgeomerty::LINE(currentLine);
			Point2f p1 = maxDistanceLine.start;
			Point2f p2 = maxDistanceLine.end;
			if (cvgeomerty::distance(p1, p2) < 5) {
				continue;
			}

			vector<Point3f>::iterator it = newCorners.begin();

			while (it != newCorners.end())
			{
				Point3f tmpPoint = (*it);
				Point p;
				p.x = tmpPoint.x;
				p.y = tmpPoint.y;
				bool on = onsegment(p1, p2, p);
				//bool onmax = onsegment(maxDistanceLine.start, maxDistanceLine.end, p);
				double tmpDistance = cvgeomerty::pointToLineDistance(p, maxDistanceLine);
				if (on&&tmpDistance < 20)
				{
					it = newCorners.erase(it);
					continue;
				}
				it++;
			}
		}
		//去除线段点 离米字角点较近的点
		for (int i = 0; i < corners1.size(); ++i)//判断角点与线段重合
		{
			vector<Point3f>::iterator it0 = newCorners.begin();
			while (it0 != newCorners.end())
			{
				Point2f tmpPoint;
				tmpPoint.x = (*it0).x;
				tmpPoint.y = (*it0).y;
				double tmpDistance = cvgeomerty::distance(corners1[i], tmpPoint);

				if (tmpDistance < 10)
				{
					it0 = newCorners.erase(it0);
					continue;
				}
				it0++;
			}

		}
#ifdef DEBUGC
		//string file_path_w = getFileName(file_path);
		//file_path_w = saveGroupPath + file_path_w;
		//cv::imwrite(file_path_w.substr(0, file_path_w.length() - 4) + "_line.jpg", src);
		if (WorkFlowPath != "") {
			string file_name = getFileName(file_path);
			imwrite(WorkFlowPath + file_name.substr(0, file_name.length() - 4) + "_line.jpg", src);
		}
#endif
#ifdef DEBUG
		{
			cv::Mat src1 = cv::imread(file_path, cv::IMREAD_COLOR);
			for (int i = 0; i < corners1.size(); i++) {
				Point2f p1;
				p1 = corners1[i];
				circle(src1, p1, 2, Scalar(0, 0, 255), -1, 8, 0);
				corners.push_back(corners1[i]);
			}
			for (int i = 0; i < newCorners.size(); i++) {
				Point2f p1;
				p1.x = newCorners[i].x;
				p1.y = newCorners[i].y;
				circle(src1, p1, 2, Scalar(255, 0, 0), -1, 8, 0);
			}
			for (int i = 0; i < corners0.size(); i++) {
				Point2f p1;
				p1.x = corners0[i].x;
				p1.y = corners0[i].y;
				circle(src1, p1, 2, Scalar(0, 255, 0), -1, 8, 0);
				corners.push_back(corners0[i]);
			}
			cv::imshow("corners", src1);
			cv::waitKey();
		}
#endif
		//判断有无平面
		int inMaxLinePointCount = 0;
		double maxlineMaxY, maxlineMinY;
		if (maxDistanceLine.start.y > maxDistanceLine.end.y)
		{
			maxlineMaxY = maxDistanceLine.start.y;
			maxlineMinY = maxDistanceLine.end.y;
		}
		else {
			maxlineMinY = maxDistanceLine.start.y;
			maxlineMaxY = maxDistanceLine.end.y;
		}
		Point2f midMaxlineP = cvgeomerty::getMidPoint(maxDistanceLine.start, maxDistanceLine.end);
		double moveToLeftMidMaxlinePX = (midMaxlineP.x - 5);//(-leftLineC - maxDistanceLine.b*midMaxlineP.y) / maxDistanceLine.a;
		midMaxlineP.x = moveToLeftMidMaxlinePX;

		vector<Point2f> polygonCorners;
		for (size_t i = 0; i < newCorners.size(); i++)
		{
			Point2f p1;
			p1.x = newCorners[i].x;
			p1.y = newCorners[i].y;
			polygonCorners.push_back(p1);
		}

		for (size_t i = 0; i < corners0.size(); i++)
		{
			polygonCorners.push_back(corners0[i]);
			corners.push_back(corners0[i]);
		}
		for (size_t i = 0; i < corners1.size(); i++)
		{
			polygonCorners.push_back(corners1[i]);
			corners.push_back(corners1[i]);
		}

		if (polygonCorners.size() < 1)
		{
			LOG(ERRORL) << "polygonCorners.size()=0";
			throw exception((std::string(__FUNCTION__) + ":polygonCorners.size()=0").c_str());
			return -1;
		}
		vector<Point2f> convexs;
		cv::convexHull(polygonCorners, convexs, false, true);
		std::cout << "midMaxlinePoint:" << midMaxlineP << std::endl;
		//根据工件截面摆放，最大边中点向虚拟平面外也就是左边移动若干像素在轮廓内说明工件正放
		double pointPolyonVal = pointPolygonTest(convexs, midMaxlineP, true);
		std::cout << "pointPolyonVal:" << pointPolyonVal << std::endl;
		no_plane = pointPolyonVal < 0 && abs(90 - abs(maxDistanceLine.angle)) > novangle;
		std::cout << "*********************************************************no_plane:"<<no_plane << std::endl;
#ifdef DEBUG
		vector<vector<Point>> contours;
		vector<Point> contour;
		for (int i = 0; i < convexs.size(); i++) {
			Point p;
			p.x = convexs[i].x;
			p.y = convexs[i].y;
			contour.push_back(p);
		}
		contours.push_back(contour);
		{
			cv::Mat src1 = cv::imread(file_path, cv::IMREAD_COLOR);
			for (int i = 0; i < convexs.size(); i++) {
				Point2f p1;
				p1.x = convexs[i].x;
				p1.y = convexs[i].y;
				circle(src1, p1, 2, Scalar(255, 0, 0), -1, 8, 0);
			}
			circle(src1, midMaxlineP, 2, Scalar(0, 0, 255), 1, 8, 0);
			vector<Vec4i> empty(0);
			drawContours(src1, contours, 0, Scalar(0, 255, 0), 2, LINE_8, empty, 0, Point(0, 0));

			//RotatedRect minRect = minAreaRect(polygonCorners);
			//Point2f vertex[4];//用于存放最小矩形的四个顶点
			//minRect.points(vertex);
			//line(src1, vertex[0], vertex[1], Scalar(255, 0, 255), 1, LINE_AA);
			//line(src1, vertex[1], vertex[2], Scalar(255, 0, 255), 1, LINE_AA);
			//line(src1, vertex[2], vertex[3], Scalar(255, 0, 255), 1, LINE_AA);
			//line(src1, vertex[3], vertex[0], Scalar(255, 0, 255), 1, LINE_AA);
			cv::imshow("src1", src1);
			cv::waitKey();
		}
#endif
		if (no_plane)
		{
			newCorners.clear();
			//两直线相等
			if (cvgeomerty::distance(maxDistanceLine2.start, maxDistanceLine.start) < 20&&cvgeomerty::distance(maxDistanceLine2.end, maxDistanceLine.end) < 20) {
				Point2f p1 = maxDistanceLine.start;
				Point2f p2 = maxDistanceLine.end;
				double vangle = cvgeomerty::lvangle(maxDistanceLine) * 180 / CV_PI;
				newCorners.push_back(Point3f((p1.x + p2.x) / 2., (p1.y + p2.y) / 2., vangle));
			}
			else {
				vector<Point2f> groupCorner;
				Point2f p1 = maxDistanceLine.start;
				Point2f p2 = maxDistanceLine.end;
				double vangle = cvgeomerty::lvangle(maxDistanceLine) * 180 / CV_PI;
				newCorners.push_back(Point3f((p1.x + p2.x) / 2., (p1.y + p2.y) / 2., vangle));
				if (p1.x < p2.x) {
					groupCorner.push_back(p1);
				}
				else {
					groupCorner.push_back(p2);
				}

				p1 = maxDistanceLine2.start;
				p2 = maxDistanceLine2.end;
				vangle = cvgeomerty::lvangle(maxDistanceLine2) * 180 / CV_PI;
				newCorners.push_back(Point3f((p1.x + p2.x) / 2., (p1.y + p2.y) / 2., vangle));
				if (p1.x < p2.x) {
					groupCorner.push_back(p1);
				}
				else {
					groupCorner.push_back(p2);
				}

				p1 = groupCorner[0];
				p2 = groupCorner[1];
				newCorners.push_back(Point3f((p1.x + p2.x) / 2., (p1.y + p2.y) / 2., 0));
			}


#ifdef DEBUG
			{
				cv::Mat src1 = cv::imread(file_path, cv::IMREAD_COLOR);

				for (int i = 0; i < newCorners.size(); i++) {
					Point2f p1;
					p1.x = newCorners[i].x;
					p1.y = newCorners[i].y;
					circle(src1, p1, 2, Scalar(255, 0, 0), -1, 8, 0);
				}

				for (int i = 0; i < groupCorner.size(); i++) {
					Point2f p1;
					p1.x = groupCorner[i].x;
					p1.y = groupCorner[i].y;
					circle(src1, p1, 2, Scalar(0, 0, 255), -1, 8, 0);
				}
				line(src1, maxDistanceLine.start, maxDistanceLine.end, Scalar(0, 0, 255), 3, LINE_AA);
				line(src1, maxDistanceLine2.start, maxDistanceLine2.end, Scalar(255, 0, 0), 3, LINE_AA);

				//circle(src1, ccs, 2, Scalar(0, 255, 0), 5, 8, 0);
				cv::imshow("noplane", src1);
				cv::waitKey();
			}
#endif
		}
		else {
			for (size_t i = 0; i < corners.size(); i++)
			{
				newCorners.push_back(Point3f(corners[i].x, corners[i].y, maxlvangle));
			}
#ifdef DEBUG
			{
				cv::Mat src1 = cv::imread(file_path, cv::IMREAD_COLOR);

				for (int i = 0; i < newCorners.size(); i++) {
					Point2f p1;
					p1.x = newCorners[i].x;
					p1.y = newCorners[i].y;
					circle(src1, p1, 2, Scalar(255, 0, 0), -1, 8, 0);
				}
				cv::imshow("newCorners", src1);
				cv::waitKey();
			}
#endif
		}
		return 0;
	}
	
	int readPlaneLine1(char* file, Vec4f &plane_line) {
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
	int getPointCloudCrossPaintCorners(const std::string &file_path, vector<PaintPoint>&paintCorners, vector<PaintPoint>&contentPaintCorners, cvgeomerty::LINE &maxDistanceLine,double minAngleThresh, double minIntervalThresh, double minDistanceThresh, double novangle, bool skeletonImage) {
		int res = fileExist((char*)file_path.c_str());
		if (res == -1) {
			LOG(ERRORL) << "加载图片错误";
			throw exception((std::string(__FUNCTION__) + ":加载图片错误").c_str());
		}
		Mat src = cv::imread(file_path, cv::IMREAD_GRAYSCALE);
		//提取骨骼
		Mat skeleton = src.clone();
		if (skeletonImage)
		{
			cv::threshold(src, skeleton, 128, 1, cv::THRESH_BINARY);
			skeleton = cvskeleton::thinImage(skeleton);
			skeleton = skeleton * 255;
		}
		//拟合线段
		vector<Vec4f>  lines; //存放检测到的线段的两个端点坐标
		double _scale = 0.8;
		double _sigma_scale = 0.6;
		double _quant = 2.0;
		double _ang_th = 22.5;
		double _log_eps = 0;
		double _density_th = 0.7;
		int _n_bins = 1024;
		Ptr<LineSegmentDetector> ls = createLineSegmentDetector(LSD_REFINE_NONE, _scale, _sigma_scale, _quant, _ang_th, _log_eps, _density_th, _n_bins);
		ls->detect(skeleton, lines);
		vector<Vec4f>  newlines = fitGroupLine(lines, minAngleThresh, minDistanceThresh, minIntervalThresh);
		vector<Vec4f>  newmaxlines = fitGroupLine(lines, 10, 10, 60);
#ifdef DEBUG
		//{
		//	Vec4f plane_line;
		//	readPlaneLine1((char*) (file_path.substr(0, file_path.length()-3)+"key").c_str(), plane_line);
		//	cvgeomerty::LINE max_line(plane_line);
		//	cv::Mat src1 = cv::imread(file_path, cv::IMREAD_COLOR);
		//	Point2f p1 = max_line.start;
		//	Point2f p2 = max_line.end;
		//	line(src1, p1, p2, Scalar(rand() % 255, rand() % 255, rand() % 255), 2, LINE_AA);
		//	imshow("line", src1);
		//	waitKey(0);
		//}

		//{
		//	cv::Mat src1 = cv::imread(file_path, cv::IMREAD_COLOR);
		//	for (int i = 0; i < newlines.size(); i++) {
		//		Vec4f currentLine = newlines[i];
		//		cvgeomerty::LINE currentCVLine = cvgeomerty::LINE(currentLine);
		//		Point2f p1 = currentCVLine.start;
		//		Point2f p2 = currentCVLine.end;
		//		line(src1, p1, p2, Scalar(rand() % 255, rand() % 255, rand() % 255), 2, LINE_AA);

		//	}
		//	imshow("line", src1);
		//	waitKey(0);
		//}

		cv::Mat src2 = cv::imread(file_path, cv::IMREAD_COLOR);
		for (int i = 0; i < newmaxlines.size(); i++) {
			Vec4f currentLine = newmaxlines[i];
			cvgeomerty::LINE currentCVLine = cvgeomerty::LINE(currentLine);
			Point2f p1 = currentCVLine.start;
			Point2f p2 = currentCVLine.end;
			line(src2, p1, p2, Scalar(rand() % 255, rand() % 255, rand() % 255), 2, LINE_AA);
			imshow("line", src2);
			waitKey(0);
		}
#endif
		//角点检测
		vector<Point2f> corners;
		vector<Point2f> corners0;
		int max_corners = 100;
		double quality_level = 0.01;
		double min_distance = 10;
		int block_size = 3;
		bool use_harris = false;
		double k = 0.04;
		//cv角点检测	
		cv::goodFeaturesToTrack(src,
			corners0,
			max_corners,
			quality_level,
			min_distance,
			cv::Mat(),
			block_size,
			use_harris,
			k);
		//米字角点检测	
		vector<Point2f> corners1;
		//findCorner(skeleton, corners1);
		for (int i = 0; i < newlines.size(); ++i)
		{
			Vec4f currentLine = newlines[i];
			cvgeomerty::LINE currentCVLine = cvgeomerty::LINE(currentLine);
			corners.push_back(currentCVLine.start);
			corners.push_back(currentCVLine.end);
			cv::line(src, currentCVLine.start, currentCVLine.end, Scalar(rand() % 255, rand() % 255, rand() % 255), 2, LINE_AA);

			vector<Point2f>::iterator it0 = corners0.begin();
			while (it0 != corners0.end())
			{
				Point2f tmpPoint = (*it0);
				double tmpDistance = cvgeomerty::pointToLineDistance(tmpPoint, cvgeomerty::LINE(currentLine));
				bool on = onsegment(cvgeomerty::LINE(currentLine).start, cvgeomerty::LINE(currentLine).end, tmpPoint);
				if (on&&tmpDistance < 8)
				{
#ifdef DEBUG
				{
					cv::Mat src1 = cv::imread(file_path, cv::IMREAD_COLOR);
					std::cout << "tmpDistance:" << tmpDistance << std::endl;
					line(src1, cvgeomerty::LINE(currentLine).start, cvgeomerty::LINE(currentLine).end, Scalar(rand() % 255, rand() % 255, rand() % 255), 2, LINE_AA);
					circle(src1, tmpPoint, 5, Scalar(0, 255, 0), -1, 8, 0);
					cv::imshow("corners", src1);
					cv::waitKey();
				}
#endif
					it0 = corners0.erase(it0);
					continue;
				}
				it0++;
			}
			vector<Point2f>::iterator it1 = corners1.begin();
			while (it1 != corners1.end())
			{
				Point2f tmpPoint = (*it1);
				double tmpDistance = cvgeomerty::pointToLineDistance(tmpPoint, currentLine);
				if (tmpDistance < 8)
				{
					it1 = corners1.erase(it1);
					continue;
				}
				it1++;
			}
		}
		for (int i = 0; i < corners0.size(); i++) {
			corners.push_back(corners0[i]);
		}
		for (int i = 0; i < corners1.size(); i++) {
			corners.push_back(corners1[i]);
		}

		Vec4f plane_line;
		readPlaneLine1((char*)(file_path.substr(0, file_path.length() - 3) + "key").c_str(), plane_line);
		cvgeomerty::LINE max_line(plane_line);
		//确保上下直线稳定
		std::cout << "max_line.start.y :" << max_line.start.y << std::endl;
		std::cout << "max_line.end.y :" << max_line.end.y << std::endl;
		if (max_line.start.y < max_line.end.y) {
			max_line = cvgeomerty::LINE(Vec4f(plane_line[2], plane_line[3], plane_line[0], plane_line[1]));
		}
		Point2f max_line_p1 = max_line.start;
		Point2f max_line_p2 = max_line.end;
#ifdef DEBUG
		{
			cv::Mat src1 = cv::imread(file_path, cv::IMREAD_COLOR);
			//line(src1, max_line_p1, max_line_p2, Scalar(rand() % 255, rand() % 255, rand() % 255), 2, LINE_AA);

			//for (int i = 0; i < corners.size(); i++) {
			//	Point2f p1;
			//	p1.x = corners[i].x;
			//	p1.y = corners[i].y;
			//	circle(src1, p1, 2, Scalar(255, 0, 0), -1, 8, 0);
			//}
			//for (int i = 0; i < newlines.size(); i++) {
			//	Vec4f currentLine = newlines[i];
			//	cvgeomerty::LINE currentCVLine = cvgeomerty::LINE(currentLine);
			//	Point2f p1 = currentCVLine.start;
			//	Point2f p2 = currentCVLine.end;
			//	line(src1, p1, p2, Scalar(rand() % 255, rand() % 255, rand() % 255), 2, LINE_AA);

			//}
			//for (int i = 0; i < corners1.size(); i++) {
			//	Point2f p1;
			//	p1 = corners1[i];
			//	circle(src1, p1, 2, Scalar(0, 0, 255), -1, 8, 0);
			//}
			for (int i = 0; i < corners0.size(); i++) {
				Point2f p1;
				p1.x = corners0[i].x;
				p1.y = corners0[i].y;
				circle(src1, p1, 2, Scalar(255, 0, 0), -1, 8, 0);
			}
			cv::imshow("corners", src1);
			cv::waitKey();
		}
#endif
		if (max_line_p1 == max_line_p2) {
			LOG(ERRORL) << "未找到plane_line错误";
			throw exception((std::string(__FUNCTION__) + ":未找到plane_line错误").c_str());
		}

		cvgeomerty::LINE mLineSEG;
		cvgeomerty::LINE leftLineSEG;
		cvgeomerty::LINE rightLineSEG;
		cvgeomerty::LINE upLineSEG;
		cvgeomerty::LINE dLineSEG;
		float k_line = max_line.k;
		if (_isnanf(k_line)) {
			mLineSEG = Vec4f(max_line_p1.x, 0, max_line_p1.x, src.rows);
			leftLineSEG = mLineSEG;
			rightLineSEG = mLineSEG;
		}
		else {
			float m_x1 = 0;
			float m_y1 = k_line * m_x1 - k_line * max_line_p2.x + max_line_p2.y;
			float m_x2 = src.rows;
			float m_y2 = k_line * m_x2 - k_line * max_line_p2.x + max_line_p2.y;
			mLineSEG = Vec4f(m_x1, m_y1, m_x2, m_y2);
			leftLineSEG = mLineSEG;
			rightLineSEG = mLineSEG;
		}

		
		k_line = -1/k_line;
		if (_isnanf(k_line)) {
			upLineSEG = Vec4f(max_line_p1.x, 0, max_line_p1.x, src.rows);
			dLineSEG = Vec4f(max_line_p2.x, 0, max_line_p2.x, src.rows);
		}
		else {
			float up_x1 = 0;
			float up_y1 = k_line * up_x1 - k_line * max_line_p1.x + max_line_p1.y;
			float up_x2 = src.cols;
			float up_y2 = k_line * up_x2 - k_line * max_line_p1.x + max_line_p1.y;
			upLineSEG = Vec4f(up_x1, up_y1, up_x2, up_y2);

			float d_x1 = 0;
			float d_y1 = k_line * d_x1 - k_line * max_line_p2.x + max_line_p2.y;
			float d_x2 = src.cols;
			float d_y2 = k_line * d_x2 - k_line * max_line_p2.x + max_line_p2.y;
			dLineSEG = Vec4f(d_x1, d_y1, d_x2, d_y2);
		}
		
#ifdef DEBUG
		//mLineSEG = Vec4f(max_line_p1.x, 0, max_line_p1.x, src.rows);
		//leftLineSEG = mLineSEG;
		//rightLineSEG = mLineSEG;
		//upLineSEG = Vec4f(max_line_p1.x, 0, max_line_p1.x, src.rows);
		//dLineSEG = Vec4f(max_line_p2.x, 0, max_line_p2.x, src.rows);
		{
			cv::Mat src1 = cv::imread(file_path, cv::IMREAD_COLOR);
			cv::line(src1, dLineSEG.start, dLineSEG.end, Scalar(0, 0, 255), 2, LINE_AA);
			cv::line(src1, upLineSEG.start, upLineSEG.end, Scalar(255, 0, 255), 2, LINE_AA);
			cv::line(src1, rightLineSEG.start, rightLineSEG.end, Scalar(0, 255, 255), 2, LINE_AA);
			cv::line(src1, leftLineSEG.start, leftLineSEG.end, Scalar(0, 255, 255), 2, LINE_AA);
			//circle(src1, max_line.start, 5, Scalar(0, 0, 255), -1, 8, 0);
			//circle(src1, upLineSEG.start, 5, Scalar(0, 255, 255), -1, 8, 0);
			cv::imshow("corners", src1);
			cv::waitKey();
		}
#endif

		vector<Point2f> leftCorners;
		vector<Point2f> rightCorners;
		vector<Point2f> upCorners;
		vector<Point2f> dCorners;
		//分离点
		for (size_t i = 0; i < corners.size(); i++)
		{
			//分离|||左侧 平面 右侧  
			if (cvgeomerty::multiply(mLineSEG.start, mLineSEG.end, corners[i]) >= 0)//表示在正面左侧
			{
				leftCorners.push_back(corners[i]);
			}
			else {
				rightCorners.push_back(corners[i]);
			}
			//分离=上侧
			if (cvgeomerty::multiply(upLineSEG.start, upLineSEG.end, corners[i]) >= 0)
			{
				upCorners.push_back(corners[i]);
			}
			//分离=下侧
			if (cvgeomerty::multiply(dLineSEG.start, dLineSEG.end, corners[i]) <= 0)
			{
				dCorners.push_back(corners[i]);
			}
		}
		//找最左边点
		std::sort(leftCorners.begin(), leftCorners.end(), [&](const Point2f &a, const Point2f &b) {
			double distance1 = cvgeomerty::pointToLineDistance(a, mLineSEG);
			double distance2 = cvgeomerty::pointToLineDistance(b, mLineSEG);
			return distance1 > distance2;
		});
		if (leftCorners.size() > 0) {
			Point2f p = leftCorners[0];
			float k = leftLineSEG.k;
			float x1 = 0;
			float y1 = k * (x1)-k * p.x + p.y;
			float x2 = src.rows;
			float y2 = k * (x2)-k * p.x + p.y;
			leftLineSEG = Vec4f(x1, y1, x2, y2);

		}
		//找最右边点
		std::sort(rightCorners.begin(), rightCorners.end(), [&](const Point2f &a, const Point2f &b) {
			double distance1 = cvgeomerty::pointToLineDistance(a, mLineSEG);
			double distance2 = cvgeomerty::pointToLineDistance(b, mLineSEG);
			return distance1 > distance2;
		});
		if (rightCorners.size()>0) {
			Point2f p = rightCorners[0];
			float k = rightLineSEG.k;
			float x1 = 0;
			float y1 = k * (x1) - k * p.x + p.y;
			float x2 = src.rows;
			float y2 = k * (x2) - k * p.x + p.y;
			rightLineSEG = Vec4f(x1, y1, x2, y2);
		}
		//找最上边点
		std::sort(upCorners.begin(), upCorners.end(), [&](const Point2f &a, const Point2f &b) {
			double distance1 = cvgeomerty::pointToLineDistance(a, upLineSEG);
			double distance2 = cvgeomerty::pointToLineDistance(b, upLineSEG);
			return distance1 > distance2;
		});
		if (upCorners.size() > 0) {
			Point2f p = upCorners[0];
			float k = upLineSEG.k;
			float x1 = 0;
			float y1 = k * (x1)-k * p.x + p.y;
			float x2 = src.rows;
			float y2 = k * (x2)-k * p.x + p.y;
			upLineSEG = Vec4f(x1, y1, x2, y2);
		}
		//找最下边点
		std::sort(dCorners.begin(), dCorners.end(), [&](const Point2f &a, const Point2f &b) {
			double distance1 = cvgeomerty::pointToLineDistance(a, dLineSEG);
			double distance2 = cvgeomerty::pointToLineDistance(b,dLineSEG);
			return distance1 > distance2;
		});
		if (dCorners.size() > 0) {
			Point2f p = dCorners[0];
			float k = dLineSEG.k;
			float x1 = 0;
			float y1 = k * (x1)-k * p.x + p.y;
			float x2 = src.rows;
			float y2 = k * (x2)-k * p.x + p.y;
			dLineSEG = Vec4f(x1, y1, x2, y2);
		}

		//求直线的交点
		vector<Point>crosss;
		crosss.push_back(cvgeomerty::getCrossPoint(leftLineSEG, dLineSEG));
		crosss.push_back(cvgeomerty::getCrossPoint(rightLineSEG, dLineSEG));
		crosss.push_back(cvgeomerty::getCrossPoint(rightLineSEG, upLineSEG));
		crosss.push_back(cvgeomerty::getCrossPoint(leftLineSEG,upLineSEG));
		if (crosss[0] == Point(0, 0) || crosss[0] == Point(0, 0) || crosss[0] == Point(0, 0) || crosss[0] == Point(0, 0)) {
			crosss[0] = max_line.start;
			crosss[1] = max_line.end;
			crosss[2] = max_line.end;
			crosss[3] = max_line.start;
		}

		//矩形最左边点
		int index = 0;
		Point2f left_cross;
		left_cross = crosss[0];
		for (int i = 1; i < crosss.size(); i++) {
			if (left_cross.x > crosss[i].x) {
				left_cross = crosss[i];
				index = i;
			}
		}
		//离最左边点最近的线段为左线段
		float d_tmp1 = cvgeomerty::pointToLineDistance(left_cross, leftLineSEG);
		float d_tmp2 = cvgeomerty::pointToLineDistance(left_cross, rightLineSEG);
		std::cout << "d_tmp1:" << d_tmp1 << std::endl;
		std::cout << "d_tmp2:" << d_tmp2 << std::endl;
		if (d_tmp1 > d_tmp2) {
			//cvgeomerty::LINE tmpline = leftLineSEG;
			//leftLineSEG = rightLineSEG;
			//rightLineSEG = tmpline;
			Point tmp;
			tmp = crosss[0];
			crosss[0] = crosss[3];
			crosss[3] = tmp;
			tmp = crosss[1];
			crosss[1] = crosss[2];
			crosss[2] = tmp;
			index = 0;
			left_cross;
			left_cross = crosss[0];
			for (int i = 1; i < crosss.size(); i++) {
				if (left_cross.x > crosss[i].x) {
					left_cross = crosss[i];
					index = i;
				}
			}
		}
		//从最左端点顺时针排序
		vector<Point>crosss_tmp;
		for (int i = 0; i < 4; i++) {
			crosss_tmp.push_back(crosss[index++]);
			if (index > 3) {
				index = 0;
			}
		}
		//与y轴最近的线段为正对线段
		std::cout << "fabs(leftLineSEG.angle):" << fabs(leftLineSEG.angle) << std::endl;
		std::cout << "fabs(dLineSEG.angle):" << fabs(dLineSEG.angle) << std::endl;
		cvgeomerty::LINE line1 = leftLineSEG;
		if (fabs(leftLineSEG.angle) < fabs(dLineSEG.angle)) {
			line1 = dLineSEG;
		}
		//|1 2|-->|0 1|
		//|0 3|-->|3 2|
		cvgeomerty::LINE line2 = Vec4f(crosss_tmp[0].x, crosss_tmp[0].y, crosss_tmp[1].x, crosss_tmp[1].y);
		std::cout << "fabs(line1.angle):" << fabs(line1.angle) << std::endl;
		std::cout << "fabs(line2.angle):" << fabs(line2.angle) << std::endl;
		std::cout << "fabs(fabs(line2.angle)- fabs(line1.angle)):" << fabs(fabs(line2.angle) - fabs(line1.angle)) << std::endl;
		std::cout << "crosss_tmp[0].y :" << crosss_tmp[0].y  << std::endl;
		std::cout << "crosss_tmp[3].y :" << crosss_tmp[3].y  << std::endl;
		crosss.clear();
		if (crosss_tmp[0].y > crosss_tmp[1].y&&fabs(fabs(line2.angle)- fabs(line1.angle))<5) {
			index = 1;
			for (int i = 0; i < 4; i++) {
				crosss.push_back(crosss_tmp[index++]);
				if (index > 3) {
					index = 0;
				}
			}
		}
		else {
			crosss = crosss_tmp;
		}
		//||最左边
		leftLineSEG = cvgeomerty::LINE(Vec4f(crosss[0].x, crosss[0].y, crosss[3].x, crosss[3].y));
		dLineSEG = cvgeomerty::LINE(Vec4f(crosss[0].x, crosss[0].y, crosss[1].x, crosss[1].y));
		upLineSEG = cvgeomerty::LINE(Vec4f(crosss[3].x, crosss[3].y, crosss[2].x, crosss[2].y));
		rightLineSEG = cvgeomerty::LINE(Vec4f(crosss[2].x, crosss[2].y, crosss[1].x, crosss[1].y));

		//cvgeomerty::LINE real_leftLineSEG;


		
		//选取离左右直线近的点
		vector<Point2f>::iterator it0 = corners.begin();
		while (it0 != corners.end())
		{
			Point2f tmpPoint = (*it0);
			
			double leftDistance = cvgeomerty::pointToLineDistance(tmpPoint, leftLineSEG);
			bool left_on = onsegment(leftLineSEG.start, leftLineSEG.end, tmpPoint);
			double rightDistance = cvgeomerty::pointToLineDistance(tmpPoint, rightLineSEG);
			bool right_on = onsegment(rightLineSEG.start, rightLineSEG.end, tmpPoint);
			double upDistance = cvgeomerty::pointToLineDistance(tmpPoint,upLineSEG);
			bool up_on = onsegmentX(upLineSEG.start, upLineSEG.end, tmpPoint);
			double dDistance = cvgeomerty::pointToLineDistance(tmpPoint, dLineSEG);
			bool d_on = onsegmentX(dLineSEG.start, dLineSEG.end, tmpPoint);
			double maxDistance = cvgeomerty::pointToLineDistance(tmpPoint, max_line);
			bool max_on = onsegment(max_line.start, max_line.end, tmpPoint);
			if ((leftDistance > 20 && rightDistance > 20 && upDistance > 10 && dDistance > 10 && left_on&&right_on&&up_on&&d_on ) || (maxDistance < 15 && max_on))
			{
				it0 = corners.erase(it0);
				continue;
			}
			it0++;
		}
		corners.push_back(max_line.start);
		corners.push_back(max_line.end);
		vector<Point2f> convexs;
		cv::convexHull(corners, convexs, false, true);

#ifdef DEBUG
		vector<vector<Point>> contours;
		vector<Point> contour;
		for (int i = 0; i < convexs.size(); i++) {
			Point p;
			p.x = convexs[i].x;
			p.y = convexs[i].y;
			contour.push_back(p);
		}
		contours.push_back(contour);
		{
			cv::Mat src1 = cv::imread(file_path, cv::IMREAD_COLOR);
			for (int i = 0; i < convexs.size(); i++) {
				Point2f p1;
				p1.x = convexs[i].x;
				p1.y = convexs[i].y;
				circle(src1, p1, 5, Scalar(0, 255, 0), -1, 8, 0);
			}
			vector<Vec4i> empty(0);
			//drawContours(src1, contours, 0, Scalar(0, 255, 0), 2, LINE_8, empty, 0, Point(0, 0));


			cv::line(src1, leftLineSEG.start, leftLineSEG.end, Scalar(255, 0, 0), 2, LINE_AA);
			cv::line(src1, dLineSEG.start, dLineSEG.end, Scalar(0, 0, 255), 2, LINE_AA);
			cv::line(src1, upLineSEG.start, upLineSEG.end, Scalar(255, 0, 255), 2, LINE_AA);
			cv::imshow("src1", src1);
			cv::waitKey();
		}
#endif

		double angle_vertical = dLineSEG.angle;
		double angle_Horizon = leftLineSEG.angle;
		std::cout << "angle_vertical:" << angle_vertical << std::endl;
		std::cout<<"angle_Horizon:" << angle_Horizon << std::endl;
		//left
		vector<Point3f> left_corners;
		for (size_t i = 0; i < corners.size(); i++)
		{
			Point2f perpendicular = cvgeomerty::perpendicular(corners[i], leftLineSEG);
			left_corners.push_back(Point3f(perpendicular.x, perpendicular.y, angle_vertical));
		}
		vector<Point3f> filter_left_corners = ignorePoints(left_corners, 20, true);
#ifdef DEBUG
		{
			cv::Mat src1 = cv::imread(file_path, cv::IMREAD_COLOR);
			//for (int i = 0; i < corners.size(); i++) {
			//	circle(src1, Point2f(corners[i].x, corners[i].y), 3, Scalar(255, 0, 0), -1, 8, 0);
			//}
			for (int i = 0; i < filter_left_corners.size(); i++) {
				circle(src1, Point2f(filter_left_corners[i].x,filter_left_corners[i].y), 5, Scalar(255, 0, 0), -1, 8, 0);
			}
			cv::imshow("corners", src1);
			cv::waitKey();
		}
#endif
		//按照y值由小到大排序
		std::sort(filter_left_corners.begin(), filter_left_corners.end(), [](const Point3f &a, const Point3f &b) {
			return a.y > b.y;
		});
		
		if (leftLineSEG.start.y < leftLineSEG.end.y) {
			leftLineSEG = cvgeomerty::LINE(Vec4f(leftLineSEG.end.x, leftLineSEG.end.y, leftLineSEG.start.x, leftLineSEG.start.y));
		}
		modifyLine(file_path, leftLineSEG);
		maxDistanceLine = cvgeomerty::LINE(Vec4f(leftLineSEG.start.x, leftLineSEG.start.y, leftLineSEG.end.x, leftLineSEG.end.y));
		//vector<PaintPoint> paintCorners;
		for (size_t i = 0; i < filter_left_corners.size()-1; i++)
		{
			Point2f p0 = Point2f(filter_left_corners[i].x, filter_left_corners[i].y);
			Point2f p1 = Point2f(filter_left_corners[i+1].x, filter_left_corners[i+1].y);
			double d = cvgeomerty::distance(p0, p1);
			if (d > 50) {
				Point2f point_tmp = cvgeomerty::getMidPoint(p0, p1);
				paintCorners.push_back(PaintPoint(point_tmp, angle_vertical, false));
				bool on = onsegment(maxDistanceLine.start, maxDistanceLine.end, point_tmp);
				if (on) {
					contentPaintCorners.push_back(PaintPoint(point_tmp, angle_vertical, false));
				}
				
			}
		}



		if (dLineSEG.start.y < dLineSEG.end.y) {
			dLineSEG = cvgeomerty::LINE(Vec4f(dLineSEG.end.x, dLineSEG.end.y, dLineSEG.start.x, dLineSEG.start.y));
		}
		if (upLineSEG.start.y < upLineSEG.end.y) {
			upLineSEG = cvgeomerty::LINE(Vec4f(upLineSEG.end.x, upLineSEG.end.y, upLineSEG.start.x, upLineSEG.start.y));
		}
		if (dLineSEG.angle < -20&& cvgeomerty::distance(dLineSEG.start, dLineSEG.end)>20) {
			Point2f point_tmp = cvgeomerty::getMidPoint(leftLineSEG.end, dLineSEG.end);
			paintCorners.push_back(PaintPoint(point_tmp, 0, false));
		}
		if (dLineSEG.angle > 20 && cvgeomerty::distance(upLineSEG.start, upLineSEG.end) > 20) {
			Point2f point_tmp = cvgeomerty::getMidPoint(leftLineSEG.start, upLineSEG.start);
			paintCorners.push_back(PaintPoint(point_tmp, 0, false));
		}

#ifdef DEBUGC
		{
			cv::Mat src_paint = cv::imread(file_path, cv::IMREAD_COLOR);
			cv::line(src_paint, leftLineSEG.start, leftLineSEG.end, Scalar(255, 0, 0), 2, LINE_AA);
			for (size_t i = 0; i < paintCorners.size(); i++)
			{
				if (paintCorners[i].needChange)
				{
					circle(src_paint, paintCorners[i].point, 5, Scalar(0, 255, 0), 2, 8, 0);
				}
				else {
					circle(src_paint, paintCorners[i].point, 5, Scalar(0, 0, 255), 2, 8, 0);
				}
				double arrowWidth = 30;
				arrowedLine(src_paint, paintCorners[i].point, Point2f(paintCorners[i].point.x - arrowWidth, paintCorners[i].point.y - (arrowWidth)* tan(paintCorners[i].angle / 180 * CV_PI)), Scalar(0, 0, 255), 2, LINE_AA, 0, 0.3);
			}
			string file_name = getFileName(file_path);
			cv::imwrite(WorkFlowPath + file_name.substr(0, file_name.length() - 4) + "_paint.jpg", src_paint);
		}

#endif
#ifdef DEBUG
		{
			cv::Mat src1 = cv::imread(file_path, cv::IMREAD_COLOR);
			cv::line(src1, leftLineSEG.start, leftLineSEG.end, Scalar(255, 0, 0), 2, LINE_AA);
			//cv::line(src1, dLineSEG.start, dLineSEG.end, Scalar(0, 0, 255), 2, LINE_AA);
			//cv::line(src1, upLineSEG.start, upLineSEG.end, Scalar(255, 0, 255), 2, LINE_AA);
			//cv::line(src1, rightLineSEG.start, rightLineSEG.end, Scalar(0, 255, 255), 2, LINE_AA);
			//cv::line(src1, max_line.start, max_line.end, Scalar(0, 255, 255), 2, LINE_AA);
			circle(src1, max_line.start, 5, Scalar(0, 0, 255), -1, 8, 0);
			circle(src1, upLineSEG.start, 5, Scalar(0, 255, 255), -1, 8, 0);
			for (int i = 0; i < convexs.size(); i++) {
				circle(src1, convexs[i], 3, Scalar(255, 0, 0), -1, 8, 0);
			}
			for (int i = 0; i < contentPaintCorners.size();i++) {
				circle(src1, contentPaintCorners[i].point, 10, Scalar(0, 255, 0), 2, 8, 0);
			}

			for (size_t i = 0; i < paintCorners.size(); i++)
			{
				if (paintCorners[i].needChange)
				{
					circle(src1, paintCorners[i].point, 5, Scalar(0, 255, 0), 2, 8, 0);
				}
				else {
					circle(src1, paintCorners[i].point, 5, Scalar(0, 0, 255), 2, 8, 0);
				}
				double arrowWidth = 30;
				arrowedLine(src1, paintCorners[i].point, Point2f(paintCorners[i].point.x - arrowWidth, paintCorners[i].point.y - (arrowWidth)* tan(paintCorners[i].angle / 180 * CV_PI)), Scalar(0, 0, 255), 2, LINE_AA, 0, 0.3);
			}

			cv::imshow("corners", src1);
			cv::waitKey();
		}
#endif
		return 0;
	}
	int getPointCloudCrossOriginCorners(vector<Point3f> &newCorners, bool &no_plane, cvgeomerty::LINE &maxDistanceLine, const std::string &file_path, double minAngleThresh, double minIntervalThresh, double minDistanceThresh, double planeWidthThresh, double novangle, double minPointDistanceThresh, bool skeletonImage) {
		int res = getPointCloudCrossOriginCornersPre(newCorners, no_plane, maxDistanceLine, file_path, minAngleThresh, minIntervalThresh, minDistanceThresh, novangle, skeletonImage);
		if (res == -1) {
			return -1;
		}
		res =  getPointCloudCrossclassifyCorners(newCorners, no_plane,maxDistanceLine,file_path,  minDistanceThresh,  planeWidthThresh,  novangle,  minPointDistanceThresh, skeletonImage);
		if (res == -1) {
			return -1;
		}
		return 0;
	}
	int getPointCloudCrossclassifyCorners(vector<Point3f> &newCorners, bool &no_plane, cvgeomerty::LINE &maxDistanceLine, const std::string &file_path, double minDistanceThresh, double planeWidthThresh, double novangle, double minPointDistanceThresh, bool skeletonImage) {
		int res = fileExist((char*)file_path.c_str());
		if (res == -1) {
			LOG(ERRORL) << "加载图片错误";
			throw exception((std::string(__FUNCTION__) + ":加载图片错误").c_str());
		}
		Mat src = cv::imread(file_path, cv::IMREAD_COLOR);

		double maxlvangle = cvgeomerty::lvangle(maxDistanceLine) * 180 / CV_PI;

		vector<Point3f> rightCorners;//正面以下的点
		vector<Point3f> planeCorners;//正面
		vector<Point3f> leftCorners;//正面以上的点

		//如果no_plane不需处理
		cvgeomerty::LINE leftLineSEG = Vec4f((-(maxDistanceLine.c + maxDistanceLine.a*planeWidthThresh) - maxDistanceLine.b * 0) / maxDistanceLine.a, 0, (-(maxDistanceLine.c + maxDistanceLine.a*planeWidthThresh) - maxDistanceLine.b * 500) / maxDistanceLine.a, src.cols);
		cvgeomerty::LINE rightLineSEG = Vec4f((-(maxDistanceLine.c - maxDistanceLine.a*planeWidthThresh) - maxDistanceLine.b * 0) / maxDistanceLine.a, 0, (-(maxDistanceLine.c - maxDistanceLine.a*planeWidthThresh) - maxDistanceLine.b * 500) / maxDistanceLine.a, src.cols);
		if (!no_plane) {

			//分离|||左侧 平面 右侧  
			for (size_t i = 0; i < newCorners.size(); i++)
			{
				Point3f newSubCorner = newCorners[i];


				if (cvgeomerty::multiply(leftLineSEG.start, leftLineSEG.end, Point2f(newSubCorner.x, newSubCorner.y)) > 0)//表示在正面左侧
				{
					leftCorners.push_back(newSubCorner);
				}
				else {


					if (cvgeomerty::multiply(rightLineSEG.start, rightLineSEG.end, Point2f(newSubCorner.x, newSubCorner.y)) < 0)
					{
						rightCorners.push_back(newSubCorner);
					}
					else {
						planeCorners.push_back(newSubCorner);
					}
				}
			}

			//滤掉在长直线上的平面点
			vector<Point3f>::iterator it = planeCorners.begin();

			while (it != planeCorners.end())
			{
				Point3f tmpPoint = (*it);
				Point p;
				p.x = tmpPoint.x;
				p.y = tmpPoint.y;
				bool onmax = onsegment(maxDistanceLine.start, maxDistanceLine.end, p);
				double tmpDistance = cvgeomerty::pointToLineDistance(p, maxDistanceLine);
				if (onmax&&tmpDistance < 5)
				{
					it = planeCorners.erase(it);
					continue;
				}
				it++;
			}
#ifdef DEBUG
			float minx = 600;
			int index = 0;
			for (size_t i = 0; i < leftCorners.size(); i++)
			{
				if (leftCorners[i].x < minx) {
					minx = leftCorners[i].x;
					index = i;
				}
			}
			{
				cv::Mat src1 = cv::imread(file_path, cv::IMREAD_COLOR);
				for (int i = 0; i < leftCorners.size(); i++) {
					Point2f p1;
					p1.x = leftCorners[i].x;
					p1.y = leftCorners[i].y;
					circle(src1, p1, 2, Scalar(255, 0, 0), -1, 8, 0);
				}
				for (int i = 0; i < rightCorners.size(); i++) {
					Point2f p1;
					p1.x = rightCorners[i].x;
					p1.y = rightCorners[i].y;
					circle(src1, p1, 2, Scalar(0, 0, 255), -1, 8, 0);
				}
				for (int i = 0; i < planeCorners.size(); i++) {
					Point2f p1;
					p1.x = planeCorners[i].x;
					p1.y = planeCorners[i].y;
					circle(src1, p1, 2, Scalar(0, 255, 0), -1, 8, 0);
				}
				if (leftCorners.size() > 0) {
					Point2f p1;
					p1.x = leftCorners[index].x;
					p1.y = leftCorners[index].y;
					//circle(src1, p1, 2, Scalar(0, 255, 0), 3, 8, 0);
					Point2f midMaxlineP = cvgeomerty::getMidPoint(maxDistanceLine.start, maxDistanceLine.end);
					float d = fabs(midMaxlineP.x - leftCorners[index].x);
					double leftLineCtt = maxDistanceLine.b + maxDistanceLine.c + maxDistanceLine.a*(d);//正面
					//line(src1, Point2f((-leftLineCtt - maxDistanceLine.b * 0) / maxDistanceLine.a, 0), Point2f((-leftLineCtt - maxDistanceLine.b * 500) / maxDistanceLine.a, 500), Scalar(0, 0, 255), 3, LINE_AA);


				}
				cv::imshow("src1", src1);
				cv::waitKey();
			}
#endif

			newCorners.clear();
			vector<Point3f> filterRightCorners = rightCorners;

			for (size_t i = 0; i < filterRightCorners.size(); i++)
			{
				Point3f subFilterRightCorner = filterRightCorners[i];

				Point2f tmpPerpendicular = cvgeomerty::perpendicular(Point2f(subFilterRightCorner.x, subFilterRightCorner.y), maxDistanceLine);

				cv::Mat src_gray = cv::imread(file_path, cv::IMREAD_GRAYSCALE);
				cv::Mat structureElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 1), cv::Point(-1, -1));
				dilate(src_gray, src_gray, structureElement, cv::Point(-1, -1), 1);
				uchar c = src_gray.at<uchar>(int(tmpPerpendicular.y), int(tmpPerpendicular.x));
				threshold(src_gray, src_gray, 0, 255, THRESH_OTSU | THRESH_BINARY);

				//cv::Mat src1 = src_gray.clone();
				//cv::cvtColor(src1, src1, cv::COLOR_GRAY2BGR);
				//circle(src1, tmpPerpendicular, 2, Scalar(255, 0, 0), -1, 8, 0);
				//Point pp;
				int count = 0;
				float tmp_y = 0;
				float tmp_x = 0;
				int gray_level = 50;
				while (c > gray_level) {
					if (count > gray_level) {
						break;
					}
					count++;
					float tmp_y10 = tmpPerpendicular.y - count;
					float tmp_x10 = (-maxDistanceLine.c - tmp_y10 * maxDistanceLine.b)*1.0 / (maxDistanceLine.a*1.0);
					/**********************/
					//pp.x = tmp_x10;
					//pp.y = tmp_y10;
					//circle(src1, pp, 2, Scalar(0, 0, 255), -1, 8, 0);
					uchar c10 = src_gray.at<uchar>(int(tmp_y10), int(tmp_x10));
					if (c10 < gray_level) {
						float tmp_y11 = tmpPerpendicular.y - count - 1;
						float tmp_x11 = (-maxDistanceLine.c - tmp_y11 * maxDistanceLine.b)*1.0 / (maxDistanceLine.a*1.0);
						/**********************/
						//pp.x = tmp_x11;
						//pp.y = tmp_y11;
						//circle(src1, pp, 2, Scalar(0, 0, 255), -1, 8, 0);
						uchar c11 = src_gray.at<uchar>(int(tmp_y11), int(tmp_x11));
						float tmp_y12 = tmpPerpendicular.y - count - 2;
						float tmp_x12 = (-maxDistanceLine.c - tmp_y12 * maxDistanceLine.b)*1.0 / (maxDistanceLine.a*1.0);
						/**********************/
						//pp.x = tmp_x12;
						//pp.y = tmp_y12;
						//circle(src1, pp, 2, Scalar(0, 0, 255), -1, 8, 0);
						uchar c12 = src_gray.at<uchar>(int(tmp_y12), int(tmp_x12));
						float tmp_y13 = tmpPerpendicular.y - count - 3;
						float tmp_x13 = (-maxDistanceLine.c - tmp_y13 * maxDistanceLine.b)*1.0 / (maxDistanceLine.a*1.0);
						/**********************/
						//pp.x = tmp_x13;
						//pp.y = tmp_y13;
						//circle(src1, pp, 2, Scalar(0, 0, 255), -1, 8, 0);
						uchar c13 = src_gray.at<uchar>(int(tmp_y13), int(tmp_x13));
						//if (c11 == 0 && c12 == 0 && c13 == 0) {
						if (c11 <= gray_level && c12 <= gray_level && c13 <= gray_level) {
							tmp_y = tmp_y11;
							tmp_x = tmp_x11;
							break;
						}
					}

					float tmp_y20 = tmpPerpendicular.y + count;
					float tmp_x20 = (-maxDistanceLine.c - tmp_y20 * maxDistanceLine.b)*1.0 / (maxDistanceLine.a*1.0);
					/**********************/
					//pp.x = tmp_x20;
					//pp.y = tmp_y20;
					//circle(src1, pp, 2, Scalar(0, 255, 0), -1, 8, 0);
					uchar c20 = src_gray.at<uchar>(int(tmp_y20), int(tmp_x20));
					if (c20 < gray_level) {
						float tmp_y21 = tmpPerpendicular.y + count + 1;
						float tmp_x21 = (-maxDistanceLine.c - tmp_y21 * maxDistanceLine.b)*1.0 / (maxDistanceLine.a*1.0);
						/**********************/
						//pp.x = tmp_x21;
						//pp.y = tmp_y21;
						//circle(src1, pp, 2, Scalar(0, 255, 0), -1, 8, 0);
						uchar c21 = src_gray.at<uchar>(int(tmp_y21), int(tmp_x21));
						float tmp_y22 = tmpPerpendicular.y + count + 2;
						float tmp_x22 = (-maxDistanceLine.c - tmp_y22 * maxDistanceLine.b)*1.0 / (maxDistanceLine.a*1.0);
						/**********************/
						//pp.x = tmp_x22;
						//pp.y = tmp_y22;
						//circle(src1, pp, 2, Scalar(0, 255, 0), -1, 8, 0);
						uchar c22 = src_gray.at<uchar>(int(tmp_y22), int(tmp_x22));
						float tmp_y23 = tmpPerpendicular.y + count + 3;
						float tmp_x23 = (-maxDistanceLine.c - tmp_y23 * maxDistanceLine.b)*1.0 / (maxDistanceLine.a*1.0);
						uchar c23 = src_gray.at<uchar>(int(tmp_y23), int(tmp_x23));
						/**********************/
						//pp.x = tmp_x23;
						//pp.y = tmp_y23;
						//circle(src1, pp, 2, Scalar(0, 255, 0), -1, 8, 0);
						if (c21 <= gray_level && c22 <= gray_level && c23 <= gray_level) {
							tmp_y = tmp_y21;
							tmp_x = tmp_x21;
							break;
						}
					}

				}
				if (!(tmp_x == 0 && tmp_y == 0)) {
					tmpPerpendicular.x = tmp_x;
					tmpPerpendicular.y = tmp_y;
				}



#ifdef DEBUG
				//cv::imshow("23", src1);
				//cv::waitKey();
				{
					/*cv::Mat src1 = src_gray.clone();*/
					//cv::cvtColor(src1, src1, cv::COLOR_GRAY2BGR);
					cv::Mat src1 = cv::imread(file_path, cv::IMREAD_COLOR);
					circle(src1, tmpPerpendicular, 2, Scalar(255, 0, 0), -1, 8, 0);
					cv::imshow("src1", src1);
					cv::waitKey();
				}
#endif

				newCorners.push_back(Point3f(tmpPerpendicular.x, tmpPerpendicular.y, maxlvangle));

			}

			vector<Point3f> filterLeftCorners = leftCorners;

			for (size_t i = 0; i < filterLeftCorners.size(); i++)
			{
				Point3f subFilterLeftCorner = filterLeftCorners[i];

				newCorners.push_back(Point3f(subFilterLeftCorner.x, subFilterLeftCorner.y, maxlvangle));

			}

			vector<Point3f> filterPlaneCorners = planeCorners;

			for (size_t i = 0; i < filterPlaneCorners.size(); i++)
			{
				Point3f subFilterPlaneCorner = filterPlaneCorners[i];

				newCorners.push_back(Point3f(subFilterPlaneCorner.x, subFilterPlaneCorner.y, maxlvangle));

			}
#ifdef DEBUG
			{
				cv::Mat src1 = cv::imread(file_path, cv::IMREAD_COLOR);
				for (int i = 0; i < newCorners.size(); i++) {
					Point2f p1;
					p1.x = newCorners[i].x;
					p1.y = newCorners[i].y;
					circle(src1, p1, 2, Scalar(0, 255, 0), -1, 8, 0);
				}
				cv::imshow("src1", src1);
				cv::waitKey();
			}
#endif
			//绘制平面区域
			cv::line(src, leftLineSEG.start, leftLineSEG.end, Scalar(0, 0, 255), 3, LINE_AA);
			cv::line(src, rightLineSEG.start, rightLineSEG.end, Scalar(255, 0, 0), 3, LINE_AA);
		}
		return 0;
	}
	vector<PaintPoint> getPointCloudCrossPaintCorners(vector<PaintPoint> &finalCorners, bool &no_plane, cvgeomerty::LINE &maxDistanceLine, const std::string &file_path, double minDistanceThresh, double planeWidthThresh, double novangle, double minPointDistanceThresh, double leftNoRPThresh) {

#ifdef DEBUG
		{
			cv::Mat src1 = cv::imread(file_path, cv::IMREAD_COLOR);
			for (int i = 0; i < finalCorners.size(); i++) {
				Point2f p1;
				p1 = finalCorners[i].point;
				circle(src1, p1, 2, Scalar(255, 0, 0), 5, 8, 0);
			}
			cv::imshow("src1", src1);
			cv::waitKey();
		}
#endif
		if (no_plane) {
			std::sort(finalCorners.begin(), finalCorners.end(), [](PaintPoint &a, const PaintPoint &b) {
				return a.point.x < b.point.x;
			});
			vector<PaintPoint>::iterator it = finalCorners.begin();
			it = finalCorners.erase(it);
		}
		else {
			/**/
			vector<PaintPoint> finalFiterCorners = ignoreFinalVertaicalSamePoints(finalCorners, 45, maxDistanceLine);
#ifdef DEBUG
			{
				cv::Mat src1 = cv::imread(file_path, cv::IMREAD_COLOR);
				for (int i = 0; i < finalFiterCorners.size(); i++) {
					Point2f p1;
					p1 = finalFiterCorners[i].point;
					circle(src1, p1, 2, Scalar(255, 0, 0), -1, 8, 0);
		}
				cv::imshow("src1", src1);
				cv::waitKey();
	}
#endif
			vector<int> finalCornerIndexes;
			for (size_t i = 0; i < finalFiterCorners.size(); i++)
			{
				PaintPoint tmpPaintPoint = finalFiterCorners[i];

				//if (tmpPaintPoint.needChange)
				//{
				//	finalCornerIndexes.push_back(i);
				//	continue;
				//}

				double minLeftY = tmpPaintPoint.point.y - leftNoRPThresh;
				double maxLeftY = tmpPaintPoint.point.y + leftNoRPThresh;

				bool haveMinLeft = false;
				bool haveMaxLeft = false;

				for (size_t j = 0; j < finalFiterCorners.size(); j++)
				{
					if (i == j)
					{
						continue;
					}
					if (finalFiterCorners[j].needChange&&finalFiterCorners[j].point.y<maxLeftY&&finalFiterCorners[j].point.y>tmpPaintPoint.point.y)
					{
						haveMaxLeft = haveMaxLeft | true;
					}

					if (finalFiterCorners[j].needChange&&finalFiterCorners[j].point.y > minLeftY&&finalFiterCorners[j].point.y < tmpPaintPoint.point.y)
					{
						haveMinLeft = haveMinLeft | true;
					}
				}

				if (!(haveMinLeft&&haveMaxLeft))
				{
					finalCornerIndexes.push_back(i);
				}
			}

			finalCorners.clear();
			for (size_t i = 0; i < finalCornerIndexes.size(); i++)
			{
				finalCorners.push_back(finalFiterCorners[finalCornerIndexes[i]]);
			}
#ifdef DEBUG
			{
				cv::Mat src1 = cv::imread(file_path, cv::IMREAD_COLOR);
				for (int i = 0; i < finalCorners.size(); i++) {
					Point2f p1;
					p1 = finalCorners[i].point;
					circle(src1, p1, 2, Scalar(255, 0, 0), -1, 8, 0);
				}
				cv::imshow("src1", src1);
				cv::waitKey();
			}
#endif
			/*
			cvgeomerty::LINE maxDistanceLine_tmp = maxDistanceLine;
			double maxlvangle = cvgeomerty::lvangle(maxDistanceLine) * 180 / CV_PI;

			vector<PaintPoint>::iterator it = finalCorners.begin();
			while (it != finalCorners.end())
			{
				PaintPoint tmpP = (*it);
				Point2f tmpPPerpendicular = cvgeomerty::perpendicular(Point2f(tmpP.point.x, tmpP.point.y), maxDistanceLine_tmp);
				if (tmpP.point == maxDistanceLine_tmp.start || tmpP.point == maxDistanceLine_tmp.end) {
					finalCorners.erase(it);
					continue;
				}
				if (fabs(tmpPPerpendicular.y - maxDistanceLine_tmp.start.y) < 60)
				{
#ifdef DEBUG
					{
						Mat src = cv::imread(file_path, cv::IMREAD_COLOR);
						circle(src, tmpPPerpendicular, 5, Scalar(0, 255, 0), 2, 8, 0);
						cv::imshow("tmpPPerpendicular", src);
						cv::waitKey();
				}
#endif
					if ((tmpP.point.x - maxDistanceLine_tmp.start.x) < -20)
						maxDistanceLine_tmp.start = tmpP.point;
				}
				if (fabs(tmpPPerpendicular.y - maxDistanceLine_tmp.end.y) < 60)
				{
#ifdef DEBUG
					{
						Mat src = cv::imread(file_path, cv::IMREAD_COLOR);
						circle(src, tmpPPerpendicular, 5, Scalar(0, 255, 0), 2, 8, 0);
						cv::imshow("tmpPPerpendicular", src);
						cv::waitKey();
				}
#endif
					if ((tmpP.point.x - maxDistanceLine_tmp.end.x) < -20)
						maxDistanceLine_tmp.end = tmpP.point;
				}
				it++;
			}
			finalCorners.clear();
			finalCorners.push_back(PaintPoint(maxDistanceLine_tmp.start, maxlvangle, false));
			finalCorners.push_back(PaintPoint(maxDistanceLine_tmp.end, maxlvangle, false));*/
		}

#ifdef DEBUGC
			if (WorkFlowPath != "") {
				int res = fileExist((char*)file_path.c_str());
				if (res == -1) {
					LOG(ERRORL) << "加载图片错误";
					throw exception((std::string(__FUNCTION__) + ":加载图片错误").c_str());
				}
				Mat src = cv::imread(file_path, cv::IMREAD_COLOR);
				for (size_t i = 0; i < finalCorners.size(); i++)
				{
					if (finalCorners[i].needChange)
					{
						circle(src, finalCorners[i].point, 5, Scalar(0, 255, 0), 2, 8, 0);
					}
					else {
						circle(src, finalCorners[i].point, 5, Scalar(0, 0, 255), 2, 8, 0);
					}
					double arrowWidth = 30;
					//arrowedLine(src, finalCorners[i].point, Point2f(finalCorners[i].point.x - arrowWidth, finalCorners[i].point.y - (arrowWidth)* tan(finalCorners[i].angle / 180 * CV_PI)), Scalar(0, 0, 255), 2, LINE_AA, 0, 0.3);
				}
				//string file_path_w = getFileName(file_path);
				//file_path_w = saveGroupPath + file_path_w;
				//cv::imwrite(file_path_w.substr(0, file_path_w.length() - 4) + "_pre.jpg", src);
				string file_name = getFileName(file_path);
				imwrite(WorkFlowPath + file_name.substr(0, file_name.length() - 4)+ "_paint.jpg", src);
				//cv::imshow("paint", src);
				//cv::waitKey();
			}
#endif 
		return finalCorners;
	}

}


