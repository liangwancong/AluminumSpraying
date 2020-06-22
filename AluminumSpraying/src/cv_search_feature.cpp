#include "cv_search_feature.h"

namespace cv_search_feature {
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
	int searchLineSEG(const std::string &file_path, Vec4f &plane_line, cvgeomerty::LINE &LineSEG, Eigen::Affine3f &transform_matrix, double minAngleThresh, double minIntervalThresh, double minDistanceThresh) {
		int res = fileExist((char*)file_path.c_str());
		if (res == -1) {
			LOG(ERRORL) << "加载图片错误";
			throw exception((std::string(__FUNCTION__) + ":加载图片错误").c_str());
		}
		Mat src = cv::imread(file_path, cv::IMREAD_GRAYSCALE);
		//提取骨骼
		Mat skeleton = src.clone();

		cv::threshold(src, skeleton, 128, 1, cv::THRESH_BINARY);
		skeleton = cvskeleton::thinImage(skeleton);
		skeleton = skeleton * 255;

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
#ifdef DEBUG
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


		k_line = -1 / k_line;
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
		if (rightCorners.size() > 0) {
			Point2f p = rightCorners[0];
			float k = rightLineSEG.k;
			float x1 = 0;
			float y1 = k * (x1)-k * p.x + p.y;
			float x2 = src.rows;
			float y2 = k * (x2)-k * p.x + p.y;
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
			double distance2 = cvgeomerty::pointToLineDistance(b, dLineSEG);
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
		crosss.push_back(cvgeomerty::getCrossPoint(leftLineSEG, upLineSEG));
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
		std::cout << "crosss_tmp[0].y :" << crosss_tmp[0].y << std::endl;
		std::cout << "crosss_tmp[3].y :" << crosss_tmp[3].y << std::endl;
		crosss.clear();
		if (crosss_tmp[0].y > crosss_tmp[1].y&&fabs(fabs(line2.angle) - fabs(line1.angle)) < 5) {
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
		LineSEG = cvgeomerty::LINE(Vec4f(crosss[0].x, crosss[0].y, crosss[3].x, crosss[3].y));
#ifdef DEBUG
		{
			cv::Mat src1 = cv::imread(file_path, cv::IMREAD_COLOR);
			cv::line(src1, dLineSEG.start, dLineSEG.end, Scalar(0, 255, 255), 2, LINE_AA);
			cv::line(src1, upLineSEG.start, upLineSEG.end, Scalar(255, 0, 0), 2, LINE_AA);
			cv::line(src1, rightLineSEG.start, rightLineSEG.end, Scalar(0, 255, 0), 2, LINE_AA);
			cv::line(src1, leftLineSEG.start, leftLineSEG.end, Scalar(0, 0, 255), 2, LINE_AA);
			cv::imshow("line", src1);
			cv::waitKey();
		}
#endif

		return 0;
	}
}
