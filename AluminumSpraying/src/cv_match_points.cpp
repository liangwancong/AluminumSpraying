#include "cv_match_points.h"
#include "parameters.h"
#include "global.h"
#include "logger.h"

namespace CVMatchPoints {
	using namespace std;
	using namespace cv;

	using namespace stringutil;
	using namespace json2model;

	using namespace CVSearchPoints;
	using namespace fileutil;
	using namespace AluminumGlobal;
	using namespace logger;

std::tuple<double, cv::Point, cvgeomerty::LINE> get2DPCATransfromTuple(const string &file_path, const char *out_path, double minAngleThresh, double minIntervalThresh, double minDistanceThresh, double novangle, int no_plane_s)
{
	int res = fileExist((char*)file_path.c_str());
	if (res == -1) {
		LOG(ERRORL) << "加载图片错误";
		throw exception((std::string(__FUNCTION__) + ":加载图片错误").c_str());
	}
	Mat img = cv::imread(file_path, cv::IMREAD_COLOR);
	Mat gary;
	cvtColor(img, gary, COLOR_BGR2GRAY);
	bool no_plane = false;
	
	double angle = 0;
	Point pos;
	vector<Point3f> newCorners;
	cvgeomerty::LINE maxline;
	bool skeletonImage = true;
	res = getPointCloudCrossOriginCornersPre(newCorners, no_plane, maxline, file_path, minAngleThresh, minIntervalThresh, minDistanceThresh, novangle, skeletonImage);
	if (res == -1) {
		LOG(ERRORL) << "getPointCloudCrossOriginCorner错误";
		throw exception((std::string(__FUNCTION__) + ":getPointCloudCrossOriginCorner错误").c_str());
	}
	Point2f midMaxlineP = cvgeomerty::getMidPoint(maxline.start, maxline.end);

	if (no_plane_s != NoPlaneS) {
		no_plane = no_plane_s;
	}
	if (no_plane)
	{
		vector<Point2d> eigenVectorsPCA(2); //特征向量ve(2x2)
		vector<double> eigenValuesPCA(2);//特征值va(2x1)


		angle = cvgeomerty::getPCAAngle(gary, eigenVectorsPCA, eigenValuesPCA, pos);

		//随便给的长度用来画坐标轴
		Point p1 = pos + 0.02*Point(static_cast<int>(eigenVectorsPCA[0].x*eigenValuesPCA[0]), static_cast<int>(eigenVectorsPCA[0].y*eigenValuesPCA[0]));
		Point p2 = pos - 0.05*Point(static_cast<int>(eigenVectorsPCA[1].x*eigenValuesPCA[1]), static_cast<int>(eigenVectorsPCA[1].y*eigenValuesPCA[1]));

		line(img, pos, p1, Scalar(255, 0, 0), 2, 8, 0);
		line(img, pos, p2, Scalar(0, 0, 255), 2, 8, 0);

	}
	else {
		angle = maxline.angle < 0 ? (180 + maxline.angle) : maxline.angle;

		pos = Point((int)midMaxlineP.x, (int)midMaxlineP.y);
		Point p1 = pos + Point(10 * sin(angle), 10 * cos(angle));
		Point p2 = pos - Point(5 * cos(angle), 5 * sin(angle));

		line(img, pos, p1, Scalar(255, 0, 0), 2, 8, 0);
		line(img, pos, p2, Scalar(0, 0, 255), 2, 8, 0);
	}


	cout << "偏转角度=" << angle << endl;

#ifdef DEBUG
	if (out_path == "")
	{
		string file_path_w = getFileName(file_path);
		file_path_w = saveGroupPath + file_path_w;
		imwrite(file_path_w.substr(0, file_path_w.length() - 4) + "_pca.jpg", img);
	}
	if (strlen(out_path) > 1)
	{
		imwrite(out_path, img);
	}
#endif
	return std::make_tuple(angle, pos, maxline);
}
	double get2DPCATransfrom(const string &file_path, const char *out_path,double minAngleThresh, double minIntervalThresh, double minDistanceThresh, double novangle, int no_plane_s = NoPlaneS)//no plane select
	{

		std::tuple<double, Point, cvgeomerty::LINE> t = get2DPCATransfromTuple(file_path, out_path, minAngleThresh, minIntervalThresh, minDistanceThresh, novangle, no_plane_s);

		return get<0>(t);
	}

	string saveMatchModelWithFilePath(const string &file_path) {
		int res = fileExist((char*)file_path.c_str());
		if (res == -1) {
			LOG(ERRORL) << "加载图片错误";
			throw exception((std::string(__FUNCTION__) + ":加载图片错误").c_str());
		}
		Mat src = cv::imread(file_path, cv::IMREAD_COLOR);
		double angle = get2DPCATransfrom(file_path, "", AluminumParameters::MinAngleThresh, AluminumParameters::MinIntervalThresh, AluminumParameters::MinDistanceThresh, AluminumParameters::Novangle);

		//创建一个旋转后的图像  
		Mat RatationedImg(src.rows, src.cols, CV_8UC1);
		RatationedImg.setTo(0);
		//对RoiSrcImg进行旋转  
		Point2f center = Point2f(src.rows / 2., src.cols / 2.);  //中心点  
		Mat M2 = getRotationMatrix2D(center, angle - 90, 1);//计算旋转加缩放的变换矩阵 
		warpAffine(src, RatationedImg, M2, src.size(), 1, 0, Scalar(0));//仿射变换 

		string newPath = file_path.substr(0, file_path.length() - 4) + "_r.jpg";
		imwrite(newPath, RatationedImg); //将矫正后的图片保存下来

		return newPath;
	}

	vector<PaintPoint>getPointCloudCrossMatchCorners(vector<Point3f> &newCorners, bool &no_plane, cvgeomerty::LINE &maxDistanceLine, const string file_path, double minAngleThresh, double minIntervalThresh, double minDistanceThresh, double planeWidthThresh, double novangle, double minPointDistanceThresh, double leftNoRPThresh) {
		int res = fileExist((char*)file_path.c_str());
		if (res == -1) {
			LOG(ERRORL) << "加载图片错误";
			throw exception((std::string(__FUNCTION__) + ":加载图片错误").c_str());
		}
		Mat src = cv::imread(file_path, cv::IMREAD_COLOR);
#ifdef DEBUG
		{
			cv::Mat src1 = cv::imread(file_path, cv::IMREAD_COLOR);
			for (int i = 0; i < newCorners.size(); i++) {
				Point2f p1;
				p1.x = newCorners[i].x;
				p1.y = newCorners[i].y;
				circle(src1, p1, 2, Scalar(255, 0, 0), -1, 8, 0);
			}
			line(src1, maxDistanceLine.start, maxDistanceLine.end, Scalar(255, 0, 255), 1, LINE_AA);
			cv::imshow("match_in", src1);
			cv::waitKey();

		}
#endif
		vector<PaintPoint> match_corners;
		if (no_plane)
		{
			for (size_t i = 0; i < newCorners.size(); i++)
			{
				Point3f newSubCorner = newCorners[i];

				match_corners.push_back(PaintPoint(Point2f(newSubCorner.x, newSubCorner.y), newSubCorner.z, false));
			}
		}
		else {
			double maxlvangle = cvgeomerty::lvangle(maxDistanceLine) * 180 / CV_PI;

			cvgeomerty::LINE leftLineSEG = Vec4f((-(maxDistanceLine.c + maxDistanceLine.a*(planeWidthThresh+3)) - maxDistanceLine.b * 0) / maxDistanceLine.a, 0, (-(maxDistanceLine.c + maxDistanceLine.a*(planeWidthThresh+3)) - maxDistanceLine.b * 500) / maxDistanceLine.a, src.cols);
			cvgeomerty::LINE rightLineSEG = Vec4f((-(maxDistanceLine.c - maxDistanceLine.a*planeWidthThresh) - maxDistanceLine.b * 0) / maxDistanceLine.a, 0, (-(maxDistanceLine.c - maxDistanceLine.a*planeWidthThresh) - maxDistanceLine.b * 500) / maxDistanceLine.a, src.cols);

			vector<Point3f> planeCorners;//正面及正面以下的点的垂点
			vector<Point3f> leftCorners;//正面以上的点

			//分离|||左侧 平面 右侧  由于线扫点光源从上至下有不稳定因素，右侧 底边和上边又或无，但和最左侧连通才会有此现象，顾将点移至平面筛选可以增加稳定性
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
						Point2f tmpPerpendicular = cvgeomerty::perpendicular(Point2f(newSubCorner.x, newSubCorner.y), maxDistanceLine);

						planeCorners.push_back(Point3f(tmpPerpendicular.x, tmpPerpendicular.y, newSubCorner.z));
					}
					else {
						planeCorners.push_back(newSubCorner);
					}
				}
			}

#ifdef DEBUG
			{
				cv::Mat src1 = cv::imread(file_path, cv::IMREAD_COLOR);
				for (int i = 0; i < leftCorners.size(); i++) {
					Point2f p1;
					p1.x = leftCorners[i].x;
					p1.y = leftCorners[i].y;
					circle(src1, p1, 2, Scalar(0, 0, 255), -1, 8, 0);
				}
				for (int i = 0; i < planeCorners.size(); i++) {
					Point2f p1;
					p1.x = planeCorners[i].x;
					p1.y = planeCorners[i].y;
					circle(src1, p1, 2, Scalar(0, 255, 0), -1, 8, 0);
				}

				line(src1, leftLineSEG.start, leftLineSEG.end, Scalar(255, 0, 0), 1, LINE_AA);
				line(src1, rightLineSEG.start, rightLineSEG.end, Scalar(255, 0, 0), 1, LINE_AA);
				//circle(src1, maxDistanceLine.start, 2, Scalar(0, 0, 255), -1, 8, 0);
				//circle(src1, maxDistanceLine.end, 2, Scalar(0, 0, 255), -1, 8, 0);
				cv::imshow("src1", src1);
				cv::waitKey();
			}

#endif

			cvgeomerty::LINE planeLine;
			cvgeomerty::LINE leftLine;
			vector<Point3f> filterLeftCorners;
			vector<Point3f> filterPlaneCorners;
			planeLine.start = Point2f((-maxDistanceLine.c - maxDistanceLine.b * 0) / maxDistanceLine.a, 0);
			planeLine.end = Point2f((-maxDistanceLine.c - maxDistanceLine.b * 500) / maxDistanceLine.a, 500);

			//选出靠左边点
			if (leftCorners.size() > 0) {
				float minx = 600;
				int index = 0;
				for (size_t i = 0; i < leftCorners.size(); i++)
				{
					if (leftCorners[i].x < minx) {
						minx = leftCorners[i].x;
						index = i;
					}
				}

				//Point2f midMaxlineP = cvgeomerty::getMidPoint(maxDistanceLine.start, maxDistanceLine.end);
				double distance = cvgeomerty::pointToLineDistance(Point2f(leftCorners[index].x, leftCorners[index].y), maxDistanceLine);
				//float offset = fabs(midMaxlineP.x - leftCorners[index].x);
				float offset = fabs(distance*sin(maxDistanceLine.angle*CV_PI/180.0));
				
#ifdef DEBUG
				{
					std::cout <<"offset:"<< offset << std::endl;
					cv::Mat src1 = cv::imread(file_path, cv::IMREAD_COLOR);
					//circle(src1, midMaxlineP, 2, Scalar(0, 0, 255), -1, 8, 0);
					circle(src1, Point2f(leftCorners[index].x, leftCorners[index].y), 2, Scalar(0, 255, 0), -1, 8, 0);
					cv::imshow("leftp", src1);
					cv::waitKey();
				}

#endif
				leftLine.start = Point2f((-(maxDistanceLine.c + maxDistanceLine.a*offset) - maxDistanceLine.b * 0) / maxDistanceLine.a, 0);
				leftLine.end = Point2f((-(maxDistanceLine.c + maxDistanceLine.a*offset) - maxDistanceLine.b * 500) / maxDistanceLine.a, 500);
				vector<Point3f>::iterator it = leftCorners.begin();
				while (it != leftCorners.end())
				{
					Point3f tmpP = (*it);

					double tmpDistance = cvgeomerty::pointToLineDistance(Point2f(tmpP.x, tmpP.y), leftLine);
					if (tmpDistance < 30)
					{
						filterLeftCorners.push_back(tmpP);
					}
					it++;
				}
			}
			//选出平面上点
			vector<Point3f>::iterator it = planeCorners.begin();
			while (it != planeCorners.end())
			{
				Point3f tmpP = (*it);

				double tmpDistance = cvgeomerty::pointToLineDistance(Point2f(tmpP.x, tmpP.y), planeLine);
				if (tmpDistance < 5)
				{
					filterPlaneCorners.push_back(tmpP);
				}
				it++;
			}

			//左边点,平面点分别进行组合
			vector<Point3f> filterLeftCorners1 = ignorePoints(filterLeftCorners, 20, true);
			vector<Point3f> filterPlaneCorners1 = ignorePoints(filterPlaneCorners, minPointDistanceThresh, true);
			//竖直方向 平面点离左边点近的,将平面点删掉
			vector<Point3f>::iterator it_left = filterLeftCorners1.begin();
			while (it_left != filterLeftCorners1.end())
			{
				Point3f tmpP_left = *it_left;
				Point2f tmpP1Perpendicular = cvgeomerty::perpendicular(Point2f(tmpP_left.x, tmpP_left.y), planeLine);
				vector<Point3f>::iterator it_plane = filterPlaneCorners1.begin();

				cvgeomerty::LINE vertaicalLine = cvgeomerty::LINE(Vec4f(tmpP_left.x, tmpP_left.y, tmpP1Perpendicular.x, tmpP1Perpendicular.y));

				while (it_plane != filterPlaneCorners1.end())
				{
					Point3f tmpP_plane = (*it_plane);

					double tmpDistance = cvgeomerty::pointToLineDistance(Point2f(tmpP_plane.x, tmpP_plane.y), vertaicalLine);
					double tmpDistance1 = cvgeomerty::distance(Point2f(tmpP_plane.x, tmpP_plane.y), maxDistanceLine.start);
					double tmpDistance2 = cvgeomerty::distance(Point2f(tmpP_plane.x, tmpP_plane.y), maxDistanceLine.end);

					if (tmpDistance < 20)
					{
						filterPlaneCorners1.erase(it_plane);
						continue;
					}

					it_plane++;
				}
				it_left++;
			}
			//过滤掉与长直线端点较近的点
			vector<Point3f>::iterator it_plane = filterPlaneCorners1.begin();
			while (it_plane != filterPlaneCorners1.end())
			{
				Point3f tmpP_plane = (*it_plane);
				double tmpDistance1 = cvgeomerty::distance(Point2f(tmpP_plane.x, tmpP_plane.y), maxDistanceLine.start);
				double tmpDistance2 = cvgeomerty::distance(Point2f(tmpP_plane.x, tmpP_plane.y), maxDistanceLine.end);

				if ((tmpDistance1 < 30) || (tmpDistance2 < 30))
				{
					filterPlaneCorners1.erase(it_plane);
					continue;
				}

				it_plane++;
			}
			//左边点竖直过滤
#ifdef DEBUG
	{
		cv::Mat src1 = cv::imread(file_path, cv::IMREAD_COLOR);

		for (int i = 0; i < filterLeftCorners1.size(); i++) {
			Point2f p1;
			p1.x = filterLeftCorners1[i].x;
			p1.y = filterLeftCorners1[i].y;
			circle(src1, p1, 2, Scalar(0, 255, 0), -1, 8, 0);
		}
		for (int i = 0; i < filterPlaneCorners1.size(); i++) {
			Point2f p1;
			p1.x = filterPlaneCorners1[i].x;
			p1.y = filterPlaneCorners1[i].y;
			circle(src1, p1, 2, Scalar(0, 0, 255), -1, 8, 0);
		}
		if (leftCorners.size() > 0) {
			line(src1, leftLine.start, leftLine.end, Scalar(0, 0, 255), 1, LINE_AA);
		}
		line(src1, planeLine.start, planeLine.end, Scalar(255, 0, 0), 1, LINE_AA);
		line(src1, maxDistanceLine.start, maxDistanceLine.end, Scalar(255, 0, 0), 1, LINE_AA);
		circle(src1, maxDistanceLine.start, 2, Scalar(0, 0, 255), -1, 8, 0);
		circle(src1, maxDistanceLine.end, 2, Scalar(0, 0, 255), -1, 8, 0);
		cv::imshow("match filter", src1);
		cv::waitKey();

	}
#endif
			for (size_t i = 0; i < filterLeftCorners1.size(); i++)
			{
				Point3f subFilterLeftCorner = filterLeftCorners1[i];

				match_corners.push_back(PaintPoint(Point2f(subFilterLeftCorner.x, subFilterLeftCorner.y), maxlvangle, true));

			}

			for (size_t i = 0; i < filterPlaneCorners1.size(); i++)
			{
				Point3f subFilterPlaneCorner = filterPlaneCorners1[i];
				match_corners.push_back(PaintPoint(Point2f(subFilterPlaneCorner.x, subFilterPlaneCorner.y), maxlvangle, false));
			}
			match_corners.push_back(PaintPoint(maxDistanceLine.start, maxlvangle, false));
			match_corners.push_back(PaintPoint(maxDistanceLine.end, maxlvangle, false));
			//绘制平面区域
			line(src, leftLineSEG.start, leftLineSEG.end, Scalar(0, 0, 255), 3, LINE_AA);
			line(src, rightLineSEG.start, rightLineSEG.end, Scalar(255, 0, 0), 3, LINE_AA);

		}
#ifdef DEBUGC
		if (WorkFlowPath != "") {
			for (size_t i = 0; i < match_corners.size(); i++)
			{
				if (match_corners[i].needChange)
				{
					circle(src, match_corners[i].point, 5, Scalar(0, 255, 0), 2, 8, 0);
				}
				else {
					circle(src, match_corners[i].point, 5, Scalar(0, 0, 255), 2, 8, 0);
				}
				double arrowWidth = 30;
				arrowedLine(src, match_corners[i].point, Point2f(match_corners[i].point.x - arrowWidth, match_corners[i].point.y - (arrowWidth)* tan(match_corners[i].angle / 180 * CV_PI)), Scalar(0, 0, 255), 2, LINE_AA, 0, 0.3);
			}
			//string file_path_w = getFileName(file_path);
			//file_path_w = saveGroupPath + file_path_w;
			//imwrite(file_path_w.substr(0, file_path_w.length() - 4) + "_match.jpg", src);
			line(src, maxDistanceLine.start, maxDistanceLine.end, Scalar(255, 0, 0), 1, LINE_AA);
			string file_name = getFileName(file_path);
			imwrite(WorkFlowPath + file_name.substr(0, file_name.length() - 4) + "_match.jpg", src);
		}
#endif 
#ifdef DEBUG
		{
			cv::Mat src1 = cv::imread(file_path, cv::IMREAD_COLOR);
			//vector<Point2f> tmpLP;
			for (int i = 0; i < match_corners.size(); i++) {
				circle(src1, match_corners[i].point, 2, Scalar(0, 255, 0), -1, 8, 0);
				//tmpLP.push_back(match_corners[i].point);
			}
			//RotatedRect minRect = minAreaRect(tmpLP);
			//Point2f vertex[4];//用于存放最小矩形的四个顶点
			//minRect.points(vertex);
			//line(src1, vertex[0], vertex[1], Scalar(255, 0, 255), 1, LINE_AA);
			//line(src1, vertex[1], vertex[2], Scalar(255, 0, 255), 1, LINE_AA);
			//line(src1, vertex[2], vertex[3], Scalar(255, 0, 255), 1, LINE_AA);
			//line(src1, vertex[3], vertex[0], Scalar(255, 0, 255), 1, LINE_AA);
			cv::imshow("match_out", src1);
			cv::waitKey();
		}
#endif

		return match_corners;
	}

	string saveDAMatchModelWithFilePath(const string file_path, const string key_name, double minAngleThresh, double minIntervalThresh, double minDistanceThresh, double planeWidthThresh, double novangle, double minPointDistanceThresh, double leftNoRPThresh, bool skeletonImage) {
		string file_path_tmp = file_path;
		vector<Point3f> corners;
		bool no_plane;
		cvgeomerty::LINE maxDistanceLine;


		int res = getPointCloudCrossOriginCorners(corners, no_plane, maxDistanceLine, file_path, minAngleThresh, minIntervalThresh, minDistanceThresh, planeWidthThresh, novangle, minPointDistanceThresh, skeletonImage);
		if (res == -1) {
			return "";
		}
		vector<PaintPoint> points = getPointCloudCrossMatchCorners(corners, no_plane, maxDistanceLine, file_path, minAngleThresh, minIntervalThresh, minDistanceThresh, planeWidthThresh, novangle, minPointDistanceThresh, leftNoRPThresh);
		vector<PaintPoint> points_tmp;
		points_tmp = points;
		vector<PaintPoint> paint_points = getPointCloudCrossPaintCorners(points_tmp, no_plane, maxDistanceLine, file_path, minDistanceThresh, planeWidthThresh, novangle, minPointDistanceThresh, leftNoRPThresh);
#ifdef DEBUG
		vector<PaintPoint> points_t;
		points_t = points;
		double matchAngle;
		string matchMat;
		Point outStablePoint;
		double matchVal = 0;
		string matchKey = odaMatchKeyPoint(file_path, points_t, saveModelPath,matchVal, matchAngle, matchMat, &outStablePoint);
		std::cout << "************************************************************************************matchVal:" << matchVal << std::endl;
		//if (matchKey.length() > 1&& matchVal>0.9) {
		//}
#endif 

		std::tuple<double, cv::Point, cvgeomerty::LINE> t = get2DPCATransfromTuple(file_path, "", minAngleThresh, minIntervalThresh, minDistanceThresh, novangle);
		double angle = get<0>(t);
		Point stable_point= get<1>(t);
		vector<Point2f> originPoints;
		char *jsonString;
		//程序执行时未找到模板,存储自动规划点,供后续工件使用
		if (key_name=="tmp") {
			for (size_t i = 0; i < paint_points.size(); i++)
			{
				PaintPoint tmpCorner = paint_points[i];
				originPoints.push_back(tmpCorner.point);
			}
			vector<Point2f> transformPoints;
			Mat M2 = getRotationMatrix2D(stable_point, angle - 90, 1);//计算旋转加缩放的变换矩阵 
			cv::transform(originPoints, transformPoints, M2);

			for (size_t i = 0; i < paint_points.size(); i++)
			{
				paint_points[i].angle = (paint_points[i].angle + 90 - angle);
				paint_points[i].point = transformPoints[i];
			}
			vector<Point2f> tmpLP;
			for (size_t i = 0; i < paint_points.size(); i++)
			{
				tmpLP.push_back(paint_points[i].point);
			}
			RotatedRect minRect = minAreaRect(tmpLP);
			Point2f vertex[4];//用于存放最小矩形的四个顶点
			minRect.points(vertex);
			double maxRectY = 0;
			double minRectY = 99999;

			for (size_t i = 0; i < 4; i++)
			{
				if (maxRectY < vertex[i].y)
				{
					maxRectY = vertex[i].y;
				}
				if (minRectY > vertex[i].y)
				{
					minRectY = vertex[i].y;
				}
			}
			MatchModel matches;

			matches.points = paint_points;
			matches.max_height = abs(maxRectY - minRectY);
			matches.stable_point = stable_point;
			jsonString = createKeyPointsToJSON(matches);
		}
		//正常存储模板,keyname必须为""
		else {
			for (size_t i = 0; i < points.size(); i++)
			{
				PaintPoint tmpCorner = points[i];
				originPoints.push_back(tmpCorner.point);
			}
			vector<Point2f> transformPoints;
			Mat M2 = getRotationMatrix2D(stable_point, angle - 90, 1);//计算旋转加缩放的变换矩阵 
			cv::transform(originPoints, transformPoints, M2);

			for (size_t i = 0; i < points.size(); i++)
			{
				points[i].angle = (points[i].angle + 90 - angle);
				points[i].point = transformPoints[i];
			}
			vector<Point2f> tmpLP;
			for (size_t i = 0; i < points.size(); i++)
			{
				tmpLP.push_back(points[i].point);
			}
			RotatedRect minRect = minAreaRect(tmpLP);
			Point2f vertex[4];//用于存放最小矩形的四个顶点
			minRect.points(vertex);
			double maxRectY = 0;
			double minRectY = 99999;

			for (size_t i = 0; i < 4; i++)
			{
				if (maxRectY < vertex[i].y)
				{
					maxRectY = vertex[i].y;
				}
				if (minRectY > vertex[i].y)
				{
					minRectY = vertex[i].y;
				}
			}
			MatchModel matches;

			matches.points = points;
			matches.max_height = abs(maxRectY - minRectY);
			matches.stable_point = stable_point;
			jsonString = createKeyPointsToJSON(matches);
		}

		


		char *substr = strtok((char*)file_path.c_str(), "\\");
		char charlist[50][50] = { "" };
		int i = 0;
		while (substr != NULL) {
			strcpy(charlist[i], substr);/*把新分割出来的子字符串substr拷贝到要存储的charlsit中*/
			i++;
			substr = strtok(NULL, "\\");/*在第一次调用时，strtok()必需给予参数str字符串，
									   往后的调用则将参数str设置成NULL。每次调用成功则返回被分割出片段的指针。*/
		}
		string lastName = charlist[i - 1];

		substr = strtok((char*)lastName.c_str(), ".");
		i = 0;
		char newcharlist[50][50] = { "" };
		while (substr != NULL) {
			strcpy(newcharlist[i], substr);
			i++;
			substr = strtok(NULL, ".");
		}
		strcpy(newcharlist[i - 1], "key");

		string newLastName = newcharlist[0];
		for (size_t j = 1; j < i; j++)
		{
			newLastName = newLastName + "." + newcharlist[j];
		}
		//程序执行时未找到模板,存储自动规划点,供后续工件使用
		string newPath = key_name.length() > 0 ? (saveModelPath + key_name + ".key") : (saveModelPath + newLastName);
		string new_imgPath = key_name.length() > 0 ? (saveModelImgPath + key_name+".jpg") : (saveModelImgPath + newLastName.substr(0, newPath.length() - 3) + "jpg");
		//int iRtn = _access(newPath.c_str(), 0);
		//if (iRtn == 0)
		//{
		//	return "";
		//}
		try
		{
			ofstream outfile(newPath, ios::binary);
			outfile << jsonString;
			outfile.close();
			cv::Mat img = cv::imread(file_path_tmp);
			cv::imwrite(new_imgPath, img);
		}
		catch (const std::exception&)
		{
			newPath = "";
		}

		return newPath;
	}

	string odaMatchKeyPoint(const string &file_path, vector<PaintPoint> corners, const string &match_dir,double &outMatchVal, double &outMatchAngle, string &outMatchMat, cv::Point *outStablePoint) {
		if (corners.size() < 1)
		{
			return "";
		}
		std::tuple<double, cv::Point, cvgeomerty::LINE> t = get2DPCATransfromTuple(file_path, "", AluminumParameters::MinAngleThresh, AluminumParameters::MinIntervalThresh, AluminumParameters::MinDistanceThresh, AluminumParameters::Novangle);
		double angle = get<0>(t);
		cv::Point stablePoint = get<1>(t);
		outStablePoint->x = stablePoint.x;
		outStablePoint->y = stablePoint.y;

		vector<Point2f> originPoints;
		for (size_t i = 0; i < corners.size(); i++)
		{
			PaintPoint tmpCorner = corners[i];
			originPoints.push_back(tmpCorner.point);
		}
		vector<Point2f> transformPoints;

		outMatchAngle = angle - 90;
		Mat M2 = getRotationMatrix2D(stablePoint, outMatchAngle, 1);//计算旋转加缩放的变换矩阵 
		cv::transform(originPoints, transformPoints, M2);
	#ifdef DEBUG
		{
			Mat src = cv::imread(file_path, cv::IMREAD_COLOR);
			for (int i = 0; i < originPoints.size(); i++) {
				circle(src, originPoints[i], 2, Scalar(0, 0, 255), -1, 8, 0);
			}
			for (int i = 0; i < transformPoints.size(); i++) {
				circle(src, transformPoints[i], 2, Scalar(255, 0, 0), -1, 8, 0);
			}
			cv::imshow("tans points", src);
			cv::waitKey();
		}
	#endif

		vector<string> fileNames;
		fileutil::getFilesName(match_dir, fileNames);
	#ifdef DEBUG
		std::cout << "****************:" << fileNames.size() << std::endl;
		for (int i = 0; i < fileNames.size(); i++) {
			std::cout << fileNames[i] << std::endl;
		}
	#endif


		string maxMatchName = "";

		double maxMatchVal = 0;
		MatchModel maxMatches;

		for (size_t i = 0; i < corners.size(); i++)
		{
			corners[i].angle = (corners[i].angle - outMatchAngle);
			corners[i].point = transformPoints[i];
		}

		vector<Point2f> tmpLP;
		for (size_t i = 0; i < corners.size(); i++)
		{
			tmpLP.push_back(corners[i].point);
		}
		RotatedRect minRect = minAreaRect(tmpLP);
		Point2f vertex[4];//用于存放最小矩形的四个顶点
		minRect.points(vertex);
	#ifdef DEBUG
		{
			Mat src = cv::imread(file_path, cv::IMREAD_COLOR);
			for (int i = 0; i < tmpLP.size(); i++) {
				circle(src, tmpLP[i], 2, Scalar(0, 0, 255), -1, 8, 0);
			}
			for (int j = 0; j < 4; j++)
				line(src, vertex[j], vertex[(j + 1) % 4], Scalar(rand() % 255, rand() % 255, rand() % 255), 1, 8);
			cv::imshow("minAreaRect", src);
			cv::waitKey();
		}

	#endif


		double maxRectY = 0;
		double minRectY = 99999;

		for (size_t i = 0; i < 4; i++)
		{
			if (maxRectY < vertex[i].y)
			{
				maxRectY = vertex[i].y;
			}
			if (minRectY > vertex[i].y)
			{
				minRectY = vertex[i].y;
			}
		}

		double max_height = abs(maxRectY - minRectY);
		Mat affineMat;

		for (size_t i = 0; i < fileNames.size(); i++)
		{
			string fileName = fileNames[i];
			string filePath = match_dir + fileName;
			std::cout << "*****************************************************fileName:" << "fileName:" << fileName << std::endl;
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

			MatchModel matches;
			vector<PaintPoint> modelKey;

			int status = parsingJSONToKeyPoints(jsonString.c_str(), matches);
	#ifdef DEBUG
			std::cout << "************************************************:" << std::endl;
			cout << "jsonString:" << jsonString << endl;
			cout << "status:" << status << endl;
	#endif

			if (status == 1)
			{
				modelKey = matches.points;

				double distanceThresh = 30;
				double stablePointDistanceThresh = 30;
				double angleThresh = 10;
				double maxHeightThresh = 10;
				double cornerSizeThresh = 5;

				//double percentDistance = 0.3;
				//double percentAngle = 0.2;
				//double percentNeedChange = 0.1;
				//double percentMaxHeight = 0.2;
				//double percentStablePoint = 0.1;
				//double percentCornerSize = 0.1;

				//double percentDistance = 0.3;
				//double percentAngle = 0.2;
				//double percentNeedChange = 0.1;
				//double percentMaxHeight = 0.02;
				//double percentStablePoint = 0.1;
				//double percentCornerSize = 0.27;
				//double sumPercent = 0;

				double percentDistance = 0.35;
				double percentAngle = 0.2;
				double percentNeedChange = 0.1;
				double percentMaxHeight = 0.07;
				double percentStablePoint = 0.1;
				double percentCornerSize = 0.17;
				double sumPercent = 0;

				vector<Point2f> originModelKey;
				vector<Point2f> originCornerKey;

				for (size_t i = 0; i < modelKey.size(); i++)
				{
					PaintPoint paintPoint = modelKey[i];

					double maxPercent = 0;

					PaintPoint matchPaintPoint;
					for (size_t j = 0; j < corners.size(); j++)
					{
						double difDistance = cvgeomerty::distance(paintPoint.point, transformPoints[j]);
						double difAngle = abs(paintPoint.angle - (corners[j].angle + 90 - angle+outMatchAngle));
						bool difNeedChange = paintPoint.needChange == corners[j].needChange;

						double currentPercentDistance = (percentDistance*((distanceThresh - difDistance) / distanceThresh));
						double currentPercentAngle = (percentAngle*((angleThresh - difAngle) / angleThresh));
						double currentPercentNeedChange = percentNeedChange * (difNeedChange ? 1 : 0.5);

						currentPercentDistance = currentPercentDistance > percentDistance ? percentDistance : (currentPercentDistance < 0 ? 0 : currentPercentDistance);
						currentPercentAngle = currentPercentAngle > percentAngle ? percentAngle : (currentPercentAngle < 0 ? 0 : currentPercentAngle);

						double currentPercent = currentPercentDistance + currentPercentAngle + currentPercentNeedChange;
						if (currentPercent > maxPercent)
						{
							cout <<"currentPercentDistance:"<< currentPercent << "---currentPercentDistance:" << currentPercentDistance << "---currentPercentAngle:" << currentPercentAngle << "---currentPercentNeedChange:" << currentPercentNeedChange << endl;
							maxPercent = currentPercent;
							matchPaintPoint = corners[j];
						}
					}

					sumPercent += maxPercent;
					originModelKey.push_back(paintPoint.point);
					originCornerKey.push_back(matchPaintPoint.point);

				}
				
	#ifdef DEBUG
				{
					std::cout << "fileName:" << fileName << std::endl;
					Mat src = cv::imread(file_path, cv::IMREAD_COLOR);
					std::cout << "modelKey.size() :" << modelKey.size() << std::endl;
					for (size_t i = 0; i < modelKey.size(); i++)
					{
						PaintPoint paintPoint = modelKey[i];
						circle(src, paintPoint.point, 2, Scalar(0, 0, 255), 2, 8, 0);
					}
					std::cout << "corners.size():" << corners.size() << std::endl;
					for (size_t j = 0; j < corners.size(); j++) {
						circle(src, corners[j].point, 2, Scalar(0, 255, 255), -1, 8, 0);
					}

					cv::imshow("matchpoint", src);
					cv::waitKey();
				}
	#endif
				double matchVal = sumPercent / (modelKey.size()*1.0);
				double currentMaxHeight = (percentMaxHeight*((maxHeightThresh - abs(max_height - matches.max_height)) / maxHeightThresh));
				currentMaxHeight = currentMaxHeight > percentMaxHeight ? percentMaxHeight : (currentMaxHeight < 0 ? 0 : currentMaxHeight);
				std::cout << "***************************************************************************************************max_height" << max_height << std::endl;
				std::cout << "***************************************************************************************************matches.max_height" << matches.max_height << std::endl;

				double difStablePointDistance = cvgeomerty::distance(matches.stable_point, stablePoint);
				double currentPercentStablePointDistance = (percentStablePoint*((stablePointDistanceThresh - difStablePointDistance) / stablePointDistanceThresh));
				currentPercentStablePointDistance = currentPercentStablePointDistance > percentStablePoint ? percentStablePoint : (currentPercentStablePointDistance < 0 ? 0 : currentPercentStablePointDistance);

				double currentCornerSize = (percentCornerSize*((cornerSizeThresh - abs(modelKey.size() - corners.size())) / cornerSizeThresh));
				currentCornerSize = currentCornerSize > percentCornerSize ? percentCornerSize : (currentCornerSize < 0 ? 0 : currentCornerSize);

				cout << "====matchVal:" << matchVal << "====currentMaxHeight:" << currentMaxHeight << "====currentPercentStablePointDistance:" << currentPercentStablePointDistance << matches.stable_point << stablePoint << "====currentCornerSize:" << currentCornerSize << endl;

				matchVal = matchVal + currentMaxHeight + currentPercentStablePointDistance+ currentCornerSize;
				//int difKeyCount = corners.size() - modelKey.size();//致命因子超过一定多余点计入概率
				//matchVal = matchVal / (difKeyCount < 2 ? (difKeyCount > 0 ? 1.05 : 1) : (0.666*difKeyCount));
				//if (difKeyCount < 0)
				//{
				//	matchVal = 0;
				//}
				cout << "matchVal:" << matchVal << endl;
				if (matchVal > maxMatchVal&&matchVal > 0.75)
				{
					maxMatchVal = matchVal;
					maxMatchName = fileName;
					maxMatches = matches;

					affineMat = getRotationMatrix2D(stablePoint, -outMatchAngle, 1);
					cout << affineMat << endl;
				}
			}

		}
		outMatchVal = maxMatchVal;
		outMatchMat = "";
		if (maxMatchName.length() > 1 && affineMat.cols > 0 && affineMat.rows > 0)
		{
			for (size_t i = 0; i < affineMat.rows; i++)
			{
				vector<double> b;
				for (size_t j = 0; j < affineMat.cols; j++)
				{
					double s = affineMat.at<double>(i, j);

					if (i == 0 && j == 2)
					{
						if ((stablePoint.x == 0 && stablePoint.y == 0) || (maxMatches.stable_point.x == 0 && maxMatches.stable_point.y == 0))
						{
							outMatchMat = outMatchMat + "," + to_string(s);
						}
						else {
							cout << stablePoint << endl;
							outMatchMat = outMatchMat + "," + to_string(s + stablePoint.x - maxMatches.stable_point.x);
						}
					}
					else if (i == 1 && j == 2) {
						if ((stablePoint.x == 0 && stablePoint.y == 0) || (maxMatches.stable_point.x == 0 && maxMatches.stable_point.y == 0))
						{
							outMatchMat = outMatchMat + "," + to_string(s);
						}
						else {
							outMatchMat = outMatchMat + "," + to_string(s + stablePoint.y - maxMatches.stable_point.y);
						}
					}
					else {
						outMatchMat = outMatchMat + (j > 0 ? "," : "") + to_string(s);
					}
				}
				if (i != affineMat.rows - 1)
				{
					outMatchMat = outMatchMat + "_";
				}
			}
		}

		return maxMatchName;
	}
	vector<PaintPoint> matchKeyPointToCorners(const string &file_path, string &matchName, double &matchVal,double &matchAngle, string &matchMat, bool &no_plane, cv::Point *outStablePoint, double minAngleThresh, double minIntervalThresh, double minDistanceThresh, double planeWidthThresh, double novangle, double minPointDistanceThresh, double leftNoRPThresh, bool skeletonImage) {
		
		vector<Point3f> originCorners;
		cvgeomerty::LINE maxDistanceLine;
		int res = getPointCloudCrossOriginCorners(originCorners, no_plane, maxDistanceLine, file_path, minAngleThresh, minIntervalThresh, minDistanceThresh, planeWidthThresh, novangle, minPointDistanceThresh, skeletonImage);
		if (res == -1) {
			LOG(ERRORL) << "getPointCloudCrossOriginCorners错误";
			throw exception((std::string(__FUNCTION__) + ":getPointCloudCrossOriginCorners错误").c_str());
		}
		vector<PaintPoint> matchCorners = getPointCloudCrossMatchCorners(originCorners, no_plane, maxDistanceLine, file_path, minAngleThresh, minIntervalThresh, minDistanceThresh, planeWidthThresh, novangle, minPointDistanceThresh, leftNoRPThresh);
		vector<PaintPoint> matchCorners_tmp;
		matchCorners_tmp = matchCorners;
		vector<PaintPoint> corners = getPointCloudCrossPaintCorners(matchCorners_tmp, no_plane, maxDistanceLine, file_path, minDistanceThresh, planeWidthThresh, novangle, minPointDistanceThresh, leftNoRPThresh);
		string matchKey = odaMatchKeyPoint(file_path, matchCorners, saveModelPath, matchVal, matchAngle, matchMat, outStablePoint);
		cout << "matchkey:" << matchKey << endl;
		if (matchKey.length() > 0)
		{
			matchName = matchKey;
		}
		else {
			matchName = "";
		}
		return corners;

	}
}