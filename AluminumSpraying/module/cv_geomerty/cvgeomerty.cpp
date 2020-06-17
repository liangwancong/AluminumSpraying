#include "cvgeomerty.h"

#include<cstdlib>
#include<cmath>
#include<cstdio>
#include<algorithm>
#include <limits>

#include <opencv2/opencv.hpp>    
#include <opencv2/core/core.hpp> 


namespace cvgeomerty {
	using namespace std;
	using namespace cv;

	/******************************************************************************************
	计算两点之间的距离
	*******************************************************************************************/

	double distance(Point2f p1, Point2f p2) {
		return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
	}


	/******************************************************************************

	r=multiply(sp,ep,op),得到(sp-op)和(ep-op)的叉积

	r>0：ep在矢量opsp的逆时针方向；

	r=0：opspep三点共线；

	r<0：ep在矢量opsp的顺时针方向

	*******************************************************************************/

	double multiply(Point2f sp, Point2f ep, Point2f op)

	{

		return((sp.x - op.x)*(ep.y - op.y) - (ep.x - op.x)*(sp.y - op.y));

	}

	/******************************************************************************

	r=dotmultiply(p1,p2,op),得到矢量(p1-op)和(p2-op)的点积，如果两个矢量都非零矢量

	r<0：两矢量夹角为锐角；

	r=0：两矢量夹角为直角；

	r>0：两矢量夹角为钝角

	*******************************************************************************/

	double dotmultiply(Point2f p1, Point2f p2, Point2f p0)

	{

		return ((p1.x - p0.x)*(p2.x - p0.x) + (p1.y - p0.y)*(p2.y - p0.y));

	}


	/* 判断点与线段的关系,用途很广泛

	本函数是根据下面的公式写的，P是点C到线段AB所在直线的垂足



	AC dot AB

	r =     ---------

	||AB||^2

	(Cx-Ax)(Bx-Ax) + (Cy-Ay)(By-Ay)

	= -------------------------------

	L^2



	r has the following meaning:



	r=0      P = A

	r=1      P = B

	r<0		 P is on the backward extension of AB

	r>1      P is on the forward extension of AB

	0<r<1	 P is interior to AB

	*/

	double relation(Point2f p, LINE l)

	{

		return dotmultiply(p, l.end, l.start) / pow(distance(l.start, l.end), 2);
	}

	// 求点C到线段AB所在直线的垂足 P 

	Point2f perpendicular(Point2f p, LINE l)

	{

		double r = relation(p, l);

		Point2f tp;

		tp.x = l.start.x + r * (l.end.x - l.start.x);

		tp.y = l.start.y + r * (l.end.y - l.start.y);

		return tp;

	}

	/* 求点p到线段l的最短距离,并返回线段上距该点最近的点np

	注意：np是线段l上到点p最近的点，不一定是垂足 */
	double pointToLineSegDistance(Point2f p, LINE l, Point2f &np)

	{

		double r = relation(p, l);

		if (r < 0)

		{

			np = l.start;

			return distance(p, l.start);

		}

		if (r > 1)

		{

			np = l.end;

			return distance(p, l.end);

		}

		np = perpendicular(p, l);

		return distance(p, np);

	}

	// 求点p到线段l所在直线的距离,请注意本函数与上个函数的区别  
	double pointToLineDistance(Point2f p, LINE l)
	{
		return abs(multiply(p, l.end, l.start)) / distance(l.start, l.end);
	}

	bool fitLineSeg(vector<LINE> lines, double min_distance, double min_interval, double min_angle, vector<LINE> &outlines, bool asc) {
		vector<LINE> originLines = lines;
		int originLinesSize = originLines.size();
		
		vector<LINE> newLines ;
		int lastLinesSize = originLinesSize;
		while (lastLinesSize != newLines.size())
		{
			if (originLines.size()<1)
			{
				originLines = newLines;
				lastLinesSize = originLines.size();
				newLines.clear();
			}

			LINE l1 = originLines[originLines.size() - 1];
			originLines.pop_back();

			vector<LINE>::iterator it = originLines.begin();

			while (it != originLines.end())
			{
				LINE l2 = *it;

				LINE newLine;

				if (fitLineSeg(l1, l2, min_distance, min_interval, min_angle, newLine, asc))
				{
					l1 = newLine;

					originLines.erase(it);
					if (originLines.size() < 1)
					{
						break;
					}
					it = originLines.begin();
				}
				it++;
			}
			newLines.push_back(l1);
		}

		outlines = newLines;

		if (originLinesSize == outlines.size() || outlines.size() < 1 )
		{
			cout << "break" << endl;
			return false;
		}
		return true;
	}

	/******************************************************************************************
	l1:线段1
	l2:线段2
	min_distance:最小线段间距离阈值  用于判断平行
	min_interval:最小线段端点间距阈值  用于判断是不是一条
	min_angle:最小线段夹角 用于判断平行
	outline:输出拟合线段
	outoverlapline:输出重叠线段
	asc:端点升序
	need_out_overlapline:是否需要输出重叠线段
	******************************************************************************************/
	//bool fitLineSeg(LINE l1, LINE l2, double min_distance, double min_interval, double min_angle, LINE &outline, LINE &outoverlapline, bool asc, bool need_out_overlapline) {
	//	Point2f l1sTol2p;
	//	double l1sTol2 = pointToLineSegDistance(l1.start, l2, l1sTol2p);//线段1起始点到线段2的最短距离
	//	double rl1sTol2 = pointToLineDistance(l1.start, l2);//线段1起始点到直线2的最短距离

	//	Point2f l1eTol2p;
	//	double l1eTol2 = pointToLineSegDistance(l1.end, l2, l1eTol2p);
	//	double rl1eTol2 = pointToLineDistance(l1.end, l2);

	//	if (rl1sTol2 < min_distance || rl1eTol2 < min_distance)
	//	{
	//		double realAngle = abs(lsangle(l1, l2) * 180 / CV_PI);
	//		if (realAngle < min_angle || 180 - realAngle < min_angle)
	//		{
	//			LINE l1sTol2s = LINE(l1.start, l2.start);
	//			LINE l1sTol2e = LINE(l1.start, l2.end);
	//			LINE l1eTol2s = LINE(l1.end, l2.start);
	//			LINE l1eTol2e = LINE(l1.end, l2.end);

	//			double l1sTol2sDistance = distance(l1sTol2s.start, l1sTol2s.end);
	//			double l1sTol2eDistance = distance(l1sTol2e.start, l1sTol2e.end);
	//			double l1eTol2sDistance = distance(l1eTol2s.start, l1eTol2s.end);
	//			double l1eTol2eDistance = distance(l1eTol2e.start, l1eTol2e.end);

	//			Point2f midl1p = getMidPoint(l1.start, l1.end);
	//			Point2f midl2p = getMidPoint(l2.start, l2.end);

	//			double l1distance = distance(l1.start, l1.end);
	//			double l2distance = distance(l2.start, l2.end);
	//			double middistance = distance(midl1p, midl2p);
	//			if (middistance>((l1distance+ l2distance)/2.+ min_interval))
	//			{
	//				return false;
	//			}

	//			vector<LINE> allLine;
	//			allLine.push_back(l1);
	//			allLine.push_back(l2);
	//			allLine.push_back(l1sTol2s);
	//			allLine.push_back(l1sTol2e);
	//			allLine.push_back(l1eTol2s);
	//			allLine.push_back(l1eTol2e);

	//			auto comp = [](
	//				LINE &a, const LINE &b) {
	//				double distace1 = distance(a.start, a.end);
	//				double distace2 = distance(b.start, b.end);

	//				return distace1 < distace2;
	//			};
	//			std::sort(allLine.begin(), allLine.end(), comp);

	//			LINE fitLine = allLine[allLine.size() - 1];

	//			LINE min1Line = allLine[0];
	//			LINE min2Line = allLine[1];
	//			LINE minLongLine = LINE(Point2f((min1Line.start.x + min1Line.end.x) / 2., (min1Line.start.y + min1Line.end.y) / 2.), Point2f((min2Line.start.x + min2Line.end.x) / 2., (min2Line.start.y + min2Line.end.y) / 2.));

	//			if (distance(min1Line.start, min1Line.end)<min_distance && distance(min2Line.start, min2Line.end)<min_distance && abs(distance(fitLine.start, fitLine.end)- distance(minLongLine.start, minLongLine.end))<min_distance)
	//			{
	//				fitLine = minLongLine;
	//			}

	//			if (fitLine.start.x > fitLine.end.x)
	//			{
	//				fitLine = LINE(fitLine.end, fitLine.start);
	//			}
	//			else
	//			{
	//				if (fitLine.start.x == fitLine.end.x&&fitLine.start.y > fitLine.end.y)
	//				{
	//					fitLine = LINE(fitLine.end, fitLine.start);
	//				}
	//			}

	//			if (!asc)
	//			{
	//				fitLine = LINE(fitLine.end, fitLine.start);

	//			}
	//			outline = fitLine;

	//			if (
	//				need_out_overlapline &&
	//				(rl1sTol2 == l1sTol2 || rl1eTol2 == l1eTol2)
	//				)
	//			{
	//				LINE overlapline = *allLine.begin();
	//				if (overlapline.start.x > overlapline.end.x)
	//				{
	//					overlapline = LINE(overlapline.end, overlapline.start);
	//				}
	//				else
	//				{
	//					if (overlapline.start.x == overlapline.end.x&&overlapline.start.y > overlapline.end.y)
	//					{
	//						overlapline = LINE(overlapline.end, overlapline.start);
	//					}
	//				}

	//				if (!asc)
	//				{
	//					overlapline = LINE(overlapline.end, overlapline.start);
	//				}
	//				outoverlapline = overlapline;
	//			}
	//			return true;
	//		}
	//	}
	//	return false;
	//}


bool fitLineSeg(LINE l1, LINE l2, double min_distance, double min_interval, double min_angle, LINE &outline, LINE &outoverlapline, bool asc, bool need_out_overlapline) {
	Point2f l1sTol2p;
	double l1sTol2 = pointToLineSegDistance(l1.start, l2, l1sTol2p);//线段1起始点到线段2的最短距离
	double rl1sTol2 = pointToLineDistance(l1.start, l2);//线段1起始点到直线2的最短距离

	Point2f l1eTol2p;
	double l1eTol2 = pointToLineSegDistance(l1.end, l2, l1eTol2p);
	double rl1eTol2 = pointToLineDistance(l1.end, l2);


	//起始点
	if (rl1sTol2 < min_distance && rl1eTol2 < min_distance)
	{
		double realAngle = abs(lsangle(l1, l2) * 180 / CV_PI);
		if (realAngle < min_angle || 180 - realAngle < min_angle)
		{
			LINE l1sTol2s = LINE(l1.start, l2.start);
			LINE l1sTol2e = LINE(l1.start, l2.end);
			LINE l1eTol2s = LINE(l1.end, l2.start);
			LINE l1eTol2e = LINE(l1.end, l2.end);

			double l1sTol2sDistance = distance(l1sTol2s.start, l1sTol2s.end);
			double l1sTol2eDistance = distance(l1sTol2e.start, l1sTol2e.end);
			double l1eTol2sDistance = distance(l1eTol2s.start, l1eTol2s.end);
			double l1eTol2eDistance = distance(l1eTol2e.start, l1eTol2e.end);

			Point2f midl1p = getMidPoint(l1.start, l1.end);
			Point2f midl2p = getMidPoint(l2.start, l2.end);

			double l1distance = distance(l1.start, l1.end);
			double l2distance = distance(l2.start, l2.end);
			double middistance = distance(midl1p, midl2p);
			if (middistance > ((l1distance + l2distance) / 2. + min_interval))
			{
				return false;
			}

			vector<LINE> allLine;
			allLine.push_back(l1);
			allLine.push_back(l2);
			allLine.push_back(l1sTol2s);
			allLine.push_back(l1sTol2e);
			allLine.push_back(l1eTol2s);
			allLine.push_back(l1eTol2e);

			auto comp = [](
				LINE &a, const LINE &b) {
				double distace1 = distance(a.start, a.end);
				double distace2 = distance(b.start, b.end);

				return distace1 < distace2;
			};
			std::sort(allLine.begin(), allLine.end(), comp);

			LINE fitLine = allLine[allLine.size() - 1];

			LINE min1Line = allLine[0];
			LINE min2Line = allLine[1];
			LINE minLongLine = LINE(Point2f((min1Line.start.x + min1Line.end.x) / 2., (min1Line.start.y + min1Line.end.y) / 2.), Point2f((min2Line.start.x + min2Line.end.x) / 2., (min2Line.start.y + min2Line.end.y) / 2.));

			if (distance(min1Line.start, min1Line.end) < min_distance && distance(min2Line.start, min2Line.end) < min_distance && abs(distance(fitLine.start, fitLine.end) - distance(minLongLine.start, minLongLine.end)) < min_distance)
			{
				fitLine = minLongLine;
			}

			if (fitLine.start.x > fitLine.end.x)
			{
				fitLine = LINE(fitLine.end, fitLine.start);
			}
			else
			{
				if (fitLine.start.x == fitLine.end.x&&fitLine.start.y > fitLine.end.y)
				{
					fitLine = LINE(fitLine.end, fitLine.start);
				}
			}
#ifdef DEBUG
		{
			std::cout << "*****" << std::endl;
			std::cout << "rl1sTol2:" << rl1sTol2 << std::endl;
			std::cout << "rl1eTol2:" << rl1eTol2 << std::endl;
			cv::Mat src1 = cv::imread("E:\\project\\AluminumProfileSpraying\\RangeImage\\build\\Release\\aluminum\\trainImage\\tmp.jpg", cv::IMREAD_COLOR);
			cvgeomerty::LINE currentCVLine = cvgeomerty::LINE(l1);
			Point2f p1 = currentCVLine.start;
			Point2f p2 = currentCVLine.end;
			line(src1, p1, p2, Scalar(0, 0, 255), 2, LINE_AA);
			currentCVLine = cvgeomerty::LINE(l2);
			p1 = currentCVLine.start;
			p2 = currentCVLine.end;
			line(src1, p1, p2, Scalar(0, 255, 0), 2, LINE_AA);

			p1 = fitLine.start;
			p2 = fitLine.end;
			line(src1, p1, p2, Scalar(0, 255, 255), 2, LINE_AA);
			imshow("groupline", src1);
			waitKey(0);
		}
#endif



			if (!asc)
			{
				fitLine = LINE(fitLine.end, fitLine.start);

			}
			outline = fitLine;

			if (
				need_out_overlapline &&
				(rl1sTol2 == l1sTol2 || rl1eTol2 == l1eTol2)
				)
			{
				LINE overlapline = *allLine.begin();
				if (overlapline.start.x > overlapline.end.x)
				{
					overlapline = LINE(overlapline.end, overlapline.start);
				}
				else
				{
					if (overlapline.start.x == overlapline.end.x&&overlapline.start.y > overlapline.end.y)
					{
						overlapline = LINE(overlapline.end, overlapline.start);
					}
				}

				if (!asc)
				{
					overlapline = LINE(overlapline.end, overlapline.start);
				}
				outoverlapline = overlapline;
			}
			return true;
		}
	}
	return false;
}
	//拟合线段
	bool fitLineSeg(LINE l1, LINE l2, double min_distance, double min_interval, double min_angle, LINE &outline, bool asc) {
		LINE outoverlapline;
		return fitLineSeg(l1, l2, min_distance, min_interval, min_angle, outline, outoverlapline, asc, false);

	}


	/* 返回顶角在o点，起始边为os，终止边为oe的夹角(单位：弧度)

	角度小于pi，返回正值

	角度大于pi，返回负值

	可以用于求线段之间的夹角

	原理：

	r = dotmultiply(s,e,o) / (dist(o,s)*dist(o,e))

	r'= multiply(s,e,o)



	r >= 1	angle = 0;

	r <= -1	angle = -PI

	-1<r<1 && r'>0	angle = arccos(r)

	-1<r<1 && r'<=0	angle = -arccos(r)

	*/

	double angle(Point2f o, Point2f s, Point2f e)

	{

		double cosfi, fi, norm;

		double dsx = s.x - o.x;

		double dsy = s.y - o.y;

		double dex = e.x - o.x;

		double dey = e.y - o.y;



		cosfi = dsx * dex + dsy * dey;

		norm = (dsx*dsx + dsy * dsy)*(dex*dex + dey * dey);

		cosfi /= sqrt(norm);



		if (cosfi >= 1.0) return 0;

		if (cosfi <= -1.0) return -CV_PI;



		fi = acos(cosfi);

		if (dsx*dey - dsy * dex > 0) return fi;      // 说明矢量os 在矢量 oe的顺时针方向 

		return -fi;

	}

	// 返回线段l1与l2之间的夹角 单位：弧度 范围(-pi，pi) 

	double lsangle(LINE l1, LINE l2)

	{

		Point2f o, s, e;

		o.x = o.y = 0;

		s.x = l1.end.x - l1.start.x;

		s.y = l1.end.y - l1.start.y;

		e.x = l2.end.x - l2.start.x;

		e.y = l2.end.y - l2.start.y;

		return angle(o, s, e);

	}

	/*
	获取两个点之间的中点
	*/
	Point2f getMidPoint(Point2f p1, Point2f p2) {
		return Point2f((p1.x + p2.x) / 2., (p1.y + p2.y) / 2.);
	}

	/*
	获取直线的垂线角度
	*/
	double lvangle(LINE line) {
		double k = line.b / line.a;
		double theta = atan(k);
		return theta;
	}



	/* 返回两个矢量l1和l2的夹角的余弦(-1 --- 1)注意：如果想从余弦求夹角的话，注意反余弦函数的定义域是从 0到pi */

	double cosine(LINE l1, LINE l2)

	{

		return (((l1.end.x - l1.start.x)*(l2.end.x - l2.start.x) +

			(l1.end.y - l1.start.y)*(l2.end.y - l2.start.y)) / (distance(l1.end, l1.start)*distance(l2.end, l2.start)));

	}

	/**********************************************

	寻找凸包的graham 扫描法

	PointSet为输入的点集；

	ch为输出的凸包上的点集，按照逆时针方向排列;

	n为PointSet中的点的数目

	len为输出的凸包上的点的个数

	**********************************************/

	void grahamScan(Point2f PointSet[], Point2f ch[], int n, int &len)

	{

		int i, j, k = 0, top = 2;

		Point2f tmp;

		// 选取PointSet中y坐标最小的点PointSet[k]，如果这样的点有多个，则取最左边的一个 

		for (i = 1; i<n; i++)

			if (PointSet[i].y<PointSet[k].y || (PointSet[i].y == PointSet[k].y) && (PointSet[i].x<PointSet[k].x))

				k = i;

		tmp = PointSet[0];

		PointSet[0] = PointSet[k];

		PointSet[k] = tmp; // 现在PointSet中y坐标最小的点在PointSet[0] 

		for (i = 1; i<n - 1; i++) /* 对顶点按照相对PointSet[0]的极角从小到大进行排序，极角相同的按照距离PointSet[0]从近到远进行排序 */

		{

			k = i;

			for (j = i + 1; j<n; j++)

				if (multiply(PointSet[j], PointSet[k], PointSet[0])>0 ||  // 极角更小    

					(multiply(PointSet[j], PointSet[k], PointSet[0]) == 0) && /* 极角相等，距离更短 */

					distance(PointSet[0], PointSet[j])<distance(PointSet[0], PointSet[k])

					)

					k = j;

			tmp = PointSet[i];

			PointSet[i] = PointSet[k];

			PointSet[k] = tmp;

		}

		ch[0] = PointSet[0];

		ch[1] = PointSet[1];

		ch[2] = PointSet[2];

		for (i = 3; i<n; i++)

		{

			while (multiply(PointSet[i], ch[top], ch[top - 1]) >= 0)

				top--;

			ch[++top] = PointSet[i];

		}

		len = top + 1;

	}

	// 卷包裹法求点集凸壳，参数说明同graham算法    
	void convexClosure(Point2f PointSet[], Point2f ch[], int n, int &len)

	{
		int top = 0, i, index, first;

		double curmax, curcos, curdis;

		Point2f tmp;

		LINE l1, l2;

		bool use[300];

		tmp = PointSet[0];

		index = 0;

		// 选取y最小点，如果多于一个，则选取最左点 

		for (i = 1; i<n; i++)

		{

			if (PointSet[i].y<tmp.y || PointSet[i].y == tmp.y&&PointSet[i].x<tmp.x)

			{

				index = i;

			}

			use[i] = false;

		}

		tmp = PointSet[index];

		first = index;

		use[index] = true;



		index = -1;

		ch[top++] = tmp;

		tmp.x -= 100;

		l1.start = tmp;

		l1.end = ch[0];

		l2.start = ch[0];



		while (index != first)

		{

			curmax = -100;

			curdis = 0;

			// 选取与最后一条确定边夹角最小的点，即余弦值最大者 

			for (i = 0; i<n; i++)

			{

				if (use[i])continue;

				l2.end = PointSet[i];

				curcos = cosine(l1, l2); // 根据cos值求夹角余弦，范围在 （-1 -- 1 ） 

				if (curcos>curmax || fabs(curcos - curmax)<1e-6 && distance(l2.start, l2.end)>curdis)

				{

					curmax = curcos;

					index = i;

					curdis = distance(l2.start, l2.end);

				}

			}

			use[first] = false;            //清空第first个顶点标志，使最后能形成封闭的hull 

			use[index] = true;

			ch[top++] = PointSet[index];

			l1.start = ch[top - 2];

			l1.end = ch[top - 1];

			l2.start = ch[top - 1];

		}

		len = top - 1;
	}

	// 返回多边形面积(signed)；输入顶点按逆时针排列时，返回正值；否则返回负值 
	double areaOfPolygon(vector<Point2f> polygon)

	{
		int vcount = polygon.size();

		int i;

		double s;

		if (vcount<3)

			return 0;

		s = polygon[0].y*(polygon[vcount - 1].x - polygon[1].x);

		for (i = 1; i<vcount; i++)

			s += polygon[i].y*(polygon[(i - 1)].x - polygon[(i + 1) % vcount].x);

		return s / 2;

	}


	/*
	*	
	*	获取图片PCA主方向
	*	
	*	gary: 灰度图
	*	eigenVectorsPCA(2):  特征向量ve(2x2)	
	*	eigenValuesPCA(2): 特征值va(2x1)
	*	pos: 求均值, (第一列一行, 第一列第二行)
	*/
	double getPCAAngle(Mat gary, vector<Point2d> &eigenVectorsPCA, vector<double> &eigenValuesPCA, Point &pos)
	{
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		double maxArea = 0;
		vector<Point> maxContor;
		//轮廓提取
		findContours(gary, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
		for (size_t i = 0; i < contours.size(); ++i)
		{
			//计算轮廓大小
			double area = contourArea(contours[i]);
			//去除过小或者过大的轮廓区域（科学计数法表示）
			if (area < 1e2 || 1e5 < area) continue;

			if (area > maxArea)
			{
				maxArea = area;
				maxContor = contours[i];
			}
		}

		//构建pca数据。这里做的是将轮廓点的x和y作为两个维压到data_pts中去。
		Mat data_pts = Mat(maxContor.size(), 2, CV_64FC1);//使用mat来保存数据，也是为了后面pca处理需要
		for (int i = 0; i < data_pts.rows; ++i)
		{
			data_pts.at<double>(i, 0) = maxContor[i].x;
			data_pts.at<double>(i, 1) = maxContor[i].y;
		}
		//执行PCA分析
		if (data_pts.rows==0|| data_pts.cols==0)
		{
			return 0;
		}
		PCA pca_analysis(data_pts, Mat(), CV_PCA_DATA_AS_ROW);

		// 求均值, 第一列一行                第一列第二行
		pos = Point(static_cast<int>(pca_analysis.mean.at<double>(0, 0)), static_cast<int>(pca_analysis.mean.at<double>(0, 1)));

		for (int i = 0; i < 2; ++i)
		{
			eigenVectorsPCA[i] = Point2d(pca_analysis.eigenvectors.at<double>(i, 0), pca_analysis.eigenvectors.at<double>(i, 1));
			eigenValuesPCA[i] = pca_analysis.eigenvalues.at<double>(i, 0);
		}

		double angle = atan2(eigenVectorsPCA[0].y, eigenVectorsPCA[0].x); //反三角函数求角度

		return angle * (180 / CV_PI);
	}
/*
*
*	两条直线的交点
*
*/
	cv::Point2f getLineCrossPoint(cv::Vec4f line1, cv::Vec4f line2) {
		cv::Point2f point;
		if (line1[1] == 1 && line2[1] == 1)
			;
		else if (line1[1] == 1) {
			double k2 = line2[1] / line2[0];
			point.x = line1[2];
			point.y = k2 * (line1[2] - line2[2]) + line2[3];
		}
		else if (line2[1] == 1) {
			double k1 = line1[1] / line1[0];
			point.x = line2[2];
			point.y = k1 * (line2[2] - line1[2]) + line1[3];
		}
		else {
			double k1 = line1[1] / line1[0];
			double k2 = line2[1] / line2[0];

			double x1 = line1[2], x2 = line2[2];
			double y1 = line1[3], y2 = line2[3];

			point.x = (y2 - y1 + k1 * x1 - k2 * x2) / (k1 - k2);
			point.y = (k1*y2 - k2 * y1 + k1 * k2*(x1 - x2)) / (k1 - k2);
		}
		return point;
	}
/*
	  求二条直线的交点的公式
	  有如下方程    a1*x+b1*y=c1
						   a2*x+b2*y=c2
	                       x= | c1 b1|  / | a1 b1 |      y= | a1 c1| / | a1 b1 |
	                            | c2 b2|  / | a2 b2 |			 | a2 c2| / | a2 b2 |
	
	   a1= (L1.pEnd.y-L1.pStart.y)   
	   b1= (L1.pEnd.x-L1.pStart.x)
	   c1= L1.pStart.x*(L1.pEnd.y-L1.pStart.y)-(L1.pEnd.x-L1.pStart.x)*L1.pStart.y
	   a2= (L2.pEnd.y-L2.pStart.y)   
	   b2= (L2.pEnd.x-L2.pStart.x)
	   c2= L2.pStart.x*(L2.pEnd.y-L2.pStart.y)-(L2.pEnd.x-L2.pStart.x)*L2.pStart.y  
	定义两个结构体方便理解
*/
	cv::Point getCrossPoint(const LINE line1, const LINE line2)
	{
		//    if(!SegmentIntersect(line1->pStart, line1->pEnd, line2->pStart, line2->pEnd))
		//    {// segments not cross   
		//        return 0;
		//    }
		Point pt;
		// line1's cpmponent
		double X1 = line1.end.x - line1.start.x;//b1
		double Y1 = line1.end.y - line1.start.y;//a1
		// line2's cpmponent
		double X2 = line2.end.x - line2.start.x;//b2
		double Y2 = line2.end.y - line2.start.y;//a2
		// distance of 1,2
		double X21 = line2.start.x - line1.start.x;
		double Y21 = line2.start.y - line1.start.y;
		// determinant
		double D = Y1 * X2 - Y2 * X1;// a1b2-a2b1
		// 
		if (D == 0) return Point(0, 0);
		// cross point
		pt.x = (X1*X2*Y21 + Y1 * X2*line1.start.x - Y2 * X1*line2.start.x) / D;
		// on screen y is down increased ! 
		pt.y = -(Y1*Y2*X21 + X1 * Y2*line1.start.y - X2 * Y1*line2.start.y) / D;
		// segments intersect.
		if ((abs(pt.x - line1.start.x - X1 / 2) <= abs(X1 / 2)) &&
			(abs(pt.y - line1.start.y - Y1 / 2) <= abs(Y1 / 2)) &&
			(abs(pt.x - line2.start.x - X2 / 2) <= abs(X2 / 2)) &&
			(abs(pt.y - line2.start.y - Y2 / 2) <= abs(Y2 / 2)))
		{
			return pt;
		}
		return pt;
	}
}