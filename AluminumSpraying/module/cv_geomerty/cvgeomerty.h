#pragma once

#include <opencv2/opencv.hpp>    
#include <opencv2/core/core.hpp> 

namespace cvgeomerty {
	using namespace cv;

	struct LINE           // 直线的解析方程 a*x+b*y+c=0  为统一表示，约定 a >= 0 

	{
		Point2f start;
		Point2f end;

		double a;
		double b;
		double c;

		double k;

		double angle;

		Vec4f line;

		LINE() {}
		LINE(Point2f _start, Point2f _end) {
			start = _start;
			end = _end;
			line = Vec4f(start.x, start.y, end.x, end.y);
			LINE(line);
		}
		
		LINE(Vec4f cvline) {
			line = cvline;

			start.x = cvline[0];
			start.y = cvline[1];
			end.x = cvline[2];
			end.y = cvline[3];

			k = (start.y - end.y) * 1.0 / (start.x - end.x);

			if (isinf(k))
			{
				a = -1;
				b = 0;
				c = start.x;
			}
			else
			{
				a = k;
				b = -1;
				c = start.y - k * start.x;
			}
			angle = atan(k) * 180 / CV_PI;
		}

	};

	/******************************************************************************************
	 计算两点之间的距离
	*******************************************************************************************/
	double distance(Point2f p1, Point2f p2);

	/******************************************************************************************
	 求点p到线段l的最短距离,并返回线段上距该点最近的点np

	 注意：np是线段l上到点p最近的点，不一定是垂足
	*******************************************************************************************/
	double pointToLineSegDistance(Point2f p, LINE l, Point2f &np);

	/******************************************************************************************
	 根据线段间距离，不重合间距以及角度阈值拟合线段
	*******************************************************************************************/
	bool fitLineSeg(LINE l1, LINE l2, double min_distance, double min_interval, double min_angle, LINE &outline, bool asc = true);
	bool fitLineSeg(std::vector<LINE> lines, double min_distance, double min_interval, double min_angle, std::vector<LINE> &outlines, bool asc = true);
	bool fitLineSeg(LINE l1, LINE l2, double min_distance, double min_interval, double min_angle, LINE &outline, LINE &outoverlapline, bool asc = true, bool need_out_overlapline = true);

	/******************************************************************************************
	 求点p到线段l所在直线的距离
	*******************************************************************************************/
	double pointToLineDistance(Point2f p, LINE l);

	/******************************************************************************************
	 返回线段l1与l2之间的夹角 单位：弧度 范围(-pi，pi)
	*******************************************************************************************/
	double lsangle(LINE l1, LINE l2);

	/******************************************************************************************
	 获取两个点之间的中点
	*******************************************************************************************/
	Point2f getMidPoint(Point2f p1, Point2f p2);

	/******************************************************************************************
	获取直线的垂线角度
	*******************************************************************************************/
	double lvangle(LINE line);

	/******************************************************************************

	r=dotmultiply(p1,p2,op),得到矢量(p1-op)和(p2-op)的点积，如果两个矢量都非零矢量

	r<0：两矢量夹角为锐角；

	r=0：两矢量夹角为直角；

	r>0：两矢量夹角为钝角

	*******************************************************************************/
	double dotmultiply(Point2f p1, Point2f p2, Point2f p0);

	/******************************************************************************

	r=multiply(sp,ep,op),得到(sp-op)和(ep-op)的叉积

	r>0：ep在矢量opsp的逆时针方向；

	r=0：opspep三点共线；

	r<0：ep在矢量opsp的顺时针方向

	*******************************************************************************/
	double multiply(Point2f p1, Point2f p2, Point2f p0);

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
	double relation(Point2f p, LINE l);
	
	/*
	  求点C到线段AB所在直线的垂足 P 
	*/
	Point2f perpendicular(Point2f p, LINE l);

	/**********************************************

	寻找凸包的graham 扫描法

	PointSet为输入的点集；

	ch为输出的凸包上的点集，按照逆时针方向排列;

	**********************************************/

	void grahamScan(Point2f PointSet[], Point2f ch[],int n,int &len);

	// 卷包裹法求点集凸壳，参数说明同graham算法    
	void convexClosure(Point2f PointSet[], Point2f ch[], int n, int &len);

	// 返回多边形面积(signed)；输入顶点按逆时针排列时，返回正值；否则返回负值 
	double areaOfPolygon(std::vector<Point2f> polygon);

	/*
	*
	*	获取图片PCA主方向
	*
	*	gary: 灰度图
	*	eigenVectorsPCA(2):  特征向量ve(2x2)
	*	eigenValuesPCA(2): 特征值va(2x1)
	*	pos: 求均值, (第一列一行, 第一列第二行)
	*/
	double getPCAAngle(Mat gary, std::vector<Point2d> &eigenVectorsPCA, std::vector<double> &eigenValuesPCA, Point &pos);
	/*
*
*	两条直线的交点
*
*/
	cv::Point2f getLineCrossPoint(cv::Vec4f line1, cv::Vec4f line2);
	cv::Point getCrossPoint(const LINE line1, const LINE line2);
}