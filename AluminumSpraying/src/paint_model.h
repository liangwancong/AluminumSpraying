#pragma once
#include <opencv2/opencv.hpp>    
#include <opencv2/core/core.hpp> 

struct PaintPoint
{
	cv::Point2f point;
	double angle;
	bool needChange;
	PaintPoint() {}
	PaintPoint(cv::Point2f _point, double _angle, bool _needChange) {
		point = _point;
		angle = _angle;
		needChange = _needChange;
	}
};

struct PaintPoints
{
	std::vector<PaintPoint> points;
	std::string picName;
	std::string matchName;
	std::string matchMat;
	cv::Point stablePoint;
	double matchAngle = 0;
	double matchVal=0;
	bool havePlane;
	cv::Point2f maxLineStart;
	cv::Point2f maxLineEnd;
	std::vector<PaintPoint> maxLineContentPoints;

	PaintPoints() {}
	PaintPoints(std::vector<PaintPoint> _points, std::string _picName, std::string _matchName, std::string _matchMat,cv::Point _stablePoint, bool _havePlane, double _matchAngle, double _matchVal, cv::Point2f _maxLineStart, cv::Point2f _maxLineEnd, std::vector<PaintPoint> _maxLineContentPoints) {
		points = _points;
		picName = _picName;
		havePlane = _havePlane;
		matchAngle = _matchAngle;
		matchMat = _matchMat;
		matchName = _matchName;
		stablePoint = _stablePoint;
		matchVal = _matchVal;
		maxLineStart = _maxLineStart;
		maxLineEnd = _maxLineEnd;
		maxLineContentPoints = _maxLineContentPoints;
	}
};

struct MatchMsg
{
	std::string picName;
	double scale = 1;
	std::string matchMat;
	double matchAngle = 0;
	MatchMsg() {}
	MatchMsg(std::string _picName, double _scale, std::string _matchMat, double _matchAngle) {
		picName = _picName;
		scale = _scale;
		matchMat = _matchMat;
		matchAngle = _matchAngle;
	}
};

struct MatchModel
{
	std::vector<PaintPoint> points;
	double max_height;
	cv::Point stable_point;
};

struct MatchModelC
{
	std::vector<PaintPoint> points;
	cv::Point stable_point;
	std::string match_name;
	std::string match_mat;
	std::string pic_name;
	double match_angle;
	std::string have_plane;
};