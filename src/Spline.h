#pragma once

#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

class SplineKnot
{
public:
	SplineKnot();
	SplineKnot(const cv::Point2f & pos, const cv::Point2f & ta, const cv::Point2f & tb);
	~SplineKnot();
	


	cv::Point2f m_position;
	cv::Point2f m_ta;
	cv::Point2f m_tb;
};

class Spline
{
public:

	Spline(const int capacity = 2, const int accuracy = 10);
	~Spline();

	void insertKnot();
	void insertKnot(const size_t nb);
	void insertKnot(const cv::Point2f & pos, const cv::Point2f & ta, const cv::Point2f & tb);
	cv::Point2f qLerp(const size_t kaid, const size_t kbid, const float t);
	void drawSpline(cv::Mat image, const cv::Scalar &color);

	int m_accuracy;							// Precision
	std::vector<SplineKnot>m_splineKnots;	// liste des noeuds de la spline.( A priori seulement deux devraient suffire pour le Robot.)
};