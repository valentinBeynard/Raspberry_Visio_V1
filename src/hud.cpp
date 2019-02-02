#include "hud.h"
#include "Vision.h"

extern Vision  vision;

void hud::drawSight(cv::Mat & image, const float extend)
{
	Point2f v0, v1;


	v0.x = vision.getIntrinsicScreenCenterX() - extend;	v0.y = vision.getIntrinsicScreenCenterY();
	v1.x = vision.getIntrinsicScreenCenterX() + extend;	v1.y = v0.y;
	line(image, v0, v1, Scalar(0, 0, 255));

	v0.x = vision.getIntrinsicScreenCenterX();			v0.y = vision.getIntrinsicScreenCenterY() - extend;
	v1.x = v0.x;										v1.y = vision.getIntrinsicScreenCenterY() + extend;
	line(image, v0, v1, Scalar(0, 0, 255));
}

void hud::drawVisionTargetRotatedRectangle(cv::InputOutputArray image, VisionTarget &visiontarget)
{
	// 01: Vertical axis
	cv::line(image, visiontarget.point[0], visiontarget.point[(1)], Scalar(50, 128, 50), 1);
	// 12
	cv::line(image, visiontarget.point[1], visiontarget.point[(2)], Scalar(100, 100, 100), 1);
	// 23
	cv::line(image, visiontarget.point[2], visiontarget.point[(3)], Scalar(100, 100, 100), 1);
	// 30: Horizontal axis
	cv::line(image, visiontarget.point[3], visiontarget.point[(0)], Scalar(50, 50, 128), 1);

	cv::putText(image, "0", visiontarget.point[0], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));
	cv::putText(image, "1", visiontarget.point[1], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));
	cv::putText(image, "2", visiontarget.point[2], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 128, 128));
	cv::putText(image, "3", visiontarget.point[3], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));
}

void hud::drawVisionTargetCoupleQuad(cv::InputOutputArray image, VisionTargetPair &vtcouple)
{
	// Version standard
	cv::line(image, vtcouple.m_pvisionTargetA->m_higher2D, vtcouple.m_pvisionTargetB->m_lower2D, Scalar(0, 0, 255), 1);
	cv::line(image, vtcouple.m_pvisionTargetA->m_lower2D, vtcouple.m_pvisionTargetB->m_higher2D, Scalar(0, 0, 255), 1);

	// version test
	if (vtcouple.m_quality < 0.0f)
	{
		// Diagonales
		cv::line(image, vtcouple.m_pvisionTargetA->m_higher2D, vtcouple.m_pvisionTargetB->m_lower2D, Scalar(0, 0, 128), 1);
		cv::line(image, vtcouple.m_pvisionTargetA->m_lower2D, vtcouple.m_pvisionTargetB->m_higher2D, Scalar(0, 0, 128), 1);
	}
	else
	{
		double val = 255.0f*vtcouple.m_quality;
		// Diagonales
		cv::line(image, vtcouple.m_pvisionTargetA->m_higher2D, vtcouple.m_pvisionTargetB->m_lower2D, Scalar(0, val, val / 2), 2);
		cv::line(image, vtcouple.m_pvisionTargetA->m_lower2D, vtcouple.m_pvisionTargetB->m_higher2D, Scalar(0, val, val / 2), 2);

		char	str[32];
		Point2f v0 = vtcouple.m_center[VisionTargetPair::TRAPEZOID];
		v0.x -= 5;
		v0.y += 50 + vtcouple.m_quality * 10;
		sprintf(str, "%.4f", vtcouple.m_quality);
		cv::putText(image, str, v0, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255));
	}
}

void hud::draw3DBase(cv::InputOutputArray image, const cv::Mat rot_vector, const cv::Mat trans_vector, const float size)
{
	vector<Point3f> base3d = { {0,0,0},{size,0,0},{0,size,0},{0,0,size} };
	vector<Point2f> base2d;

	projectPoints(base3d, rot_vector, trans_vector, vision.m_intrinsic, vision.m_distCoeffs, base2d);

	arrowedLine(image, base2d[0], base2d[1], Scalar(0, 0, 255), 2);
	arrowedLine(image, base2d[0], base2d[2], Scalar(0, 255, 0), 2);
	arrowedLine(image, base2d[0], base2d[3], Scalar(255, 0, 0), 2);

}

void hud::drawPoint(cv::InputOutputArray image, const cv::Point2f & p, const float extend, const cv::Scalar & color)
{
	// Trace une  croix blanche ed taille "extend" � la position p.
	cv::Point2f v0 = p;
	cv::Point2f v1 = p;
	// |
	v0.y -= extend;
	v1.y += extend;
	line(image, v0, v1, color);
	// _
	v0.y = p.y;
	v1.y = p.y;
	v0.x -= extend;
	v1.x += extend;
	line(image, v0, v1, color);
}
