#pragma once

#include "opencv2/opencv.hpp"
#include "VisionTarget.h"
#include "VisionTargetPair.h"
using namespace cv;


namespace hud
{
	// Dessine un viseur au milieu de l'image
	void drawSight(cv::Mat & image, const float extend);
	// Dessine le rectangle orienté associé à une VisionTarget
	void drawVisionTargetRotatedRectangle(cv::InputOutputArray image, VisionTarget &visiontarget);
	// Dessine de quadrilatere associé au couple de vision target
	void drawVisionTargetCoupleQuad(cv::InputOutputArray image, VisionTargetPair &vtcouple);
	// (projète) et Dessine un repère 3D
	void draw3DBase(cv::InputOutputArray image, const cv::Mat rot_vector, const cv::Mat trans_vector, const float size);
	void drawPoint(cv::InputOutputArray image, const cv::Point2f & p, const float extend, const cv::Scalar & color);
}