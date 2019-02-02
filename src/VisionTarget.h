#pragma once


#include "opencv2/opencv.hpp"
#include "Statistics.h"
#include "Vision.h"

class VisionTarget;
typedef struct 
{
	VisionTarget	*m_pto;
	float			m_Gain;			// note de qualité de la paire "from-to"
	float			m_continuity;	// note de continuité ( même longeur, coliméarité ) avec la paire précédente.
}VisionTargetLink;

class VisionTargetPair;

#define VISIONTARGET_LINKSIZE	4
class VisionTarget
{
public:
	enum flags
	{
		FLAG_DISABLE = 0
	};
	//VisionTarget();
	//~VisionTarget();

	//VisionTarget & operator = (const VisionTarget & shape);

	bool estimate3Dposition(Vision *pcontext3D);
	//const Point2f& getHigher2D();
	//const Point2f& getLower2D();
	//bool compare_x(const VisionTarget &shapeA, const VisionTarget &shapeB);
	
	//cv::Rect		boundingRect;
	size_t			contourId;		// contour associé
	cv::Point2f		m_center;		// Vision Target Center of rotated rectangle ( il ne s'agit pas du centre projeté !!! ) 

	cv::Point2f		point[4];
	size_t			m_lowerId;		// Id of the lower point (between the 4)
	size_t			m_higherId;		// Id of the higher point (between the 4)

	cv::Point2f		m_lower2D;
	cv::Point2f		m_higher2D;

	//cv::Point3f		m_center3D;
	cv::Point3f		m_lower3D;
	cv::Point3f		m_higher3D;

	float			norm01;
	float			norm03;
	float			area;
	
	cv::Point2f		n01;		// supposed to be the ROTATED RECT Y axis ( The vertical ONE, oriented from bottom to top )
	cv::Point2f		n03;		// supposed to be the ROTATED RECT X axis ( The horizontal ONE, oriented from left to right )
	

	int				m_score;
	unsigned long	m_flags;

	//cv::Point2f		middleTop;
	//cv::Point2f		middleBottom;
	//cv::Point3f		middleTop3D;
	//cv::Point3f		middleBottom3D;
	//VisionTargetPair	*m_pVisionTargetPair;	// Pointeur sur la classe VisionTargetPair Associée 

	// Pour la construction du chain graph
	VisionTargetLink	m_links[VISIONTARGET_LINKSIZE];
	int					m_linkSize;
	VisionTarget		*m_pChainParent;
};

