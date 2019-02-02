#pragma once


#include "VisionTarget.h"

#define PAIR_NORM3DTOP			2.5f // distance réelle en mètre séparant les deux shapes d'une même paire.
#define PAIR_NORM3DBOTTOM		3.5f // distance réelle en mètre séparant les deux shapes d'une même paire.
#define PAIR_NORM3DRATIO		(PAIR_NORM3DTOP/PAIR_NORM3DBOTTOM)

class VisionTarget;

class VisionTargetPair
{
public:
	enum enumCenter
	{
		TRAPEZOID = 0,
		TRAPEZOID_BOTTOM_SIDE = 1
	};
	const float assemble(VisionTarget *pshapeA, VisionTarget *pshapeB);
	VisionTarget	*m_pvisionTargetA;
	VisionTarget	*m_pvisionTargetB;


	float			m_norm3DBottom;
	float			m_norm3DTop;

	float			m_quality;				// note de qualité du couple en fonction des résultats aux tests

	cv::Point2f		m_center[2];			// 


	// 3D Output: rotation and translation for Pnp:
	bool			b3Dsolved;				// Pnp return value
	cv::Mat			m_rotation_vector;		// Rotation in axis-angle form
	cv::Mat			m_translation_vector;	// Translation from camera(0,0,0)
};

class VisionTargetChain
{
public:
	std::vector<VisionTarget> m_visionTargets;
	void buildChainGraph();
};
