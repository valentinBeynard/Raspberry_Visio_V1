#include "VisionTarget.h"
#include "Vision.h"
#include "Values.h"


/*
VisionTarget::~VisionTarget()
{
}

VisionTarget::VisionTarget()
{

}
*/
/*
bool VisionTarget::compare_x(const VisionTarget & va, const VisionTarget & vb)
{
	return  (va.center.x < vb.center.x);
}
*/
/*
VisionTarget & VisionTarget::operator=(const VisionTarget & shapetocopy)
{
}
*/


bool VisionTarget::estimate3Dposition(Vision * pcontext3D)
{
	if (!pcontext3D->estimate3DPoint(m_lower2D, REAL_Y_VISIONTARGETPAIR_BOTTOM, m_lower3D) )
	{
		// On place le point très très loin car il n'a pas pu être calculé 
		m_lower3D.x = -COORD3DMAX;
		m_lower3D.y = -COORD3DMAX;
		m_lower3D.z = -COORD3DMAX;
	}

	if (!pcontext3D->estimate3DPoint(m_higher2D, REAL_Y_VISIONTARGETPAIR_TOP, m_higher3D) )
	{
		// On place le point très très loin car il n'a pas pu être calculé 
		m_higher3D.x = -COORD3DMAX;
		m_higher3D.y = -COORD3DMAX;
		m_higher3D.z = -COORD3DMAX;
	}

	return true;
}

/*

const Point2f & VisionTarget::getHigher2D
{
	return point[m_higherId];
}

const Point2f & VisionTarget::getLower2D
{
	return point[m_lowerId];
}

*/
