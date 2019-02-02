#include "Spline.h"
#include "Tools.h"

SplineKnot::SplineKnot()
{
	m_position	= Point2f(0, 0);
	m_ta		= Point2f(0, 0);
	m_tb		= Point2f(0, 0);
}
SplineKnot::SplineKnot(const cv::Point2f & pos, const cv::Point2f & ta, const cv::Point2f & tb)
{
	m_position = pos;
	m_ta = ta;
	m_tb = tb;
}
SplineKnot::~SplineKnot()
{}

Spline::Spline(const int capacity, const int accuracy)
{
	m_accuracy = accuracy;
	m_splineKnots.reserve(capacity);
}

Spline::~Spline()
{
	m_splineKnots.clear();
}

void Spline::insertKnot()
{
	SplineKnot knot;
	m_splineKnots.push_back(knot);
}

void Spline::insertKnot(const size_t nb)
{
	m_splineKnots.resize(m_splineKnots.size()+nb);
}

void Spline::insertKnot(const cv::Point2f & pos, const cv::Point2f & ta, const cv::Point2f & tb)
{
	SplineKnot knot = SplineKnot(pos,ta,tb);
	m_splineKnots.push_back(knot);
}

cv::Point2f Spline::qLerp(const size_t kaid, const size_t kbid, const float t)
{
	Point2f R, L1, L2, L3, L4, L5;

	//Tools::point2fLerp( m_splineKnots[kaid].m_position, m_splineKnots[kaid].m_tb, t );
	//Tools::point2fLerp(m_splineKnots[kaid].m_position, m_splineKnots[kaid].m_tb, t, &L1);
	L1 = Tools::point2fLerp( m_splineKnots[kaid].m_position, m_splineKnots[kaid].m_tb, t );
	L2 = Tools::point2fLerp(m_splineKnots[kaid].m_tb, m_splineKnots[kbid].m_ta, t);
	L3 = Tools::point2fLerp(m_splineKnots[kbid].m_ta, m_splineKnots[kbid].m_position, t);
	L4 = Tools::point2fLerp(L1,L2, t);
	L5 = Tools::point2fLerp(L2, L3, t);
	R  = Tools::point2fLerp(L4, L5, t);
	return R;
}

void Spline::drawSpline(cv::Mat image, const cv::Scalar & color)
{
	Point2f a, b;
	float fr, fac;
	fr = fac = 1.0f / (float)m_accuracy;

	for (size_t i = 1; i < m_splineKnots.size(); i++)
	{
		a = m_splineKnots[i - 1].m_position;
		for (size_t j = 1; j <= m_accuracy; j++, fr += fac)
		{
			b = qLerp(i - 1, i, fr);
			cv::line(image, a, b, color );
			a = b;
		}
	}
}


