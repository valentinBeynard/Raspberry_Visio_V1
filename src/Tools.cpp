#include "Tools.h"


namespace Tools
{
	float TriangleSurface(const Point2f &A, const Point2f &B, const Point2f &C)
	{
		// Calcul de l'aire d'un triangle avec la formule de H�ron
		float p, a, b, c, x, y;

		// cot� a. (= BC)
		x = C.x - B.x;
		y = C.y - B.y;
		a = sqrt(x*x + y * y);

		//cot� b (= CA)
		x = A.x - C.x;
		y = A.y - C.y;
		b = sqrt(x*x + y * y);

		//cot� c (= AB)
		x = B.x - A.x;
		y = B.y - A.y;
		c = sqrt(x*x + y * y);

		p = (a + b + c) / 2.0f; //p est le demi p�rim�tre du triangle
		return sqrt(p * (p - a) * (p - b) * (p - c));
	}

	bool SegXSeg(const cv::Point2f &A, const cv::Point2f &B, const cv::Point2f &C, const cv::Point2f &D, cv::Point2f &pxrx)
	{
		float		d, t, u;
		Point2f		AB, AC, CD;

		// AB Cross CD ?
		// ==========================
		//				D
		//		 	   |
		//			  |		
		//		 	 |		
		//		 A ..+...........B
		//		    |
		//		   |
		//		  |
		//		 C

		AB.x = B.x - A.x;
		AB.y = B.y - A.y;
		CD.x = D.x - C.x;
		CD.y = D.y - C.y;

		d = AB.x*CD.y - CD.x*AB.y;
		if (fabs(d) < FLT_EPSILON_SEGXSEG) // should be if(d == 0.0f)
			return false;

		AC.x = C.x - A.x;
		AC.y = C.y - A.y;
		t = AC.x*CD.y - AC.y*CD.x;
		t /= d;
		if (t<0.0f || t>1.0f)
			return false;

		u = AC.x*AB.y - AC.y*AB.x;
		u /= d;
		if (u<0.0f || u>1.0f)
			return false;

		pxrx.x = A.x + (t*AB.x);
		pxrx.y = A.y + (t*AB.y);

		return true;
	}

Point2f point2fLerp(const cv::Point2f vorigin, const cv::Point2f v, const float t)
{
	Point2f vr;
	vr.x = vorigin.x + (v.x - vorigin.x)*t;
	vr.y = vorigin.y + (v.y - vorigin.y)*t;
	return vr;
}

void point2fLerp(const cv::Point2f vorigin, const cv::Point2f v, const float t, Point2f * ptr)
{
	if(ptr != NULL)
	{
		ptr->x = vorigin.x + (v.x - vorigin.x)*t;
		ptr->y = vorigin.y + (v.y - vorigin.y)*t;
	}
	/*
	vr.x = vorigin.x + (v.x - vorigin.x)*t;
	vr.y = vorigin.y + (v.y - vorigin.y)*t;
	return vr;*/
}

Point3f point3fLerp(const cv::Point3f vorigin, const cv::Point3f v, const float t)
{
	Point3f vr;
	vr.x = vorigin.x + (v.x - vorigin.x)*t;
	vr.y = vorigin.y + (v.y - vorigin.y)*t;
	vr.z = vorigin.z + (v.z - vorigin.z)*t;
	return vr;
}

}