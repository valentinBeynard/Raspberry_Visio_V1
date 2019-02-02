#pragma once
//#include <iostream>

#include "opencv2/opencv.hpp"
using namespace cv;

// Usefull defines
#define FLT_MAX								3.402823466e+38F			// MAX value for a 32 bits 	float
#define NF64_PI								3.14159265358979323846		// PI
#define NF64_EPSILON_VECTOR_DOTPRODUCT		0.000001					// tres petite valeur pour un dot produt
#define FLT_EPSILON_VECTOR_DOTPRODUCT		0.000001f					// tres petite valeur pour un dot produt (float)
#define FLT_EPSILON_NORM					0.0001f						// tres petite valeur pour une norme (loat)
#define FLT_EPSILON_2DCOORD					0.0001f						// tres (trop) petite valeur pour une coordonn�es 2D 
																		//	...utilis�e dans les calculs de "d�projection" pour retrouver un point 3D a partir de sa projection et d'une composante connue.

#define FLT_TOLERANCE_3DCOORD				0.5f						// On parle ici en cm !! et il s'agit de la marge d'erreur acceptable dans l'estimation des distances 3D


#define NDEGtoRAD(deg)		((NF64_PI / 180) * (deg))	// Deg -> Radian
#define NRADtoDEG(rad)		((180 / NF64_PI) * (rad))	// Radian -> Deg
#define NABS(a) (((a) < 0) ? -(a) : (a))				// Absolute value ( valeur sans le signe )
#define IS_ODD(a) ((a)&1)								// Test if a number is a Odd number (2xn + 1 ) if not is Even (2xn) !

// Specific define
#define FLT_EPSILON_SEGXSEG					0.000001f


// BIT Manipulation
#define BITSET(val,bit_id)			((val) |= (1 << (bit_id))) 
#define BITCLEAR(val,bit_id)		((val) &=~(1 << (bit_id)))  
#define BITGET(val,bit_id)			((val) &  (1 << (bit_id)))
#define BITTOGLE(val,bit_id)		((val) ^= (1 << (bit_id)))

namespace Tools
{
	bool	SegXSeg(const Point2f &A, const Point2f &B, const Point2f &C, const Point2f &D, Point2f &pxrx);
	float	TriangleSurface(const Point2f &A, const Point2f &B, const Point2f &C);
	Point2f point2fLerp(const cv::Point2f vorigin, const cv::Point2f v, const float t);
	Point3f point3fLerp(const cv::Point3f vorigin, const cv::Point3f v, const float t);
	void point2fLerp(const cv::Point2f vorigin, const cv::Point2f v, const float t, Point2f * ptr);
}


