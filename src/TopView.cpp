

#include <vector>
#include <memory>
#include <math.h>

#include "TopView.h"
#include "Vision.h"
#include "Tools.h"
#include "Values.h"
#include "VisionTarget.h"
#include "VisionTargetPair.h"
#include "hud.h"
#include "Spline.h"

using namespace cv;

extern Vision vision;
extern vector<VisionTarget> visionTargets;
extern vector<VisionTargetPair> detectedPairs;
extern Spline robotPath;;

double _Error(double trueNumber, double approxNumber)
{
	return approxNumber / trueNumber * 100;
}


double estimate_VisionTargetPairAngle(double angleAOZ, double angleBOZ, double angleCOZ)
{
	double angleAOB;
	double angleBOC;
	double approx;

	double pi = 3.14159265359;
	double x = pi/2;
	int i = 0;
	double k = 0;

	angleAOB = angleAOZ + angleBOZ;
	angleBOC = angleCOZ - angleBOZ;

	/*for (x=0; x<=pi; x+=0.01)
	{
		//y=2*pi-(x+c1+c2)
		approx = Error(sin(angleBOC)/sin(angleAOB), sin(pi-(x+angleAOB+angleBOC))/sin(x));

		if(approx <100.3 && approx > 99.7)
		{
		   // cout << approx<< ":"<<sin(pi-(x+angleAOB+angleBOC))/sin(x)<<"   "<<sin(angleBOC)/sin(angleAOB)<< endl;
			return x;
		}

	}*/
	double test = sin(angleBOC) / sin(angleAOB);
	for (i = 0; i < 15; i++)
	{
		if (sin(pi - (x + angleAOB + angleBOC)) / sin(x) > test)
		{
			x = (x + k) / 2;
		}
		if (sin(pi - (x + angleAOB + angleBOC)) / sin(x) < test)
		{
			k = x;
			x += (x - k) / 2;
		}

	}
	return x;
}


void TopView::render(cv::Mat image)
{

	Point2f topcam;
	Point2f	v0,v1;
	Point3f	v3d;
	Point2f vtxt;
	float	norm;
	char	str[32];
	// FENETRE VUE DE TOP 3D

	// -----------------------------------------------------------------------------------------------------------------------------------------------------
	// -----------------------------------------------------------------------------------------------------------------------------------------------------
	// 1) CONE CAMERA:
	// Position de la camera en vue de Haut
	topcam.x = (float)(image.size().width / 2);
	topcam.y = (float)(image.size().height - image.size().height / 4);

	// Pour dessiner la camera et son cone de vision en vue de Top
	// nous allons d�projeter les coins de l'�cran...
	// Axe camera ( = axe de vis�e = axe Z )
	v0.x = image.size().width/2;
	v0.y = image.size().height / 4;
	vision.estimate3DPoint(v0, REAL_Y_CAMERA+10.0f, v3d);
	v1.x = v3d.x*SCALEFACTOR3D + topcam.x;
	v1.y = topcam.y - v3d.z*SCALEFACTOR3D; 	//	v1.y = v3d.z*SCALEFACTOR3D + topcam.y; Inversion due � l'inversion de l'axe Y en coordonn�es ecran.
	arrowedLine(image, topcam, v1, Scalar(0, 0, 255));

	// Frustum Camera plan de droite ( pour le dessiner nous deprojetons le coin superieur droit de l'�cran)
	// le resultat est le point 3D v3d, pour le tracer nous ne nous interressons qu'� ses coordonn�es X et Z.
	v0.x = (float)vision.m_screenWidth;
	v0.y = 0;
	vision.estimate3DPoint(v0, REAL_Y_CAMERA+10.0f, v3d);
	v1.x = v3d.x*SCALEFACTOR3D + topcam.x;
	v1.y = topcam.y - v3d.z*SCALEFACTOR3D; 	//	v1.y = v3d.z*SCALEFACTOR3D + topcam.y; Inversion due � l'inversion de l'axe Y en coordonn�es ecran.
	line(image, topcam, v1, Scalar(0, 0, 255));

	// Frustum Camera plan de gauche
	// M�me d�marche que pour le plan de droite avec cette fois ci l'angle sup�rieur gauche.
	v0.x = 0;
	v0.y = 0;
	vision.estimate3DPoint(v0, REAL_Y_CAMERA+10.0f, v3d);
	v1.x = v3d.x*SCALEFACTOR3D + topcam.x;
	v1.y = topcam.y - v3d.z*SCALEFACTOR3D; 	//	v1.y = v3d.z*SCALEFACTOR3D + topcam.y; Inversion due � l'inversion de l'axe Y en coordonn�es ecran.
	line(image, topcam, v1, Scalar(0, 0, 255));


	// Calcul et representation de l'angle du mur...
	double angle;
	for (size_t j = 0; j < detectedPairs.size(); j++)
	{
		double angleAOZ = vision.estimateIntrinsicHorizontalAngleRad( detectedPairs[j].m_pvisionTargetA->m_lower2D );
		double angleBOZ = vision.estimateIntrinsicHorizontalAngleRad( detectedPairs[j].m_center[VisionTargetPair::TRAPEZOID_BOTTOM_SIDE] );
		double angleCOZ = vision.estimateIntrinsicHorizontalAngleRad( detectedPairs[j].m_pvisionTargetB->m_lower2D );

		angle = estimate_VisionTargetPairAngle(angleAOZ, angleBOZ, angleCOZ);
		v0 = topcam;
		v1 = v0;
		v1.x += 25 * cos(angle);
		v1.y += 25 * sin(angle);
		line(image, v0, v1, Scalar(0, 0, 255));
	}

	// -----------------------------------------------------------------------------------------------------------------------------------------------------
	// -----------------------------------------------------------------------------------------------------------------------------------------------------
	//
	// 2) VISIONTARGET POINTS 
	vtxt.x = 10;
	vtxt.y = 10;
	for (size_t i = 0; i < visionTargets.size(); i++)
	{
		// Points Bas
		v0.x = visionTargets[i].m_lower3D.x*SCALEFACTOR3D + topcam.x;
		v0.y = topcam.y - visionTargets[i].m_lower3D.z*SCALEFACTOR3D;
		hud::drawPoint(image, v0, 5, Scalar(0, 255, 255) );
		
		// Points Haut
		v0.x = visionTargets[i].m_higher3D.x*SCALEFACTOR3D + topcam.x;
		v0.y = topcam.y - visionTargets[i].m_higher3D.z*SCALEFACTOR3D;
		hud::drawPoint(image, v0, 5, Scalar(255, 255, 0));

/*
		v1.x = v0.x;
		v1.y = v0.y + 10;
		line(image, v0, v1, Scalar(255, 255, 255));
		// X horizontal
		v0.x -= 5;
		v0.y += 5;
		v1.x = v0.x+10;
		v1.y = v0.y;
		line(image, v0, v1, Scalar(255, 255, 255));
*/

		// TRACAGE DES LIGNES reliant tous les points bas et tous les points hauts des Vision Target
		if (i > 0)
		{
			// lower points
			v0.x = visionTargets[i].m_lower3D.x*SCALEFACTOR3D + topcam.x;
			v0.y = topcam.y - visionTargets[i].m_lower3D.z*SCALEFACTOR3D;

			v1.x = visionTargets[i - 1].m_lower3D.x*SCALEFACTOR3D + topcam.x;
			v1.y = topcam.y - visionTargets[i - 1].m_lower3D.z*SCALEFACTOR3D;
			line(image, v0, v1, Scalar(0, 128, 255));
			
			v3d = visionTargets[i].m_lower3D - visionTargets[i-1].m_lower3D;
			norm = sqrtf(v3d.x + v3d.x + v3d.z*v3d.z); // norme "vue de haut"
			sprintf(str, "%.3f", norm);
			cv::putText(image, str, vtxt, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));
			vtxt.y += 15;


			// higher points
			v0.x = visionTargets[i].m_higher3D.x*SCALEFACTOR3D + topcam.x;
			v0.y = topcam.y - visionTargets[i].m_higher3D.z*SCALEFACTOR3D;

			v1.x = visionTargets[i - 1].m_higher3D.x*SCALEFACTOR3D + topcam.x;
			v1.y = topcam.y - visionTargets[i - 1].m_higher3D.z*SCALEFACTOR3D;
			line(image, v0, v1, Scalar(0, 255, 128));
		}
	}

	if ( detectedPairs.size() )
	{	
		size_t i = detectedPairs.size() / 2;
		Point3f direction = detectedPairs[i].m_pvisionTargetA->m_lower3D - detectedPairs[i].m_pvisionTargetB->m_lower3D;
		robotPath.m_splineKnots[0].m_position.x = topcam.x;
		robotPath.m_splineKnots[0].m_position.y = topcam.y;
		robotPath.m_splineKnots[0].m_tb.x = topcam.x;
		robotPath.m_splineKnots[0].m_tb.y = topcam.y - 75;

		
		robotPath.m_splineKnots[1].m_position.x = detectedPairs[i].m_pvisionTargetA->m_lower3D.x*SCALEFACTOR3D + topcam.x;
		robotPath.m_splineKnots[1].m_position.y = topcam.y - detectedPairs[i].m_pvisionTargetA->m_lower3D.z*SCALEFACTOR3D;
		robotPath.m_splineKnots[1].m_ta.x = robotPath.m_splineKnots[1].m_position.x - direction.z * 25 ;
		robotPath.m_splineKnots[1].m_ta.y = robotPath.m_splineKnots[1].m_position.y - direction.x * 25;
		
		robotPath.drawSpline(image, Scalar(0, 128, 255));

		//tangente:
		line(image, robotPath.m_splineKnots[0].m_position, robotPath.m_splineKnots[0].m_tb, Scalar(0, 255, 128));
		line(image, robotPath.m_splineKnots[1].m_position, robotPath.m_splineKnots[1].m_ta, Scalar(0, 255, 128));

	}
	/*
	//vision.get3DPointOnHorizontalPlane(v3a, v0, REAL_PLANEY_ID_VISIONTARGET_TOP);
	v1.x = v3a.x*SCALEFACTOR3D + cam.x;
	//	v1.y = v3a.z*scalefactor + cam.y;
	v1.y = cam.y - v3a.z*SCALEFACTOR3D;
	line(Render3DImage, cam, v1, Scalar(0, 0, 255));

	v0.x = (float)vision.m_screenWidth;
	v0.y = 0;
	//vision.get3DPointOnHorizontalPlane(v3a, v0, REAL_PLANEY_ID_VISIONTARGET_TOP);
	v1.x = v3a.x*SCALEFACTOR3D + cam.x;
	//	v1.y = v3a.z*scalefactor + cam.y;
	v1.y = cam.y - v3a.z*SCALEFACTOR3D;
	line(Render3DImage, cam, v1, Scalar(0, 0, 255));

	v0.x = 0;
	v0.y = 0;
	//.get3DPointOnHorizontalPlane(v3a, v0, REAL_PLANEY_ID_VISIONTARGET_TOP);
	v1.x = v3a.x*SCALEFACTOR3D + cam.x;
	//	v1.y = v3a.z*scalefactor + cam.y;
	v1.y = cam.y - v3a.z*SCALEFACTOR3D;
	line(Render3DImage, cam, v1, Scalar(0, 0, 255));

	for (size_t i = 0; i < visionTargets.size(); i++)
	{
		v0.x = visionTargets[i].middleBottom3D.x*SCALEFACTOR3D + cam.x;
		v0.y = cam.y - visionTargets[i].middleBottom3D.z*SCALEFACTOR3D - 5;
		v1.x = v0.x;
		v1.y = v0.y + 5;
		line(Render3DImage, v0, v1, Scalar(0, 255, 255));

		v0.x = visionTargets[i].middleTop3D.x*SCALEFACTOR3D + cam.x;
		v0.y = cam.y - visionTargets[i].middleTop3D.z*SCALEFACTOR3D - 5;
		v1.x = v0.x;
		v1.y = v0.y + 5;
		line(Render3DImage, v0, v1, Scalar(255, 255, 0));


		// VUE DE TOP TRACAGE DES LIGNES Visio Target Top et bottom et center
		if (i > 0)
		{
			// Vue de TOP: tracage lignes passant par le BAS des vision target
			v0.x = visionTargets[i].middleBottom3D.x*SCALEFACTOR3D + cam.x;
			v0.y = cam.y - visionTargets[i].middleBottom3D.z*SCALEFACTOR3D;
			v1.x = visionTargets[i - 1].middleBottom3D.x*SCALEFACTOR3D + cam.x;
			v1.y = cam.y - visionTargets[i - 1].middleBottom3D.z*SCALEFACTOR3D;
			line(Render3DImage, v0, v1, Scalar(0, 255, 255));

			// angle du segement avec (v1v0) l'axe z de la camera, coincidant ici, en vue de top 2D avec l'axe y (0,1)
			v0.x -= v1.x;
			v0.y -= v1.y;
			n0 = sqrt(v0.x*v0.x + v0.y*v0.y);
			//v0.x /= n0;
			asB += v0.y / n0;


			// Vue de TOP: tracage lignes passant par le HAUT des vision target
			v0.x = visionTargets[i].middleTop3D.x*SCALEFACTOR3D + cam.x;
			v0.y = cam.y - visionTargets[i].middleTop3D.z*SCALEFACTOR3D;
			v1.x = visionTargets[i - 1].middleTop3D.x*SCALEFACTOR3D + cam.x;
			v1.y = cam.y - visionTargets[i - 1].middleTop3D.z*SCALEFACTOR3D;
			line(Render3DImage, v0, v1, Scalar(0, 255, 0));

			// angle du segement avec (v1v0) l'axe z de la camera, coincidant ici, en vue de top 2D avec l'axe y (0,1)
			v0.x -= v1.x;
			v0.y -= v1.y;
			n0 = sqrt(v0.x*v0.x + v0.y*v0.y);
			//v0.x /= n0;
			asT += v0.y / n0;

			// Vue de TOP: tracage lignes passant par le CENTRE des vision target
			v0.x = visionTargets[i].center3D.x*SCALEFACTOR3D + cam.x;
			v0.y = cam.y - visionTargets[i].center3D.z*SCALEFACTOR3D;
			v1.x = visionTargets[i - 1].center3D.x*SCALEFACTOR3D + cam.x;
			v1.y = cam.y - visionTargets[i - 1].center3D.z*SCALEFACTOR3D;
			line(Render3DImage, v0, v1, Scalar(0, 0, 255));

			// angle du segement avec (v1v0) l'axe z de la camera, coincidant ici, en vue de top 2D avec l'axe y (0,1)
			v0.x -= v1.x;
			v0.y -= v1.y;
			n0 = sqrt(v0.x*v0.x + v0.y*v0.y);
			//v0.x /= n0;
			asC += v0.y / n0;

		}

	}
	*/
}