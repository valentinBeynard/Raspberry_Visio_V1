#pragma once
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

#define COORD3DMAX							-1000000					// Just a big number

class Vision
{
public:
	Vision(const double camerafov_deg, const double cameraYworld, const size_t screenwidth, const size_t screenheight);
	~Vision();

	// METHODES
	void setFov(const double camerafov_deg);
	void setCameraYWorld(const float y);
	bool estimate3DPoint(const cv::Point2f& point, const float realYFromTheGround, Point3f &resultat3D);
	bool estimate3DVerticalBipoint(const cv::Point2f(&point)[2], const float (&realYFromTheGround)[2], Point3f(&resultat3D)[2]);
	
	float estimateIntrinsicHorizontalAngleRad(const cv::Point2f &p);
	float estimateIntrinsicVerticalAngleRad(const cv::Point2f &p);
	float estimateIntrinsicDistanceZ(const cv::Point2f &p, const float realHeightFromTheGround);

	// NEW METHODES
	bool loadCameraIntrinsic(char *filename);
	bool saveCameraIntrinsic(char *filename);
	bool loadVisionTargetModel(char *filename);

	float	getIntrinsicCameraFocalDistX();
	float	getIntrinsicCameraFocalDistY();
	float	getIntrinsicScreenCenterX();
	float	getIntrinsicScreenCenterY();


	float		m_screenWidth;
	float		m_screenHeight;
	float		m_cameraYworld;
	
	double		m_fovY;						// Camera FOVY (DEG). Specifies the field of view angle, in degrees, in the y direction.
	double		m_fovDist;
	double		m_aspectRatio;				// Specifies the aspect ratio that determines the field of view in the x direction. The aspect ratio is the ratio of x (width) to y (height).	
	double		m_tangent;					// Pre-computed Data used for precise FRUSTRUM Culling and other things ...
	double		m_inverseTangent;				// Pre-computed Data 
	double		m_widthoutof2;
	double		m_heightoutof2;

	//

	// *********************************************************************************************************************
	// NEW ONE
	Mat			m_intrinsic;
	Mat			m_distCoeffs;
	vector<Mat> m_rvecs;
	vector<Mat> m_tvecs;


	
	vector<Point3f> m_visionTarget3DModel;



// CAMERA DEFINITION
//	cv::Point3f	m_Xaxis;
//	cv::Point3f	m_Yaxis;
//	cv::Point3f	m_Zaxis;

};
