/*
#include "opencv2/opencv.hpp"

using namespace cv;


int main(int, char**)
{
	VideoCapture cap(0); // open the default camera
	if (!cap.isOpened())  // check if we succeeded
		return -1;

	Mat edges;
	namedWindow("cap", 1);
	for (;;)
	{
		Mat frame;
		//cap >> frame; // get a new frame from camera
		cap.read(frame);
		//cvtColor(frame, edges, COLOR_BGR2GRAY);
		//GaussianBlur(edges, edges, Size(7, 7), 1.5, 1.5);
		//Canny(edges, edges, 0, 30, 3);
		imshow("cap", frame);
		if (waitKey(30) >= 0) break;
	}
	// the camera will be deinitialized automatically in VideoCapture destructor
	return 0;
}
*/
// Definir le code de filtrage qui sera compilé:

//#define PRECOMPIL_IMAGEFILTER_HSV

#define RASPICAM_API
#define STREAM_5553
#define STREAM_PORT	8081
#define STREAM_SINK_NAME	"httppi5553"


#include "PrecompilationDefine.h"

#include <unistd.h>
#include <iostream>
#include <chrono>
#include <vector>
#include <memory>
//#include <cmath>
#include <math.h>
#include "opencv2/opencv.hpp"

#include "Vision.h"
#include "Tools.h"
#include "Values.h"

#include "VisionTarget.h"
#include "VisionTargetPair.h"
#include "hud.h"
#include "Spline.h"
#include "TopView.h"

/* API for Raspberry PI Camera on Camera Bus */
#include "raspicam_cv.h"

/* Streaming Serveur */
#include <cscore.h>

//#include <networktables/NetworkTable.h>
//#include <networktables/NetworkTableInstance.h>


using namespace cv;
using namespace std;


unsigned long RoboLyonFlags = 0;


/* Camera Specs and Configurations */
/* Note:  Higher resolution & framerate is possible, depending upon processing cpu usage */
cv::VideoCapture camera;

/* RaspiCAM for Raspberry Camera on BUS */
raspicam::RaspiCam_Cv* camera_pi;

/* Stream */
cs::CvSource* stream_src;
cs::MjpegServer* stream_server;

/*
const int frames_per_sec = 5;
const double width = CAPTURED_IMAGE_WIDTH;
const double height = CAPTURED_IMAGE_HEIGHT;
const double fov = REAL_CAMERA_FOV;
const double fov_rad = fov * (3.14 / 180);
const double distance_focale = width / (2 * tan(fov_rad / 2));
*/

// Declaration de Vision en globale
Vision  vision(REAL_CAMERA_FOV, REAL_Y_CAMERA, CAPTURED_IMAGE_WIDTH, CAPTURED_IMAGE_HEIGHT);


/* Matrices used for processing */
// IMAGES
cv::Mat SrcImage;
cv::Mat RawImage;		// Image capturée par la caméra.	
cv::Mat FilteredImage;	// Image filtrée utilisée pour la détection de contour. 
cv::Mat OnScreenImage;	// Image affichée à l'écran.
cv::Mat HlsImage;		// Image HSL (Hue,Saturation,Lightness) utilisée par certaines batterie de filtre ...
cv::Mat Render3DImage(CAPTURED_IMAGE_HEIGHT, CAPTURED_IMAGE_WIDTH, CV_8UC3);	// Image 3D calculée à partir des données capturées






/* Vector of Vectors of Points = contours */
std::vector<std::vector<cv::Point> > contours;
std::vector<std::vector<cv::Point> > hull;
std::vector<std::vector<cv::Point> > approxContours;
std::vector<cv::Vec4f > fitLines;
std::vector<cv::Moments> moms;

//std::vector<std::vector<cv::Point> > filterContours;

//std::vector<RotatedRectXtd> rotatedRectangles;
std::vector<VisionTarget> visionTargets;
std::vector<VisionTargetPair> detectedPairs;


std::vector<cv::Point2f> centroids;

Spline robotPath;

cv::Point2f real_centroid;
cv::Point2f target;
float orientation;


// Editable Values
int camera_bright = 75;


int areamin_ratio	= ADVPARAM_VISIONTARGET_SIZEMIN;	// means a% of each screen dimension >>> (a% of screen width) * (a% of screen height)
int areamax_ratio	= ADVPARAM_VISIONTARGET_SIZEMAX;	// means a% of each screen dimension >>> (a% of screen width) * (a% of screen height)
int horizon			= ADVPARAM_HORIZON;					// means a% of screen height

#ifdef PRECOMPIL_IMAGEFILTER_HSV
int Hlow = FILTRE_HLS_HLOW;
int Llow = FILTRE_HLS_LLOW;
int Slow = FILTRE_HLS_SLOW;
int Hhigh = FILTRE_HLS_HHIGH;
int Lhigh = FILTRE_HLS_LHIGH;
int Shigh = FILTRE_HLS_SHIGH;
#endif

#ifdef PRECOMPIL_IMAGEFILTER_CANNY_AND_THRESHOLD
int Alpha = 10;
int Beta  = 255;
int cannythresh1 = FILTRE_CANNY_THRESH1;
int cannythresh2 = FILTRE_CANNY_THRESH2;

int threshold_min = FILTRE_THRESHOLD_MIN;
int threshold_max = FILTRE_THRESHOLD_MAX;
#endif




float centroid_size = 1.0f;
/* Network Tables */
//nt::NetworkTableInstance inst;
//std::shared_ptr<nt::NetworkTable> table;
//nt::NetworkTableEntry entry;


bool Init();
bool CaptureFrame();
void ProcessFrame();
void ShowFrames();
void StreamFrames();
bool InitStream();

const double t = 0.2f;

void StreamFrames()
{
	stream_src->PutFrame(SrcImage);
}

bool InitStream()
{
	stream_server = new cs::MjpegServer(STREAM_SINK_NAME, STREAM_PORT);
	stream_src = new cs::CvSource("cvsource5553", cs::VideoMode::kBGR,CAPTURED_IMAGE_WIDTH,CAPTURED_IMAGE_HEIGHT,camera_pi->get(CV_CAP_PROP_FPS));
	stream_server->SetSource(*stream_src);
	return true;
}

double polygonArea(vector<Point> &polygon)
{
	double area = 0.0;
	size_t j = polygon.size() - 1;
	for (size_t i = 0; i < polygon.size(); i++)
	{
		area += (polygon[j].x + polygon[i].x) * (polygon[j].y - polygon[i].y);
		j = i;
	}
	return abs(area / 2.0);

}

int main()
{
	time_t timestamp_debut = std::time(0);

	if (!Init())
	{
		cerr << "Error Init Camera !" << std::endl;
		return 0;
	}
	#ifdef STREAM_5553
	if (!InitStream())
	{
		cerr << "Error Init Stream" << std::endl;
		return 0;
	}
	#endif
	//CaptureFrame();
	//cv::imwrite("output_img.jpg", SrcImage);
	/*
	Size s = Size(320,240);
	VideoWriter outputVideo("outputvid.avi",CV_FOURCC('M','J','P','G') , 50, s, true);

	if(!outputVideo.isOpened())
	{
		cerr << "Error video writter" << std::endl;
		return 0;	
	}

	std::cout << "Start Record !" << std::endl; 
	*/
	//for (int i = 0; i < 100; i++)
	for(int i = 0; i<1000; i++)
	{
		if (CaptureFrame())
		{
//			outputVideo.write(SrcImage);
			//ProcessFrame();
			#ifdef STREAM_5553
				StreamFrames();
				//ShowFrames();
			#endif
		}
	}
	//cv::imwrite("output_img.jpg", SrcImage);
	time_t timestamp_fin = std::time(0);
	std::cout << "Programme terminé au bout de " << timestamp_fin - timestamp_debut << " secondes" << std::endl;
	std::cout << "FPS = " << 1000.0 /  (timestamp_fin - timestamp_debut) << std::endl;

	camera_pi->release();

	return 0;
}


void on_trackbar_fov(int value, void *ptr)
{
	vision.setFov((double)value / 10);
}

void on_trackbar_camheight(int value, void *ptr)
{
	vision.setCameraYWorld((double)value / 100);
}

#ifdef RASPICAM_API
bool Init()
{
	
	// Load Intrinsic parameter of the Camera
	if (!vision.loadCameraIntrinsic(INTRINSIC_FILENAME))
		return false;
	// Load The Visual target Pair 3D descrition.( orign(0,0,0) in between the 2 vision targets and on the ground level.) 
	if (!vision.loadVisionTargetModel(VISIONTARGET_FILENAME))
		return false;

	camera_pi = new raspicam::RaspiCam_Cv();
	camera_pi->set(CV_CAP_PROP_FORMAT, CV_8UC3);
	camera_pi->set(CV_CAP_PROP_MODE, CAMERA_MODE);
	if( !camera_pi->open() )
	{
		cerr << "Error loading raspicam !" << std::endl;
		return false;
	}
	camera_pi->set(CV_CAP_PROP_BRIGHTNESS, camera_bright);

	camera_pi->set(CV_CAP_PROP_FRAME_WIDTH, CAPTURED_IMAGE_WIDTH);
	camera_pi->set(CV_CAP_PROP_FRAME_HEIGHT, CAPTURED_IMAGE_HEIGHT);
	camera_pi->set(CV_CAP_PROP_FPS, 30);
	camera_pi->set(CV_CAP_PROP_EXPOSURE, 70);
	//camera_pi->set(CV_CAP_PROP_FORMAT, CV_8UC3);

	return true;
/*
	camera_pi = new raspicam::RaspiCam_Cv();
	if( !camera_pi->open() )
	{
		cerr << "Error loading raspicam !" << std::endl;
		return false;
	}
	camera_pi->set(CV_CAP_PROP_BRIGHTNESS, camera_bright);

	camera_pi->set(CV_CAP_PROP_FRAME_WIDTH, CAPTURED_IMAGE_WIDTH);
camera_pi->set(CV_CAP_PROP_FRAME_HEIGHT, CAPTURED_IMAGE_HEIGHT);
	//camera_pi->set(cv::CAP_PROP_FPS, 50);
*/
}
#else



bool Init()
{
	// Load Intrinsic parameter of the Camera
	if (!vision.loadCameraIntrinsic(INTRINSIC_FILENAME))
		return false;
	// Load The Visual target Pair 3D descrition.( orign(0,0,0) in between the 2 vision targets and on the ground level.) 
	if (!vision.loadVisionTargetModel(VISIONTARGET_FILENAME))
		return false;

	/* Initialize the Robot Path */ // ( une spline pour le moment mais peut être plutot des cercles et des tangentes ... à voir )
	robotPath.insertKnot(2);

	

	/* Open camera and set properties */
	if(!camera.open(0))
	{ std::cout << "Loading Error" << std::endl;}
	
	//camera.open("D:/_PROJETS/FIRST/Image/videorobot.mp4");
	camera.set(cv::CAP_PROP_BRIGHTNESS, camera_bright);

	camera.set(cv::CAP_PROP_FRAME_WIDTH, CAPTURED_IMAGE_WIDTH);
	camera.set(cv::CAP_PROP_FRAME_HEIGHT, CAPTURED_IMAGE_HEIGHT);
	camera.set(cv::CAP_PROP_FPS, 50);
	camera.set(cv::CAP_PROP_BRIGHTNESS, camera_bright);
	camera.set(cv::CAP_PROP_MODE,1);
	/* Create tables */
	/*inst = nt::NetworkTableInstance::GetDefault();
	table = inst.GetTable("datatable");
	entry = table->GetEntry("Angle");
	inst.StartClientTeam(5553);*/

	target.x = 400;
	target.y = 300;

// BASIC FILTER PARAM (toujours actif et compilé, quelquesoit le filtrage actif)
	/*namedWindow("CAMERA", WINDOW_AUTOSIZE);
	resizeWindow("CAMERA", 1000, 200);
	createTrackbar("CamLight", "CAMERA", &camera_bright, 100, NULL);
	createTrackbar("CamFOV", "CAMERA",nullptr, 900, on_trackbar_fov);
	createTrackbar("CamHEIGHT", "CAMERA", nullptr, 2000, on_trackbar_camheight); // 0 to 1000 pour 0 to 10.00 cm

#ifdef PRECOMPIL_IMAGEFILTER_HSV
	namedWindow("FILTERING-A", WINDOW_AUTOSIZE);
	resizeWindow("FILTERING-A", 500, 300);
	createTrackbar("H Low", "FILTERING-A", &Hlow, 255, NULL);
	createTrackbar("L Low", "FILTERING-A", &Llow, 255, NULL);
	createTrackbar("S Low", "FILTERING-A", &Slow, 255, NULL);
	
	createTrackbar("H High", "FILTERING-A", &Hhigh, 255, NULL);
	createTrackbar("L High", "FILTERING-A", &Lhigh, 255, NULL);
	createTrackbar("S High", "FILTERING-A", &Shigh, 255, NULL);
#endif

#ifdef PRECOMPIL_IMAGEFILTER_CANNY_AND_THRESHOLD
	namedWindow("FILTERING-B", WINDOW_AUTOSIZE);
	resizeWindow("FILTERING-B", 1000, 200);
	createTrackbar("Conv-A", "FILTERING-B", &Alpha,100, NULL);
	createTrackbar("Conv-B", "FILTERING-B", &Beta, 512, NULL);
	createTrackbar("Canny1", "FILTERING-B", &cannythresh1, 1000, NULL);
	createTrackbar("Canny2", "FILTERING-B", &cannythresh2, 1000, NULL);
	createTrackbar("ThresMin", "FILTERING-B", &threshold_min, 255, NULL);
	createTrackbar("ThresMax", "FILTERING-B", &threshold_max, 255, NULL);
#endif


// DETECTION PARAM
	namedWindow("CONTOUR-DETECT", WINDOW_AUTOSIZE);
	resizeWindow("CONTOUR-DETECT", 1000, 200);
	createTrackbar("Horizon", "CONTOUR-DETECT", &horizon, 100, NULL);
	createTrackbar("AreaMin", "CONTOUR-DETECT", &areamin_ratio, 1000, NULL);
	createTrackbar("AreaMax", "CONTOUR-DETECT", &areamax_ratio, 1000, NULL);*/

	return true;
}
#endif

#ifdef RASPICAM_API

bool CaptureFrame()
{
	//camera_pi->set(CV_CAP_PROP_BRIGHTNESS, camera_bright);
/*
	if (BITGET(RoboLyonFlags, FLAG_CAPTURE_PAUSE))
	{
		RawImage = SrcImage.clone();
		return true;
	}*/
	time_t t_0 = std::time(0);
	if (!camera_pi->grab() )
	{
		cerr << "Error Grabbing Video Frame " << std::endl;
		return false;
	}else
	{
		camera_pi->retrieve(SrcImage);
		//cv::cvtColor(YUV_SrcImage, SrcImage, cv::COLOR_YCrCb2BGR);
		undistort(SrcImage, RawImage, vision.m_intrinsic, vision.m_distCoeffs);
		std::cout << "Time Elapsed : " << std::time(0) - 1.0 * t_0 << std::endl;
		return true;
	}
	
 }

#else
bool CaptureFrame()
{
	camera.set(cv::CAP_PROP_BRIGHTNESS, camera_bright);
/*
	// Si on demande pause, on retourne true
	if (BITGET(RoboLyonFlags, FLAG_CAPTURE_PAUSE))
	{
		// Undistord RawImage
		RawImage = SrcImage.clone();
		return true;
	}
	//camera >> SrcImage;
*/	
	
	
	if (!camera.read(SrcImage))
	{
		std::cout << "Error Grabbing Video Frame" << std::endl;
		return false;
	}
	else
	{
		// Undistord RawImage
		//RawImage = SrcImage;
		undistort(SrcImage, RawImage, vision.m_intrinsic, vision.m_distCoeffs);
		return true;
	}
}
#endif

float GetOrientation(Point2f v0, Point2f v1)
{
	return v0.x*v1.y - v0.y*v1.x;
}

void GetDirection(RotatedRect rect, Point2f *r)
{
	Point2f vertices[4];
	rect.points(vertices);

	r->x = vertices[1].x - vertices[0].x;
	r->y = vertices[1].y - vertices[0].y;
}


bool compare_x(const VisionTarget &shapeA, const VisionTarget &shapeB)
{
	return  (shapeA.m_center.x < shapeB.m_center.x);
}

void ProcessFrame()
{

// ---------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------
// ETAPE 1/3 : (PRE-)FILTERING
//			En entrée, l'image "Brut" ( sans traitement) capturée par la camera. 
//			En sortie, l'image "filtrée" prête à être analysée.
// ---------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------

	// First Method
#ifdef PRECOMPIL_IMAGEFILTER_HSV
	//########## BGR TO HLS ##########
	cv::cvtColor(RawImage, HlsImage, cv::COLOR_BGR2HLS);
	//########## Threshold ##########
	cv::inRange(HlsImage, Scalar(Hlow, Llow, Slow), Scalar(Hhigh, Lhigh, Shigh), FilteredImage);
	//cv::inRange(HlsImage, cv::Scalar(50, 130, 150), cv::Scalar(95, 255, 255), FilteredImage);
	//########## Erode and Dilate ##########
	// test effectué pas très concluant. A appronfondir plus tard... pour l'instant on met de côté.
	//cv::Mat kernel = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(9, 9), cv::Point(-1, -1));
	//cv::morphologyEx(FilteredImage, TestImage, cv::MORPH_OPEN, kernel);
	//cv::imshow("TEST MORPHOLOGY", TestImage);

	// or (instead of Erode and Dilate )
	//FilteredImage.convertTo(FilteredImage, CV_8UC3);
	//cv::cvtColor(FilteredImage, FilteredImage, cv::COLOR_HSV2BGR);

#endif	




	// Second Method
#ifdef	PRECOMPIL_IMAGEFILTER_CANNY_AND_THRESHOLD
	// Pre-filtrage de type luminosité contraste
	if (Alpha && Beta)
		RawImage.convertTo(RawImage, -1, (double)Alpha / 10, (double)(Beta - 255));

	// Filtre 1: On passe l'image en niveau de gris.
	cvtColor(RawImage, FilteredImage, COLOR_BGR2GRAY);

	// Filtre 2: L'image est floutée ( un peu )
	//GaussianBlur(FilteredImage, FilteredImage, Size(7, 7), 0, 0);
	// Filtre 3: L'image est "écrêtée" et passe en deux couleurs : Noir ou Blanc
	// ( le filtre n'est appliqué que si ses deux paramètres éditable sont differents de 0 )
	if(threshold_min && threshold_max)
		inRange(FilteredImage, threshold_min, threshold_max, FilteredImage);
	
	// Filtre 4: Filtre spécial Canny qui dessinne en blanc sur fond noir les pourtours de l'image.
	// ( le filtre n'est appliqué que si ses deux paramètres éditables sont differents de 0 )
	if (cannythresh1 && cannythresh2)
	{
		blur(FilteredImage, FilteredImage, Size(3, 3));
		Canny(FilteredImage, FilteredImage, cannythresh1, cannythresh2, 3);
	}
#endif
	
	// Third Method
	//RawImage.convertTo(FilteredImage, -1, (float)Alpha/1000.0f, (float)Beta*-1.0f);
	//cvtColor(FilteredImage, FilteredImage, COLOR_BGR2GRAY);

// ---------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------
// ETAPE 2/3 : ANALYSE DE L'IMAGE FILTREE ET DETECTION DE CONTOUR
//			En entrée, l'image "filtrée"
//			En sortie, une liste de "contours" détectés par OpenCV
// ---------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------
	contours.resize(0);
	cv::findContours(FilteredImage, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	cv::cvtColor(FilteredImage, FilteredImage, cv::COLOR_GRAY2BGR);

	/*
	hull.clear();
	hull.resize(contours.size());
	
	approxContours.clear();
	approxContours.resize(contours.size());

	fitLines.clear();
	fitLines.resize(contours.size());

	moms.clear();
	moms.resize(contours.size());

	double epsilon;
	for (size_t i = 0; i < contours.size(); i++)
	{
		// Hull 
		cv::convexHull(contours[i], hull[i], false, true);

		// approx poly
		epsilon = cv::arcLength(contours[i], true)*0.01;
		do
		{
			cv::approxPolyDP(contours[i], approxContours[i], epsilon, true);
			epsilon *= 2;

		} while (approxContours[i].size() > 4);

		//Fit Line
		cv::fitLine(contours[i], fitLines[i], DIST_L12, 0, 0.01, 0.01);

		// Moment
		moms[i] = moments(contours[i], false);
	}
	*/

// ---------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------
// ETAPE 3	ANALYSE DE LA LISTE DES CONTOURS ET IDENTIFICATION DES "VISION TARGET"
//			En sortie d'étape 2 on se retrouve avec une liste de contours "bruts".
//			En fait, avec tous les contours que OpenCV à pu trouver...
//			Potentiellement cette liste contient des contours "parasites" comme des surfaces blanches, spots, flash, etc ...
//			qui ne sont pas les Vision target refléchissantes que nous voulons repérer ...
//			Le but de cette 3ème étape est d'essayer de repérer les contours qui s'apparentent le plus à des Visions target.
//			
//			Nous allons donc appliquer une série de test sur chaque contour de la liste et décider si ce dernier peut "être ou ne pas être" ... une Vision Target !
//			Il est important de noter que pour cette analyse chaque contour est testé indépendement des autres.
//
//				-test 0: test de hauteur apparente.		>>> Si le contour est trop bas dans l'image, c'est à dire en dessous d'une limite arbitraire qu'on appelle horizon  ... on passe.
//				-test 1: test de Taille (surface)		>>> Si le contour est trop petit, ou trop grand... on passe. ( valeur min et max fixées )
//				-test 2: test de forme et d'orientation	>>> Si le contour a des proportions qui ne "collent pas" avec les proportions que pourrait avoir un scotch reflechissant ( 5cm X 14.5cm ) 
//															même déformé par la perspective ... on passe.
//														>>> Si le contour n'est pas un rectangle "vertical" ( avec son plus grand côté plutôt dans la hauteur ) ... on passe

	visionTargets.resize(0);
	
	Rect			rect;	 
	RotatedRect		rotrect;
	VisionTarget	shape;
	Point2f			vertices[4];
	Point2f			v01, v03;
	Point2f			u, v;
	float			unorm, vnorm;
	float area;
	float proportion;
	float test;

	// ---------------------------------------------------------------------------------------------
	// Calcul surface min et max en fonction du "pour mille"  (editable)
	// Calcul Horizon en fonction du "pour cent"  (editable)
	// TODO: Devraientt etre calculés une bonnefois pour toute et stockés !
	float r = (float)(areamin_ratio*areamin_ratio) / 1000000.0f; // surface ! 
	float areamin = RawImage.size().width*RawImage.size().height*r;
	r = (float)(areamax_ratio*areamax_ratio) / 1000000.0f; // surface ! 
	float areamax = RawImage.size().width*RawImage.size().height*r;
	float horizon_limit = RawImage.size().height*(float)horizon / 100.0f;

	// ---------------------------------------------------------------------------------------------	
	// "Boucle décisionnelle" : 
	//		Un contour est-il une vision target ?
	//		Si oui, on crée et stocke une VisionTarget et on la "push" dans le tableau des Vision Targets					
	float norm01, norm03;
	for (size_t i = 0; i < contours.size(); i++)
	{
		rotrect = minAreaRect(contours[i]);
		rotrect.points(vertices);

		// test 0: Under Horizon limit ?
		if (vertices[0].y > horizon_limit)
			continue;

		// test 1: Too small ?
		area = rotrect.size.height * rotrect.size.width;
		if ((area < (float)areamin) || (area > (float)areamax) )
			continue;

		//shape.boundingRect = boundingRect(contours[i]);
		// Bad proportion ?
		//proportion = MAX(rotrect.size.height, rotrect.size.width) / MIN(rotrect.size.height, rotrect.size.width);
		//if (proportion < 2.25f/* || proportion > 3.75f*/)
		//	continue;
		

		// Not a "vertical bar" ?
		// Quel est le plus long côté ? 01 or 03
		
		// calcul des deux vecteur 01 et 03
		rotrect.points(vertices);
		v01.x = vertices[1].x - vertices[0].x;
		v01.y = vertices[1].y - vertices[0].y;
		norm01 =  sqrt(v01.x*v01.x + v01.y*v01.y);

		v03.x = vertices[2].x - vertices[1].x;
		v03.y = vertices[2].y - vertices[1].y;
		norm03 = sqrt(v03.x*v03.x + v03.y*v03.y);
		
		// Pour que notre Rectangle soit valide il faut que son plus long côté soit également le plus "vertical"
		
		if (norm01 > norm03) // 01 est le plus long
		{
			v01.x /= norm01;
			v01.y /= norm01;
			test = v01.y;

			shape.point[0] = vertices[0];
			shape.point[1] = vertices[1];
			shape.point[2] = vertices[2];
			shape.point[3] = vertices[3];
			shape.n01 = v01;
			shape.n03 = v03;
			shape.norm01 = norm01;
			shape.norm03 = norm03;

		}
		else // 03 est le plus long
		{
			v03.x /= norm03;
			v03.y /= norm03;
			test = v03.y;

			shape.point[0] = vertices[1];
			shape.point[1] = vertices[2];
			shape.point[2] = vertices[3];
			shape.point[3] = vertices[0];
			
			shape.n01 = v03;
			shape.n03 = v01;
			shape.norm01 = norm03;
			shape.norm03 = norm01;
		}

		// Reste à vérifier que le plus long est bien le plus "vertical"
		// normalement on devrait ic tester contre sinus 45° (=0.707...), mais on en profite pour eliminer un peu plus largement les candidats ...( 0.5f correspond au sinus d'un angle de 30¨°  )
		if(test > -0.5f) // Vertical Screen Axis is from top to bottom, ... donc la limite est négative...
			continue;

		//Fin de l'initialisation de la Vision Target
		// centre2d (attention il s'agit du centre du rectangle orienté, rien à voir avec le centre projeté du vision target que nous n'avons pas ... ! ), surface et contour associé
		shape.m_center = rotrect.center;
		shape.area = shape.norm01*shape.norm03;
		shape.contourId = i;

		// On défini les points haut et bas. Pour plus de facilité on repère ces point par leur ID dans le vector m_point 
		// ... ET on duplique également les coordonnées de ces 2 point dans m_lower et m_higher pour un accès plus direct et simple.
		if (shape.point[3].y > shape.point[0].y)
		{
			shape.m_lowerId = 3;
			shape.m_lower2D = shape.point[3];
		}
		else
		{
			shape.m_lowerId = 0;
			shape.m_lower2D = shape.point[0];
		}

		if (shape.point[2].y < shape.point[1].y)
		{
			shape.m_higherId = 2;
			shape.m_higher2D = shape.point[2];
		}
		else
		{
			shape.m_higherId = 1;
			shape.m_higher2D = shape.point[1];
		}

		shape.m_score = 0;
		shape.m_flags = 0;
		// Et on enregistre !
		visionTargets.push_back(shape);
	}

// ---------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------
// ETAPE 4	ANALYSE DE LA LISTE DES "VISION TARGET" 
//			En sortie d'étape 3 on a "laissé de côté" tous les contours détectés par OpenCV qui, de part leur forme, orientation et autre critères ne pouvait visiblement pas être des VISION TARGET.
//			On se retourve donc avec une liste de "VISION TARGET", mais seulement potentielles. En effet, parmis elles, se trouvent potentiellement des reflets, spots, parties de robots adverses réfléchissant
//			la lumière qui ont passés les tests avec succès et se sont fait passé pour des "Vision Target"	:(
//			
//			Le but de cette ETAPE 4 est de tenter de les repérer et de les écarter en utilisant une 'approche globale'.
//			En effet, on sait que les VRAIES VISION TARGETS sont alignées horizontalement les unes par rapports aux autres. 
//			Si on fait passer une droite par les points bas des VRAIES Vision target, on obtient une droite horizontale.
//			Evidement, la déformation perspective fait que cette droite horizontale semble oblique vue par la caméra mais la pente n'est jamais très forte et surtout elle est encadrée ( limite inf et sup...)
//			L'idée est donc de repérer les VISION TARGET qui,  quand on les associe avec les autres, ne forme pas (ou peu)  de paires alignées avec l'horizontale.
//			
	if (visionTargets.size() > 1)
	{

		int scoremax = 1;
		
		while(scoremax)
		{
			cout << scoremax << endl;

			scoremax = 0;

			for (size_t i = 0; i < visionTargets.size() - 1; i++)
			{
				if (BITGET(visionTargets[i].m_flags, VisionTarget::FLAG_DISABLE))
					continue;

				for (size_t j = i + 1; j < visionTargets.size(); j++)
				{
					if (BITGET(visionTargets[j].m_flags, VisionTarget::FLAG_DISABLE))
						continue;

					// Les Points Hauts des deux visions targets sont-ils suffisement alignés avec l'horizontale ? 
					u = visionTargets[i].m_higher2D - visionTargets[j].m_higher2D;
					unorm = sqrtf(u.x*u.x + u.y*u.y);
					//u.x /= unorm; Seul le y normalisé nous intéresse donc on evite le x
					u.y = fabs(u.y/unorm);

					// Les Points Bas des deux visions targets sont-ils suffisement alignés avec l'horizontale ? 
					v = visionTargets[i].m_lower2D - visionTargets[j].m_lower2D;
					vnorm = sqrtf(v.x*v.x + v.y*v.y);
					//v.x /= vnorm; Seul le y normalisé nous intéresse donc on evite le x
					v.y = vnorm;

					// INVALIDE  !
					if ((fabs(u.y) > VISIONTARGET_COUPLE_INCLINE_UPPERLIMIT) || (fabs(v.y) > VISIONTARGET_COUPLE_INCLINE_UPPERLIMIT))
					{
						visionTargets[j].m_score++;
						visionTargets[i].m_score++;

						scoremax = MAX( scoremax, MAX(visionTargets[i].m_score, visionTargets[j].m_score) );
					}
				}
			}

			// Suppression des "mauvais élèves" et 
			for (vector<VisionTarget>::iterator it = visionTargets.begin(); it != visionTargets.end();it++)
			{
				//assert(it->m_score <= scoremax);

				if (it->m_score == scoremax)
				{
					 BITSET(it->m_flags, VisionTarget::FLAG_DISABLE);
					//it = visionTargets.erase(it);
				}
				else
				{
					it->m_score = 0;
					//it++;
				}
			}
			// Et on recommence !
		}

		// Hop et voilà c'est fait ! Ne reste dans le vector que les Vision Target ayant le même et meilleur score 
	}

// ---------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------
// ETAPE 5 : ANALYSE DES "VISION TARGET" dans leur ensemble et construction de la liste de Paires
//			Les scotchs refléchissant que nous recherchons sont tous situés à la même hauteur. 
//			Donc, à l'image, l'ensemble des detectedshape retenues comme "bons candidats" devraient toutes être alignées sur un même plan en perspective.
//			Sur la fusée il y a une paire de scotch alignés, sur l'avant du cargo 2 paires de scotch alignés et sur chaque côté 3 paires de scotch alignés !!!
//			Les detected shape que nous avons retenu doivent donc vérifier ses configurations pour définitivement être VALIDEES et former des couples utilisables.   
//			
//			En entrée, la liste des "VisionTarget" retenue comme étant de bons candidats
//			En sortie, la liste de "DetectedCouple" Finale.
// ---------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------
	Point2f vdir;
	size_t k;
	detectedPairs.clear();

	if (visionTargets.size() > 1 )
	{
		// Pour commencer on trie le vector de shape selon leur coordonnée X de gauche à droite
		std::sort(visionTargets.begin(), visionTargets.end(), compare_x);

		// Association "basic" des shapes 2 a 2 en partant de la gauche.
		// La qualité de la paire est estimée par la fonction assemble. Si la note retrounée est négative on la rejette.
		VisionTargetPair pair;

		for (size_t i = 1; i < visionTargets.size(); i++)
		{
			if (pair.assemble(&visionTargets[i-1], &visionTargets[i]) >= 0.0f)
			{
				detectedPairs.push_back(pair);
			}
		}
		// AUTRES METHODE DE CONSTRUCTION DE PAIRES A EXPLORER ..... SI BESOIN, pour ecarter les "fausses paires" de vision target et ne garder que les Paires valides.
		//		En effet des elements parasites, comme un reflet, peuvent être confondu avec une Vision Target et du coup former de fausses paires !!!
		//		Comment faire , en regardant l'organisation des vision target détectées les unes par raport aux autres, pour deviner que telle ou telle shape détectée ne peut pas être une vision target ....
		//
		//		IDEES:
		//		-Recherche de la plus longue chaine "cohérente" angle ouvert vers le haut, puis ouvert vers le bas, puis vers le haut .... ( suppose qu'on soit sur le côté du cargo en train de regarder les 6 VT )
		//		-Une chaine Cohérente est composée de shapes alignées et de tailles diminuant ou grossissant selon un même facteur constant.
		/*
		for (size_t i = 0; i < visionTargets.size() - 1; i++)
		{
			for (size_t j = i+1; j < visionTargets.size(); j++)
			{

				if (pair.assemble(&visionTargets[i], &visionTargets[j]) > 0.0f)
				{
					detectedPairs.push_back(pair);
				}
			}
		}
		*/


		// Ensuite,on estime la position 3D de chaque Vision Target détectée
		for (size_t i = 0; i < visionTargets.size(); i++)
		{
			visionTargets[i].estimate3Dposition(&vision);
		}

		// Nous allons maintenant vérifier les shapes deux à deux de gauche à droite et vérifier que le couple ainsi formé est possible.


	}
	
	// From 2D detected visual Target to 3D Visual Target Couple
	/*
	vector<cv::Point2f> visiontargetcouplequad;
	visiontargetcouplequad.reserve(4);

	for (size_t i = 0; i < detectedPairs.size(); i++)
	{
		visiontargetcouplequad.clear();
		visiontargetcouplequad.push_back( detectedPairs[i].m_pvisionTargetA->m_higher2D );
		visiontargetcouplequad.push_back(detectedPairs[i].m_pvisionTargetB->m_higher2D);
		visiontargetcouplequad.push_back(detectedPairs[i].m_pvisionTargetB->m_lower2D);
		visiontargetcouplequad.push_back(detectedPairs[i].m_pvisionTargetA->m_lower2D);

		detectedPairs[i].b3Dsolved = cv::solvePnPRansac(vision.m_visionTarget3DModel, visiontargetcouplequad, vision.m_intrinsic, vision.m_distCoeffs, detectedPairs[i].m_rotation_vector, detectedPairs[i].m_translation_vector,100,2.0F,0.98);
	}
	*/


	//########## Approx Contours ##########
	/*
	approxContours.clear();
	approxContours.resize(contours.size());
	for (size_t i = 0; i < contours.size(); i++)
	{
		cv::approxPolyDP(contours[i], approxContours[i], 17, true);
	}
	*/

	/*//########## Filter Contours ##########
	filterContours.clear();
	for (size_t i = 0; i < approxContours.size(); i++)
	{
		//std::cout << "######## CONTOUR N " << i << " ########" << std::endl;
		cv::Rect boundRect = cv::boundingRect(approxContours[i]);

		double contourArea = cv::contourArea(approxContours[i]);
		//std::cout << "Area " << contourArea << std::endl;
		if (contourArea < 10)//(contourArea > maxArea || contourArea < minArea)
			continue;

		double ratio = (double)boundRect.width / boundRect.height;
		//std::cout << "Ratio " << ratio << std::endl;
		//if (contourArea > maxArea || contourArea < minArea)
		//	continue;

		//std::cout << "Angles " << approxContours[i].size() << std::endl;
		//if (approxContours[i].size() != 4)
		//	continue;

		filterContours.push_back(approxContours[i]);
	}*/
//	std::cout << "###### " << filterContours.size() << " contour trouve" << std::endl;


	//########## Calcul centroid ##########
	/*
	centroids.clear();
	centroids.resize(approxContours.size());

	real_centroid.x = 0.f;
	real_centroid.y = 0.f;

	for (size_t i = 0; i < approxContours.size(); i++)
	{
		for (size_t f = 0; f < approxContours[i].size(); f++)
		{
			centroids[i].x += approxContours[i][f].x;
			centroids[i].y += approxContours[i][f].y;
		}

		centroids[i].x /= approxContours[i].size();
		centroids[i].y /= approxContours[i].size();

		real_centroid.x += centroids[i].x;
		real_centroid.y += centroids[i].y;
	}
	real_centroid.x /= 2.f;
	real_centroid.y /= 2.f;
	target.x += (real_centroid.x - target.x)*t;
	target.y += (real_centroid.y - target.y)*t;
	*/

	//########## Calcul centroid II##########

	real_centroid.x = 0.f;
	real_centroid.y = 0.f;

/*
	float dx,dy,topdist,botdist;
	if (visionTargets.size()>1)
	{
		for (size_t i = 0; i < visionTargets.size() - 1; i++)
		{

			//real_centroid.x += rotatedRectangles[i].center.x;
			//real_centroid.y += rotatedRectangles[i].center.y;

			dx = visionTargets[i].middleBottom.x - visionTargets[i + 1].middleBottom.x;
			dy = visionTargets[i].middleBottom.y - visionTargets[i + 1].middleBottom.y;
			botdist = dx * dx + dy * dy;

			dx = visionTargets[i].middleTop.x - visionTargets[i + 1].middleTop.x;
			dy = visionTargets[i].middleTop.y - visionTargets[i + 1].middleTop.y;
			topdist = dx * dx + dy * dy;

			if (botdist > topdist)
			{
				real_centroid.x = (visionTargets[i].center.x + visionTargets[i+1].center.x)/2.0f;
				real_centroid.y = (visionTargets[i].center.y + visionTargets[i+1].center.y)/2.0f;
				centroid_size = 1.00f;
				break;
			}
			else
			{
				real_centroid.x = 0;
				real_centroid.y = 0;
				centroid_size = 0.25f;
			}
		}

		//real_centroid.x /= rotatedRectangles.size();
		//real_centroid.y /= rotatedRectangles.size();
	}
	else
	{
		real_centroid.x = 0;
		real_centroid.y = 0;
		centroid_size = 0.25f;
	}

	// Smoothed target
	target.x += (real_centroid.x - target.x)*t;
	target.y += (real_centroid.y - target.y)*t;
*/



	/*double angle;
	if(filterContours.size() != 0)
	{
		cv::Rect boundRect = cv::boundingRect(filterContours[0]);
		double centerX = boundRect.x + (boundRect.width / 2);
		double angle_rad = atan((centerX- (width/2)) / distance_focale);
		angle = angle_rad * (180/M_PI);
	}
	else
	{
		angle = 0;
	}
	std::cout << "Angle " << angle << std::endl << std::endl;

	if (entry.Exists())
	{
		entry.SetDouble(angle);
		std::cout << "Angle envoyé" << std::endl << std::endl;
	}*/
}

void ShowFrames()
{
	Point2f		v,v0, v1, cam2D;
	char		str[256];
	float		f;
	// Effacage image vue de top
	Render3DImage = cv::Scalar(0, 0, 0);



	// Affichage des couples de VisionTarget
	vector<cv::Point2f> projectedPoints;

	for (size_t i = 0; i < visionTargets.size(); i++)
	{
		hud::drawVisionTargetRotatedRectangle(FilteredImage, visionTargets[i]);
	}


	for (size_t i = 0; i < detectedPairs.size(); i++)
	{

		// Affichage des deux Vision Target du couple
		hud::drawVisionTargetRotatedRectangle(FilteredImage, *detectedPairs[i].m_pvisionTargetA);
		hud::drawVisionTargetRotatedRectangle(FilteredImage, *detectedPairs[i].m_pvisionTargetB);

		// Affichage du "Quadrilatere" du couple
		hud::drawVisionTargetCoupleQuad(FilteredImage, detectedPairs[i]);

		if (detectedPairs[i].m_quality >= 0)
		{

/*
			// Affichage du Ratio de hauteur VTA VTB du couple
			f = MIN(detectedPairs[i].m_pvisionTargetA->norm01, detectedPairs[i].m_pvisionTargetB->norm01) / MAX(detectedPairs[i].m_pvisionTargetA->norm01, detectedPairs[i].m_pvisionTargetB->norm01);
			v0 = detectedPairs[i].m_center;
			v0.x -= 5;
			v0.y += 50;
			sprintf(str, "rH %.4f", f);
			cv::putText(RawImage, str, v0, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));

			Point2f v = detectedPairs[i].m_pvisionTargetB->m_higher2D - detectedPairs[i].m_pvisionTargetA->m_higher2D;
			float l = sqrtf(v.x*v.x + v.y + v.y);
			float r = MIN(detectedPairs[i].m_pvisionTargetA->norm01, detectedPairs[i].m_pvisionTargetB->norm01) / l;
			v0.y += 50;
			sprintf(str, "rQ %.4f", r);
			cv::putText(FilteredImage, str, v0, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));

			// Pente
			Point2f topv = detectedPairs[i].m_pvisionTargetB->m_higher2D - detectedPairs[i].m_pvisionTargetA->m_higher2D;
			float topnorm = sqrtf(topv.x*topv.x + topv.y + topv.y);
			//basev.x /= basenorm; !!! Inutile de normaliser basev.x, car on ne l'utilise pas.
			topv.y /= topnorm;

			Point2f basev = detectedPairs[i].m_pvisionTargetB->m_lower2D - detectedPairs[i].m_pvisionTargetA->m_lower2D;
			float basenorm = sqrtf(basev.x*basev.x + basev.y + basev.y);
			//basev.x /= basenorm; !!! Inutile de normaliser basev.x, car on ne l'utilise pas.
			basev.y /= basenorm;
			v0.y += 50;
			sprintf(str, "pB:%.4f p T:%.4f", basev.y, topv.y);
			cv::putText(RawImage, str, v0, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));
			v0.y += 50;
			sprintf(str, "Quality:%.4f", detectedPairs[i].m_quality);
			cv::putText(FilteredImage, str, v0, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));
*/

			// Affichage angle horizontal estimé en Degré 
			f = NRADtoDEG(vision.estimateIntrinsicHorizontalAngleRad(detectedPairs[i].m_center[VisionTargetPair::TRAPEZOID]));
			v0 = detectedPairs[i].m_center[VisionTargetPair::TRAPEZOID];
			v0.x -= 10;
			v0.y += 50;
			sprintf(str, ":%.1f", f);
			cv::putText(RawImage, str, v0, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));
			
			// Affichage Ditance Z par rapport à la camera en cm !!! Distance non corrigée en cas de non-alignement avec l'horizontal ...
			f = vision.estimateIntrinsicDistanceZ(detectedPairs[i].m_center[VisionTargetPair::TRAPEZOID], REAL_Y_VISIONTARGETPAIR_CENTER);
			v0.y += 30;
			sprintf(str, ":%.1f", f);
			cv::putText(RawImage, str, v0, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));


		}
		/*
		// rQp
		v = detectedPairs[i].m_pvisionTargetB->m_lower2D - detectedPairs[i].m_pvisionTargetA->m_lower2D;
		float a = sqrtf(v.x*v.x + v.y + v.y);
		v = detectedPairs[i].m_pvisionTargetB->m_higher2D - detectedPairs[i].m_pvisionTargetA->m_higher2D;
		float b = sqrtf(v.x*v.x + v.y + v.y);
		r = (detectedPairs[i].m_pvisionTargetA->norm01*detectedPairs[i].m_pvisionTargetB->norm01) / (a*b);
		v0.y += 50;
		sprintf(str, "rQp %f", r);
		cv::putText(RawImage, str, v0, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));
		*/
		
		
		
		/*
		// Affichage 3D
		if (detectedPairs[i].b3Dsolved)
		{
			projectedPoints.clear();
			projectPoints(vision.m_visionTarget3DModel, detectedPairs[i].m_rotation_vector, detectedPairs[i].m_translation_vector, vision.m_intrinsic, vision.m_distCoeffs, projectedPoints);
			
			for (size_t j = 1; j < projectedPoints.size(); j++)
			{
				cv::line(RawImage, projectedPoints[j-1], projectedPoints[j], cv::Scalar(255, 0, 0), 2);
			}
			cv::line(RawImage, projectedPoints[projectedPoints.size() - 1], projectedPoints[0], cv::Scalar(255, 0, 0), 2);

			draw3DBase(RawImage, detectedPairs[i].m_rotation_vector, detectedPairs[i].m_translation_vector, 25);

			// Infos 3D
			
			cout << detectedPairs[i].m_rotation_vector << endl;
			//cout << detectedPairs[i].m_translation_vector.at<double>(0,0) << endl;
			//cout << detectedPairs[i].m_translation_vector.at<double>(1, 0) << endl;
			//cout << detectedPairs[i].m_translation_vector.at<double>(2, 0) << endl;
			
			// Vue de Top Position et orientation de la paire
			cam2D.x = (float)Render3DImage.size().width / 2.0f;
			cam2D.y = (float)Render3DImage.size().height / 2.0f;

			v.x = detectedPairs[i].m_translation_vector.at<double>(0, 0)*SCALEFACTOR3D;
			v.y = detectedPairs[i].m_translation_vector.at<double>(2, 0)*SCALEFACTOR3D;
			float f = sqrtf(v.x*v.x + v.y*v.y);
			v0.x = cam2D.x + v.x;
			v0.y = cam2D.y - v.y;
			v1.x = v0.x + detectedPairs[i].m_rotation_vector.at<double>(0, 0)*SCALEFACTOR3D*50;
			v1.y = v0.y - detectedPairs[i].m_rotation_vector.at<double>(2, 0)*SCALEFACTOR3D*50;
			arrowedLine(Render3DImage, v0, v1, Scalar(0, 255, 0));
			
	
			sprintf(str, "distance au sol %f", f );
			v0.x += 10;
			putText(Render3DImage, str, v0, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));
		}
		*/

	}


	// HUD: Affichage Information Sur ecran video ( retour pilote )
	hud::drawSight(RawImage,50.0f);

	float horizon_limit = RawImage.size().height*(float)horizon / 100.0f;
	v0.x = 0.0f;
	v0.y = horizon_limit;
	v1.x = RawImage.size().width;
	v1.y = horizon_limit;
	line(FilteredImage, v0, v1, Scalar(255, 255, 0));

	// Rendu 3D VUE DE TOP
	TopView::render(Render3DImage);


	cv::imshow("RAW", RawImage);
	cv::imshow("FilteredImage", FilteredImage);
	cv::imshow("TOPVIEW", Render3DImage);

	//Pour l'affichage il faut attendre avant d'actualiser
	//Pour l'affichage il faut attendre avant d'actualiser
	if (waitKey(50) == ' ')
	{
		BITTOGLE(RoboLyonFlags, FLAG_CAPTURE_PAUSE);
	}
}


//								ANCIENNE VERSION EN DESSOUS
// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------

void ShowFrames_old()
{
#ifdef _FILETRING_A
	imshow("HLS", HlsImage);
#endif

	Point2f vertices[4];
	Point2f textpos;
	Point2f v0, v1, v2, cam, screencenter,xcenter;
	
	Point3f v3a, v3b;

	float f,r,s0,s1;
	float n0;
	float asB = 0, asT = 0, asC = 0,target_size;

	float hullarea0, hullarea1 , rectarea0, rectarea1;

	char  str[256];
	
	target_size = (float)RawImage.size().height / 4.0f;
	screencenter.x = (float)RawImage.size().width / 2.0f;
	screencenter.y = (float)RawImage.size().height / 2.0f;

	v0.x = screencenter.x - target_size;	v0.y = screencenter.y;
	v1.x = screencenter.x + target_size;	v1.y = screencenter.y;
	line(RawImage,v0 , v1, Scalar(0, 0, 255));

	v0.x = screencenter.x;	v0.y = screencenter.y - target_size;
	v1.x = screencenter.x;	v1.y = screencenter.y + target_size;
	line(RawImage, v0, v1, Scalar(0, 0, 255));


#ifdef PRECOMPIL_IMAGEFILTER_CANNY_AND_THRESHOLD
	cv::cvtColor(FilteredImage, FilteredImage, cv::COLOR_GRAY2RGB);
#endif



	// ----------------------------------------------------------------------------------------------------------------------
	// ----------------------------------------------------------------------------------------------------------------------
	// FENETRE VUE DE TOP 3D
	// Effacage image vue de top
	Render3DImage = cv::Scalar(0, 0, 0);
	// Tracage cone camera en vue de top

	cam.x = (float)(Render3DImage.size().width / 2);
	cam.y = (float)(Render3DImage.size().height - Render3DImage.size().height / 4);


	// axe camera
	v0.x = (float)vision.m_widthoutof2;
	v0.y = 0;
	//vision.get3DPointOnHorizontalPlane(v3a, v0, REAL_PLANEY_ID_VISIONTARGET_TOP);
	v1.x = v3a.x*SCALEFACTOR3D+cam.x;
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
	v1.y = cam.y-v3a.z*SCALEFACTOR3D;
	line(Render3DImage, cam, v1, Scalar(0, 0, 255));

	for (size_t i=0; i < visionTargets.size(); i++)
	{
		v0.x = visionTargets[i].m_lower3D.x*SCALEFACTOR3D + cam.x;
		v0.y = cam.y - visionTargets[i].m_lower3D.z*SCALEFACTOR3D -5;
		v1.x = v0.x;
		v1.y = v0.y + 5;
		line(Render3DImage, v0, v1, Scalar(0, 255, 255));

		v0.x = visionTargets[i].m_higher3D.x*SCALEFACTOR3D + cam.x;
		v0.y = cam.y - visionTargets[i].m_higher3D.z*SCALEFACTOR3D - 5;
		v1.x = v0.x;
		v1.y = v0.y + 5;
		line(Render3DImage, v0, v1, Scalar(255, 255, 0));


		// VUE DE TOP TRACAGE DES LIGNES Visio Target Top et bottom et center
		if (i > 0)
		{
			// Vue de TOP: tracage lignes passant par le BAS des vision target
			v0.x = visionTargets[i].m_lower3D.x*SCALEFACTOR3D + cam.x;
			v0.y = cam.y - visionTargets[i].m_lower3D.z*SCALEFACTOR3D;
			v1.x = visionTargets[i - 1].m_lower3D.x*SCALEFACTOR3D + cam.x;
			v1.y = cam.y - visionTargets[i - 1].m_lower3D.z*SCALEFACTOR3D;
			line(Render3DImage, v0, v1, Scalar(0, 255, 255));

			// angle du segement avec (v1v0) l'axe z de la camera, coincidant ici, en vue de top 2D avec l'axe y (0,1)
			v0.x -= v1.x;
			v0.y -= v1.y;
			n0 = sqrt(v0.x*v0.x + v0.y*v0.y);
			//v0.x /= n0;
			asB += v0.y / n0;


			// Vue de TOP: tracage lignes passant par le HAUT des vision target
			v0.x = visionTargets[i].m_higher3D.x*SCALEFACTOR3D + cam.x;
			v0.y = cam.y - visionTargets[i].m_higher3D.z*SCALEFACTOR3D;
			v1.x = visionTargets[i - 1].m_higher3D.x*SCALEFACTOR3D + cam.x;
			v1.y = cam.y - visionTargets[i - 1].m_higher3D.z*SCALEFACTOR3D;
			line(Render3DImage, v0, v1, Scalar(0, 255, 0));

			// angle du segement avec (v1v0) l'axe z de la camera, coincidant ici, en vue de top 2D avec l'axe y (0,1)
			v0.x -= v1.x;
			v0.y -= v1.y;
			n0 = sqrt(v0.x*v0.x + v0.y*v0.y);
			//v0.x /= n0;
			asT += v0.y / n0;

			// Vue de TOP: tracage lignes passant par le CENTRE des vision target
			/*
			v0.x = visionTargets[i].m_center3D.x*SCALEFACTOR3D + cam.x;
			v0.y = cam.y - visionTargets[i].m_center3D.z*SCALEFACTOR3D;
			v1.x = visionTargets[i - 1].m_center3D.x*SCALEFACTOR3D + cam.x;
			v1.y = cam.y - visionTargets[i - 1].m_center3D.z*SCALEFACTOR3D;
			line(Render3DImage, v0, v1, Scalar(0, 0, 255));
			*/
			// angle du segement avec (v1v0) l'axe z de la camera, coincidant ici, en vue de top 2D avec l'axe y (0,1)
			v0.x -= v1.x;
			v0.y -= v1.y;
			n0 = sqrt(v0.x*v0.x + v0.y*v0.y);
			//v0.x /= n0;
			asC += v0.y / n0;

		}

	}
	if (visionTargets.size() > 1)
	{
		asB /= ((float)visionTargets.size() - 1);
		asT /= ((float)visionTargets.size() - 1);
		asC /= ((float)visionTargets.size() - 1);

		v0.x = 10;
		v0.y = 20;
		float an = (float)NRADtoDEG(asinf((asT)));

		sprintf(str, "Moyenne Top %.5f Angle:%.3g Deg ", asT,an);
		cv::putText(Render3DImage, str, v0, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));

		an = (float)NRADtoDEG(asinf((asB)));
		sprintf(str, "Moyenne Bot %.5f Angle:%.3g Deg ", asB, an);
		v0.y += 20;
		cv::putText(Render3DImage, str, v0, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));

		an = (float)NRADtoDEG(asinf((asC)));
		sprintf(str, "Moyenne Centre %.5f Angle:%.3g Deg ", asC,an);
		v0.y += 20;
		cv::putText(Render3DImage, str, v0, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));

		asT = (asT + asB + asC) / 3.0f;
		an = (float)NRADtoDEG(asinf((asT)));
		sprintf(str, "Moyenne %.5f Angle:%.3g Deg  ",asT,an  );
		v0.y += 20;
		cv::putText(Render3DImage, str, v0, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));


		sprintf(str, "height: %.3f Fov: %.3f", vision.m_cameraYworld, vision.m_fovY);
		v0.y += 20;
		cv::putText(Render3DImage, str, v0, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));
	}


	// ----------------------------------------------------------------------------------------------------------------------
	// ----------------------------------------------------------------------------------------------------------------------


	// ----------------------------------------------------------------------------------------------------------------------
	// ----------------------------------------------------------------------------------------------------------------------
	// FENETRE FILTRE ET DETECTION

	drawContours(FilteredImage, hull,-1, cv::Scalar(0, 0, 255), 1);


	for (size_t i = 0; i < visionTargets.size(); i++)
	{
		
		// 01: Vertical axis
		line(FilteredImage, visionTargets[i].point[0], visionTargets[i].point[(1)], Scalar(100, 255,255));
		// 12: // to Horizontal axis
		line(FilteredImage, visionTargets[i].point[1], visionTargets[i].point[(2)], Scalar(100, 255,255));
		// 23: // to Vertical axis
		line(FilteredImage, visionTargets[i].point[2], visionTargets[i].point[(3)], Scalar(100, 255,255));
		// 30: Horizontal axis
		line(FilteredImage, visionTargets[i].point[0], visionTargets[i].point[(3)], Scalar(100, 255,255));
		
		cv::putText(FilteredImage, "0", visionTargets[i].point[0], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));
		cv::putText(FilteredImage, "1", visionTargets[i].point[1], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));
		cv::putText(FilteredImage, "2", visionTargets[i].point[2], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 128, 128));
		cv::putText(FilteredImage, "3", visionTargets[i].point[3], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));

		// Center Cross
		v0 = visionTargets[i].m_center; v0.y -= 5.0f;
		v1 = visionTargets[i].m_center; v1.y += 5.0f;
		line(FilteredImage, v0, v1, Scalar(0, 255, 255));
		
		v0.x -= 5.0f; v0.y += 5.0f;
		v1.x += 5.0f; v1.y -= 5.0f;
		line(FilteredImage, v0, v1, Scalar(0, 255, 255));

		// FITLINE
		v0.x = fitLines[i][2] - 50.0f*fitLines[i][0];
		v0.y = fitLines[i][3] - 50.0f*fitLines[i][1];
		v1.x = fitLines[i][2] + 50.0f*fitLines[i][0];
		v1.y = fitLines[i][3] + 50.0f*fitLines[i][1];
		line(FilteredImage, v0, v1, Scalar(0, 0, 255));

		/*
		if (i > 0)
		{
			// cross product
			f = visionTargets[i - 1].n01.cross(visionTargets[i].n01);
			v0.x = (visionTargets[i - 1].center.x + visionTargets[i].center.x)*0.5f;
			v0.y = (visionTargets[i - 1].center.y + visionTargets[i].center.y)*0.5f + 25.0f;
			sprintf(str, "X%.3f",f);
			cv::putText(FilteredImage, str, v0, cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 255, 255));
			
			// raport de surface
			v0.y += 20.0f;
			f = (visionTargets[i - 1].area / visionTargets[i].area);
			sprintf(str, "S%.3f", f);
			cv::putText(FilteredImage, str, v0, cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 255, 255));
			
			// raport de hauteur
			v0.y += 20.0f;
			f = (visionTargets[i - 1].norm01 / visionTargets[i].norm01);
			sprintf(str, "H%.3f", f);
			cv::putText(FilteredImage, str, v0, cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 255, 255));
		}

		f = f = (visionTargets[i].norm01 / visionTargets[i].norm03);
		v0.x = visionTargets[i].center.x;
		v0.y = visionTargets[i].center.y + 35.0f;
		sprintf(str, "wh%.3f", f);
		cv::putText(FilteredImage, str, v0, cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 255, 255));
		*/

		// mass center
		v0.x = (float)(moms[i].m10 / moms[i].m00);
		v0.y = (float)(moms[i].m01 / moms[i].m00);

		v1.x = v0.x - 5;
		v1.y = v0.y;
		v2.x = v0.x + 5;
		v2.y = v0.y;
		line(FilteredImage, v0, v1, Scalar(0, 0, 255));

		v1.x = v0.x;
		v1.y = v0.y- 5;
		v2.x = v0.x;
		v2.y = v0.y + 5;
		line(FilteredImage, v0, v1, Scalar(0, 0, 255));

		if (i > 0)
		{
			// Segments croisés par paire
			if (visionTargets[i - 1].n01.cross(visionTargets[i].n01) > 0.0f)
			{
	//			line(RawImage, visionTargets[i - 1].point[visionTargets[i - 1].higherId], visionTargets[i].point[visionTargets[i].lowerId], Scalar(0, 0, 255));
	//			line(RawImage, visionTargets[i - 1].point[visionTargets[i - 1].lowerId], visionTargets[i].point[visionTargets[i].higherId], Scalar(0, 0, 255));
			}
			else
			{
		//		line(RawImage, visionTargets[i - 1].point[visionTargets[i - 1].higherId], visionTargets[i].point[visionTargets[i].lowerId], Scalar(0, 255, 0));
		//		line(RawImage, visionTargets[i - 1].point[visionTargets[i - 1].lowerId], visionTargets[i].point[visionTargets[i].higherId], Scalar(0, 255, 0));
			}

		//	line(RawImage, visionTargets[i - 1].point[visionTargets[i - 1].higherId], visionTargets[i].point[visionTargets[i].higherId], Scalar(0, 255, 255));
		//	line(RawImage, visionTargets[i - 1].point[visionTargets[i - 1].lowerId], visionTargets[i].point[visionTargets[i].lowerId], Scalar(0, 255, 255));


			// Infos/rapports
			/*
			f = contourArea(hull[visionTargets[i].contourId], false);
			v0.x = visionTargets[i].center.x;
			v0.y = visionTargets[i].center.y + 75.0f;
			sprintf(str, "Hull %.3f", f);
			cv::putText(RawImage, str, v0, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));

			f = visionTargets[i].area;
			v0.y += 15.0f;
			sprintf(str, "Rect %.3f", f);
			cv::putText(RawImage, str, v0, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));
			*/
			/*
			v0.y += 15.0f;
			sprintf(str, "%d cnt", contours[visionTargets[i].contourId].size());
			cv::putText(FilteredImage, str, v0, cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 255, 255));

			v0.y += 15.0f;
			sprintf(str, "%d Hull", hull[visionTargets[i].contourId].size());
			cv::putText(FilteredImage, str, v0, cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 255, 255));

			v0.y += 15.0f;
			sprintf(str, "%d aprx", approxContours[visionTargets[i].contourId].size());
			cv::putText(FilteredImage, str, v0, cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 255, 255));
			*/

//			if (tools.SegXSeg(&visionTargets[i - 1].point[visionTargets[i - 1].higherId], &visionTargets[i].point[visionTargets[i].lowerId],
	//			&visionTargets[i - 1].point[visionTargets[i - 1].lowerId], &visionTargets[i].point[visionTargets[i].higherId],
		//		&xcenter))

			// point d'intersection des segments et surface des deux triangles:
			if (Tools::SegXSeg(visionTargets[i - 1].m_higher2D, visionTargets[i].m_lower2D, visionTargets[i - 1].m_lower2D, visionTargets[i].m_higher2D,xcenter) )
			{
				v0 = xcenter;
				v0.y -= 5;
				v1 = xcenter;
				v1.y += 5;
				line(RawImage, v0,v1, Scalar(255, 255, 255));
				v0 = xcenter;
				v0.x -= 5;
				v1 = xcenter;
				v1.x += 5;
				line(RawImage, v0, v1, Scalar(255, 255, 255));

				// Surface du triangle de gauche
				s0 = Tools::TriangleSurface(visionTargets[i - 1].m_higher2D, xcenter, visionTargets[i - 1].m_lower2D);
				// Surface du triangle de Droite
				s1 = Tools::TriangleSurface(visionTargets[i].m_higher2D, xcenter, visionTargets[i].m_lower2D);
				// Rapport
				r = s0 / s1;

				v0.x = visionTargets[i].m_center.x;
				v0.y = visionTargets[i].m_center.y + 50.0f;
				
				float angle = 288 - 270 * r + 66.2*r*r;
				sprintf(str, "TRI %.1f", angle );
				cv::putText(RawImage, str, v0, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));

				r = visionTargets[i - 1].area / visionTargets[i].area;
				angle = 203 - 143 * r + 28.3*r*r;
				sprintf(str, "RCT %.1f", angle);
				v0.y += 20.0f;
				cv::putText(RawImage, str, v0, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));

				// Dot product N1xN1
				v0.x = xcenter.x;
				v0.y = xcenter.y + 115.0f;
				sprintf(str, "N1.N1= %.3f", visionTargets[i - 1].n01.dot(visionTargets[i].n01));
				cv::putText(RawImage, str, v0, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));


/*
				// Infos/rapports sur VIDEO ... beurk a refaire
				if (i == 1)
				{
					hullarea0 = contourArea(hull[visionTargets[i-1].contourId], false);
					v0.x = visionTargets[i-1].center.x;
					v0.y = visionTargets[i-1].center.y + 115.0f;
					sprintf(str, "Hull %.2f", hullarea0);
					cv::putText(RawImage, str, v0, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));

					rectarea0 = visionTargets[i-1].area;
					v0.y += 15.0f;
					sprintf(str, "Rect %.2f", rectarea0);
					cv::putText(RawImage, str, v0, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));
				
					v0.y += 15.0f;
					sprintf(str, "TRI %.2f", s0);
					cv::putText(RawImage, str, v0, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));
				}

				hullarea1 = contourArea(hull[visionTargets[i].contourId], false);
				v0.x = visionTargets[i].center.x + 100.0f;
				v0.y = visionTargets[i].center.y + 115.0f;
				sprintf(str, "Hull %.2f", hullarea1);
				cv::putText(RawImage, str, v0, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));

				rectarea1 = visionTargets[i].area;
				v0.y += 15.0f;
				sprintf(str, "Rect %.2f", rectarea1);
				cv::putText(RawImage, str, v0, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));
				
				v0.y += 15.0f;
				sprintf(str, "TRI %.2f", s1);
				cv::putText(RawImage, str, v0, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));
*/
		

			//	v0.y += 20.0f;
			//	sprintf(str, "%.3f",r);
			//	cv::putText(RawImage, str, v0, cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 255, 255));
			}

		}
		 

		// Infos/rapports
		f = contourArea(hull[visionTargets[i].contourId],false);
		v0.x = visionTargets[i].m_lower2D.x;
		v0.y = visionTargets[i].m_lower2D.y + 10.0f;
		sprintf(str, "CS %.3f", f);
		cv::putText(FilteredImage, str, v0, cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 255, 255));
		
		f = visionTargets[i].area;
		v0.y += 15.0f;
		sprintf(str, "RS %.3f", f);
		cv::putText(FilteredImage, str, v0, cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 255, 255));
		
		v0.y += 15.0f;
		sprintf(str, "N01( %.3f,%.3f )", visionTargets[i].n01.x, visionTargets[i].n01.y);
		cv::putText(FilteredImage, str, v0, cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 255, 255));


		/*
		v0.y += 15.0f;
		sprintf(str, "%d cnt", contours[visionTargets[i].contourId].size());
		cv::putText(FilteredImage, str, v0, cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 255, 255));

		v0.y += 15.0f;
		sprintf(str, "%d Hull", hull[visionTargets[i].contourId].size());
		cv::putText(FilteredImage, str, v0, cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 255, 255));

		v0.y += 15.0f;
		sprintf(str, "%d aprx", approxContours[visionTargets[i].contourId].size());
		cv::putText(FilteredImage, str, v0, cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 255, 255));
		*/

	}
	/*
	for (size_t i = 0; i < detectedPairs.size(); i++)
	{
		line(FilteredImage, detectedPairs[i].m_pvisionTargetA->middleTop, detectedPairs[i].m_pvisionTargetB->middleTop, Scalar(255, 255, 0));
		line(FilteredImage, detectedPairs[i].m_pvisionTargetA->middleBottom, detectedPairs[i].m_pvisionTargetB->middleBottom, Scalar(0, 255, 255));
	}
	putText(FilteredImage, "O", target, cv::FONT_HERSHEY_SIMPLEX, centroid_size, cv::Scalar(255, 0, 255));
	*/
	float horizon_limit = RawImage.size().height*(float)horizon / 100.0f;
	v0.x = 0.0f;
	v0.y = horizon_limit;
	v1.x = RawImage.size().width;
	v1.y = horizon_limit;
	line(FilteredImage, v0,v1, Scalar(255, 255, 0));

	cv::imshow("RAW", RawImage);;
	imshow("FilteredImage", FilteredImage);
	imshow("TOPVIEW", Render3DImage);


}

 
