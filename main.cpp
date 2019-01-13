#include <iostream>
#include <chrono>
#include <vector>
#include <memory>

#include "opencv2/opencv.hpp"
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>


using namespace cv;
using namespace std;

/* Camera Specs and Configurations */
/* Note:  Higher resolution & framerate is possible, depending upon processing cpu usage */
cv::VideoCapture camera;
const double width = 800;
const double height = 600;
const int frames_per_sec = 15;
const double fov = 68.5;
const double fov_rad = fov * (M_PI/180);
const double distance_focale = width / (2*tan(fov_rad/2));

/* Matrices used for processing */
cv::Mat input;
cv::Mat hsl;
cv::Mat thresholdOutput;
cv::Mat openOutput;
cv::Mat findContoursOutput;
cv::Mat approxContoursOutput;
cv::Mat filterContoursOutput;
cv::Mat rotatedRectanglesOutput;

/* Vector of Vectors of Points = contours */
std::vector<std::vector<cv::Point> > contours;
std::vector<std::vector<cv::Point> > filterContours;
std::vector<cv::RotatedRect> rotatedRectangles;

/* Network Tables */
nt::NetworkTableInstance inst;
std::shared_ptr<nt::NetworkTable> table;
nt::NetworkTableEntry entry;


void Init();
bool CaptureFrame();
void ProcessFrame();
void ShowFrames();


int main()
{
	time_t timestamp_debut = std::time (0);

    Init();
    while(true)
    {
        if(CaptureFrame())
        {
            ProcessFrame();
            ShowFrames();
        }
    }

    time_t timestamp_fin = std::time (0);
	std::cout << "Programme terminé au bout de " << timestamp_fin - timestamp_debut << " secondes" << std::endl;

    return 0;
}

void Init()
{
    /* Open camera and set properties */
    camera.open(1);
	camera.set(cv::CAP_PROP_FRAME_WIDTH, width);
	camera.set(cv::CAP_PROP_FRAME_HEIGHT, height);
	camera.set(cv::CAP_PROP_FPS, frames_per_sec);
	camera.set(cv::CAP_PROP_BRIGHTNESS, 0);

    /* Create tables */
	/*inst = nt::NetworkTableInstance::GetDefault();
	table = inst.GetTable("datatable");
	entry = table->GetEntry("Angle");
	inst.StartClientTeam(5553);*/
}


bool CaptureFrame()
{
    if(!camera.read(input))
    {
        std::cout << "Error Grabbing Video Frame"  << std::endl;
        return false;
    }
    return true;
}


void ProcessFrame()
{
    //########## BGR TO HLS ##########
    cv::cvtColor(input, hsl, cv::COLOR_BGR2HLS);


    //########## Threshold ##########
    //cv::inRange(input, Scalar(lowH, lowL, lowS), Scalar(highH, highL, highS), output);
    cv::inRange(hsl, cv::Scalar(50, 130, 150), cv::Scalar(95, 255, 255), thresholdOutput);


    //########## Erode and Dilate ##########
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3), cv::Point(-1, -1));
    cv::morphologyEx(thresholdOutput, openOutput, cv::MORPH_OPEN, kernel);


    //########## Find Contours ##########
    contours.clear();
    cv::findContours(openOutput, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::cout << "############### " << contours.size() << " CONTOURS DETECTES ###############" << std::endl;


    //########## Filter Contours ##########
    filterContours.clear();
    rotatedRectangles.clear();
    for (size_t i = 0; i < contours.size(); i++)
    {
        //Rotated rectangle autour du contour
        RotatedRect rect = minAreaRect(contours[i]);
        int height = rect.size.height;
        int width = rect.size.width;

        //Si l'aire < 450 on ne garde pas
        double area = height * width;
        if(area < 750)
            continue;
        
        //Calcul du ratio : si ratio < 1 alors on prend l'inverse
        double ratio = double(height)/double(width);
        if(ratio < 1)
            ratio = 1/ratio;
        //Si ratio < 1 ou ration > 5 on ne garde pas
        if(abs(3.0 - ratio) > 2.0)
            continue;

        //Sinon on prend le contour et son rectangle
        rotatedRectangles.push_back(rect);
        filterContours.push_back(contours[i]);
    }
    std::cout << "###### " << filterContours.size() << " contour filtrés trouve" << std::endl;

    
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
    cv::imshow("input", input);

    cv::imshow("threshold", thresholdOutput);

    cv::imshow("erode and dilate", openOutput);

    findContoursOutput = input.clone();
    cv::drawContours(findContoursOutput, contours, -1, cv::Scalar(0, 0, 255), 3);
    cv::imshow("find contours", findContoursOutput);

    filterContoursOutput = input.clone();
    cv::drawContours(filterContoursOutput, filterContours, -1, cv::Scalar(255, 0, 0), 3);
    cv::imshow("filtered contours", filterContoursOutput);

    rotatedRectanglesOutput = input.clone();
    for(size_t i = 0; i < rotatedRectangles.size(); i++)
    {
        //Affichage du rectangle
        Point2f cotes[4];
        rotatedRectangles[i].points(cotes);
        for (int i = 0; i < 4; i++)
            line(rotatedRectanglesOutput, cotes[i], cotes[(i+1)%4], Scalar(150,0,150), 3);
    }
    cv::imshow("rect", rotatedRectanglesOutput);
	
	//Pour l'affichage il faut attendre avant d'actualiser
	cv::waitKey(30);
}
