#include <iostream>
#include <chrono>
#include <vector>
#include <memory>

#include "opencv2/opencv.hpp"
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>


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

/* Vector of Vectors of Points = contours */
std::vector<std::vector<cv::Point> > contours;
std::vector<std::vector<cv::Point> > approxContours;
std::vector<std::vector<cv::Point> > filterContours;

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
    for(int i = 0; i < 100; i++)
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
    camera.open(0);
	camera.set(cv::CAP_PROP_FRAME_WIDTH, width);
	camera.set(cv::CAP_PROP_FRAME_HEIGHT, height);
	camera.set(cv::CAP_PROP_FPS, frames_per_sec);
	camera.set(cv::CAP_PROP_BRIGHTNESS, 0);

    /* Create tables */
	inst = nt::NetworkTableInstance::GetDefault();
	table = inst.GetTable("datatable");
	entry = table->GetEntry("Angle");
	inst.StartClientTeam(5553);
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
    cv::inRange(hsl, cv::Scalar(43, 225, 225), cv::Scalar(87, 255, 255), thresholdOutput);


    //########## Erode and Dilate ##########
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3), cv::Point(-1, -1));
    cv::morphologyEx(thresholdOutput, openOutput, cv::MORPH_OPEN, kernel);


    //########## Find Contours ##########
    contours.clear();
    cv::findContours(openOutput, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::cout << "############### " << contours.size() << " CONTOURS DETECTES ###############" << std::endl;


    //########## Approx Contours ##########
    approxContours.clear();
    approxContours.resize(contours.size());
    for (size_t i = 0; i < contours.size(); i++)
    {
        cv::approxPolyDP(contours[i], approxContours[i], 17, true);
    }


    //########## Filter Contours ##########
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
    }
    std::cout << "###### " << filterContours.size() << " contour trouve" << std::endl;

    
    double angle;
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
    }
}


void ShowFrames()
{
    cv::imshow("input", input);

    cv::imshow("threshold", thresholdOutput);

    cv::imshow("erode and dilate", openOutput);

    findContoursOutput = openOutput.clone();
    cv::cvtColor(findContoursOutput, findContoursOutput, cv::COLOR_GRAY2RGB);
    cv::drawContours(findContoursOutput, contours, -1, cv::Scalar(0, 0, 255), 3);
    cv::imshow("find contours", findContoursOutput);

    approxContoursOutput = openOutput.clone();
    cv::cvtColor(approxContoursOutput, approxContoursOutput, cv::COLOR_GRAY2RGB);
    for (size_t i = 0; i < approxContours.size(); i++)
    {
        for (size_t n = 0; n < approxContours[i].size(); n++)
        {
            cv::putText(approxContoursOutput, std::to_string(n), approxContours[i][n], cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 255));
        }
    }
    cv::drawContours(approxContoursOutput, approxContours, -1, cv::Scalar(0, 0, 255), 3);
    cv::imshow("approx contours", approxContoursOutput);

    filterContoursOutput = openOutput.clone();
    cv::cvtColor(filterContoursOutput, filterContoursOutput, cv::COLOR_GRAY2RGB);
    cv::drawContours(filterContoursOutput, filterContours, -1, cv::Scalar(255, 0, 0), 3);
    cv::imshow("filtered contours", filterContoursOutput);
	
	//Pour l'affichage il faut attendre avant d'actualiser
	cv::waitKey(30);
}
