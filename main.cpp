#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <string>
#include <stdio.h>
#include <thread>
#include <chrono>
#include <vector>
#include <fstream>

#include "include/utils/getAngles.hpp"
#include "include/utils/distance.hpp"
#include "include/utils/getBoundedRects.hpp"
#include "include/utils/getCenterOfMass.hpp"
#include "include/utils/getContours.hpp"
#include "include/utils/getCorners.hpp"
#include "include/utils/netThread.hpp"
#include "include/utils/udpClientServer.hpp"
#include "include/utils/mjpgStream.hpp"
#include "include/utils/gui.hpp"
#include "include/utils/enumCvType.hpp"

#include "include/filters/selectMode.hpp"
#include "include/filters/cannyEdgeDetect.hpp"
#include "include/filters/dilateErode.hpp"
#include "include/filters/gaussianBlur.hpp"
#include "include/filters/houghLines.hpp"
#include "include/filters/houghCircles.hpp"
#include "include/filters/hsvColorThreshold.hpp"
#include "include/filters/laplacianSharpen.hpp"
#include "include/filters/mergeFinal.hpp"
#include "include/filters/shapeThresholds.hpp"

#include "include/filters/cannyEdgeDetectWindows.hpp"
#include "include/filters/dilateErodeWindows.hpp"
#include "include/filters/gaussianBlurWindows.hpp"
#include "include/filters/houghLinesWindows.hpp"
#include "include/filters/houghCirclesWindows.hpp"
#include "include/filters/hsvColorThresholdWindows.hpp"
#include "include/filters/laplacianSharpenWindows.hpp"
#include "include/filters/mergeFinalWindows.hpp"
#include "include/filters/shapeThresholdsWindows.hpp"

// Calculating fps
const bool FPS = true;
// Calibrating with windows instead of deployment
const bool CALIB = true;
// Streaming to mjpg-streamer instead of cv::imshow
const bool STREAM = false; 

const int SCREEN_WIDTH = 640;
const int SCREEN_HEIGHT = 480;

// Measurements are in inches
const double TOWER_HEIGHT = 83;     
const double CAMERA_HEIGHT = 6;
const int CALIB_DISTANCE = 186;
// Measurements in inches
const double GAME_ELEMENT_WIDTH = 2;
const double GAME_ELEMENT_HEIGHT = 5;
const int CAMERA_NUM = 0;

double pitch = 0;
double yaw = 0;
double fps = 0;

const std::string FPS_FILE = "logs/fps.log";
const std::string PROC_DATA_FILE = "logs/processed_data.log";

//const std::string TARGET_ADDR = "10.1.15.2";
//const std::string HOST_ADDR = "10.1.15.8";
const std::string TARGET_ADDR = "2.9.9.1";
// const std::string HOST_ADDR = "10.42.0.1";
const std::string HOST_ADDR = "localhost";
const int UDP_PORT = 5810;

cv::Scalar GREEN (0, 255, 0);
cv::Scalar BLUE_GREEN (255, 255, 0);
cv::Scalar PURPLE (255, 0, 255);
cv::Scalar LIGHT_GREEN (255, 100, 100);
cv::Scalar RED (0, 0, 255);
cv::Scalar YELLOW (0, 255, 255);

// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(cv::Mat& R)
{
    cv::Mat Rt;
    transpose(R, Rt);
    cv::Mat shouldBeIdentity = Rt * R;
    cv::Mat I = cv::Mat::eye(3,3, shouldBeIdentity.type());

    return  norm(I, shouldBeIdentity) < 1e-6;
}

float toDeg (float rad)
{
    return rad * 180 / PI;
}

// Calculates rotation matrix to euler angles
// The order is (pitch, yaw, roll)
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
cv::Vec3f rotationMatrixToEulerAngles(cv::Mat& R)
{
    if (!isRotationMatrix(R))
    {
        std::cerr << "NOT A ROTATION MATRIX\n";
        return cv::Vec3f (-1.0, -1.0, -1.0);
    }

    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
    bool singular = sy < 1e-6; // If
    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    return cv::Vec3f(toDeg(x), toDeg(y), toDeg(z));
    // return Vec3f(x, y, z);
}

bool containsNaN(cv::Mat& m)
{
    for (size_t j = 0; j < m.rows; ++j)
    {
        for (size_t k = 0; k < m.cols; ++k)
        {
            if (cvIsNaN(m.at<double>(j, k)) || cvIsInf(m.at<double>(j, k))) return true;
        }
    }
    return false;
}

// Precondition: input image should be grayscale and corners should be a rect
// Return a vector of the angular displacement of the camera from the target
// in degrees from -180 < theta < 180.
// Return (-1.0, -1.0, -1.0) on failure
cv::Vec3f getAngularDisplacement(cv::Mat& img,
        std::vector<cv::Point>& corners,
        std::string& cameraConfigFile,
        float targetWidth, float targetHeight)
{
    if (corners.size() != 4) return cv::Vec3f (-1.0, -1.0, -1.0);

    cv::FileStorage fs;
    fs.open(cameraConfigFile, cv::FileStorage::READ);
    // Read camera matrix and distortion coefficients from file

    cv::Mat intrinsics, distortion;
    fs["Camera_Matrix"] >> intrinsics;
    fs["Distortion_Coefficients"] >> distortion;

    // Close the input file
    fs.release();

    std::vector<cv::Point2f> projectedAxesPoints;
    std::vector<cv::Point3f> targetPoints, axesPoints;

    axesPoints.push_back(cv::Point3f(0.0, 0.0, 0.0));
    axesPoints.push_back(cv::Point3f(5.0, 0.0, 0.0));
    axesPoints.push_back(cv::Point3f(0.0, 5.0, 0.0));
    axesPoints.push_back(cv::Point3f(0.0, 0.0, 5.0));

    // Bottom left, top left, top right, bottom right
    // targetPoints.push_back(cv::Point3f(0.0, 0.0, 0.0));
    // targetPoints.push_back(cv::Point3f(0.0, 6.5, 0.0));
    // targetPoints.push_back(cv::Point3f(11.5, 6.5, 0.0));
    // targetPoints.push_back(cv::Point3f(11.5, 0.0, 0.0));
    //
    targetPoints.push_back(cv::Point3f(-targetWidth / 2, -targetHeight / 2, 0.0));
    targetPoints.push_back(cv::Point3f(-targetWidth / 2,  targetHeight / 2, 0.0));
    targetPoints.push_back(cv::Point3f( targetWidth / 2,  targetHeight / 2, 0.0));
    targetPoints.push_back(cv::Point3f( targetWidth / 2, -targetHeight / 2, 0.0));
    //
    // targetPoints.push_back(cv::Point3f(0.0, 0.0, 0.0));
    // targetPoints.push_back(cv::Point3f(0.0, 1.0, 0.0));
    // targetPoints.push_back(cv::Point3f(1.0, 0.0, 0.0));
    // targetPoints.push_back(cv::Point3f(1.0, 1.0, 0.0));

    // cv::Mat rmat, tmat;

    // Find the 2D pose estimations in vector representations
    // (pitch, yaw, roll) and (x, y, z) of the target
    // std::cerr << corners.size() << ", " << targetPoints.size() << "\n";
    // std::cerr << intrinsics << "\n" << distortion << "\n";

    // Convert the corner points into floats for solvePnP
    // TODO: Change corners to be a vector of cv::Point2f
    std::vector<cv::Point2f> cornerPoints;
    for (int i = 0; i < corners.size(); ++i)
        cornerPoints.push_back(cv::Point2f(corners[i].x, corners[i].y));

    cv::Mat rmat, tmat;

    // std::cerr << cornerPoints << "\n";
    // TODO: Use RANSAC
    solvePnPRansac(targetPoints, cornerPoints, intrinsics, distortion, rmat, tmat, false, CV_EPNP);
    if (containsNaN(rmat) || containsNaN(tmat)) return cv::Vec3f (-1.0, -1.0, -1.0);

    char str[30];
    sprintf(str, "R (%8.2f, %8.2f, %8.2f)", rmat.at<double>(0, 0), rmat.at<double>(1, 0), rmat.at<double>(2, 0));
    cv::putText(img, str, cv::Point(10, 390), CV_FONT_HERSHEY_COMPLEX_SMALL, 0.75, cv::Scalar(255, 0, 200), 1, 8, false);

    // Project the axes onto the image
    cv::projectPoints(axesPoints, rmat, tmat, intrinsics, distortion, projectedAxesPoints);

    // Change the vectors to matrices
    cv::Rodrigues(rmat, rmat);
    // cv::Rodrigues(tmat, tmat);

    cv::Vec3f rotAngles = rotationMatrixToEulerAngles(rmat);

    // Phi is the heading (angle between normal line to camera and line to target)
    float phi = toDeg(atan(tmat.at<double>(0, 0) / tmat.at<double>(2, 0)));
    // float phi = toDeg(cv::fastAtan2(tmat.at<double>(2, 0), tmat.at<double>(0, 0)));

    // Put the angle measurements on the image
    // sprintf(str, "R (%8.2f, %8.2f, %8.2f)", rmat.at<double>(0, 0), rmat.at<double>(1, 0), rmat.at<double>(2, 0));
    // cv::putText(img, str, cv::Point(10, 390), CV_FONT_HERSHEY_COMPLEX_SMALL, 0.75, cv::Scalar(255, 0, 200), 1, 8, false);
    sprintf(str, "R (%8.2f, %8.2f, %8.2f)", rotAngles[0], rotAngles[1], rotAngles[2]);
    cv::putText(img, str, cv::Point(10, 410), CV_FONT_HERSHEY_COMPLEX_SMALL, 0.75, cv::Scalar(255, 0, 200), 1, 8, false);
    sprintf(str, "T (%8.2f, %8.2f, %8.2f)", tmat.at<double>(0, 0), tmat.at<double>(2,0), tmat.at<double>(2, 0));
    cv::putText(img, str, cv::Point(10, 430), CV_FONT_HERSHEY_COMPLEX_SMALL, 0.75, cv::Scalar(255, 0, 200), 1, 8, false);
    sprintf(str, "Phi: %.2f", phi);
    cv::putText(img, str, cv::Point(10, 450), CV_FONT_HERSHEY_COMPLEX_SMALL, 0.75, cv::Scalar(255, 0, 200), 1, 8, false);

    // Theta - Phi is the actual angular displacement
    sprintf(str, "Theta - Phi: %.2f", rotAngles[1] - phi);
    cv::putText(img, str, cv::Point(10, 470), CV_FONT_HERSHEY_COMPLEX_SMALL, 0.75, cv::Scalar(255, 0, 200), 1, 8, false);

    // Draw the axes of rotation at the bottom left corner
    cv::circle(img, projectedAxesPoints[0], 4, cv::Scalar(255, 0, 0), -1);
    cv::line(img, projectedAxesPoints[0], projectedAxesPoints[1], cv::Scalar(0, 0, 255), 2);
    cv::line(img, projectedAxesPoints[0], projectedAxesPoints[2], cv::Scalar(0, 255, 0), 2);
    cv::line(img, projectedAxesPoints[0], projectedAxesPoints[3], cv::Scalar(255, 0, 0), 2);

    rotAngles[1] -= phi;
    return rotAngles;
}

int main (int argc, char *argv[])
{
    // Parameters for selecting which filter windows to see
    int blur = 0;
    int color = 0;
    int dilateErode = 0;
    int edge = 0;
    int laplacian = 0;
    int houghLines = 0;
    int houghCircles = 0;
    int uShapeThresholdWindow = 0;
    int sideRatioThresholdWindow = 0;
    int areaRatioThresholdWindow = 0;
    int angleThresholdWindow = 0;
    int drawStats = 0;
    int merge = 0;

    // Parameters for applying filters even if windows are closed
    //int applyBlur = 1;
    //int applyColor = 1;
    //int applyDilateErode = 0;
    //int applyEdge = 0;
    //int applyLaplacian = 0;
    //int applyHoughLines = 0;
    //int applyHoughCircles = 0;
    //int applyUShapeThreshold = 1;
    //int applySideRatioThreshold = 1;
    //int applyAreaRatioThreshold = 1;
    //int applyAngleThreshold = 1;
    //int applyMerge = 0;
    int applyBlur = 1;
    int applyColor = 1;
    int applyDilateErode = 0;
    int applyEdge = 0;
    int applyLaplacian = 0;
    int applyHoughLines = 0;
    int applyHoughCircles = 0;
    int applyUShapeThreshold = 0;
    int applySideRatioThreshold = 1;
    int applyAreaRatioThreshold = 1;
    int applyAngleThreshold = 1;
    int applyMerge = 1;

    // gaussianBlur parameters
    int blur_ksize = 5;
    int sigmaX = 10;
    int sigmaY = 10;

    // hsvColorThreshold parameters
    int hMin = 0;
    int hMax = 360;
    int sMin = 0;
    int sMax = 35;
    int vMin = 50;
    int vMax = 100;
    int debugMode = 0;
    // 0 is none, 1 is bitAnd between h, s, and v
    int bitAnd = 1;

    // dilateErode parameters
    int holes = 0;
    int noise = 0;
    int size = 1;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS,
                      cv::Size(2 * size + 1, 2 * size + 1), 
                      cv::Point(size, size) );
    
    // cannyEdgeDetect parameters
    int threshLow = 100;
    int threshHigh = 245;
    
    // laplacian paraeters
    int laplacian_ksize = 3;
    int scale = 1;
    int delta = 0;
    
    // houghLines parameters
    int rho = 1;
    int theta = 180;
    int threshold = 50;
    int lineMin = 50;
    int maxGap = 10;        

    // houghCircles parameters
    int hcMinDist = SCREEN_HEIGHT / 8;
    int hcMinRadius = 116;
    int hcMaxRadius = 212;  

    // mergeFinal parameters
    int mergeWeight1 = 20;
    int mergeWeight2 = 100;

    // Shape threshold parameters
    // Params need to be ints for opencv windows
    int sideRatioParam = 270;
    int areaRatioParam = 100;
    int minAreaParam = 800;
    int maxAreaParam = 20000;
    int sideRatioMaxDeviationParam = 80;
    int areaRatioMaxDeviationParam = 90;
    int angleMaxDeviationParam = 20;
    double sideRatio = (double) sideRatioParam / 100;
    double areaRatio = (double) areaRatioParam / 100;
    double minArea = (double) minAreaParam;
    double maxArea = (double) maxAreaParam;
    double sideRatioMaxDeviation = (double) sideRatioMaxDeviationParam / 100;
    double areaRatioMaxDeviation = (double) areaRatioMaxDeviationParam / 100;
    double angleMaxDeviation = (double) angleMaxDeviationParam;

    // uShape
    int minDistFromContours = 5;
    int maxDistFromContours = 15;

    // Tower height is 7 feet (to the bottom of the tape) which is 84 inches

    int contoursThresh = 140;
    std::vector< std::vector<cv::Point> > contours;
    std::vector<cv::RotatedRect> boundedRects;
    cv::Point2f rectPoints[4];
    int goalInd = 0;
    // Game specific elements
    int height = TOWER_HEIGHT - CAMERA_HEIGHT;

    // Distance calculation
    int isFocalLengthCalib = 0;
    double focalLength = 600;

    // Angular displacement parameters
    cv::Vec3f angleVec;
    std::string cameraConfigFile = "logs/out_camera_data.xml";

    // Linearize contours parameters
    int approximationAccuracy = 3;

    // Corner parameters
    int blockSize = 2;
    int apertureSize = 7;
    int k = 1;
    int isHarris = 0;
    int maxCorners = 10;
    int minQualityRatio = 80;
    int minDist = 10;

    udp_client_server::udp_client client(TARGET_ADDR, UDP_PORT);
    udp_client_server::udp_server server(HOST_ADDR, UDP_PORT);

    std::thread netSend (sendData, std::ref(client));
    netSend.detach();
    std::thread netReceive (receiveData, std::ref(server));
    netReceive.detach();

    cv::VideoCapture camera (CAMERA_NUM);
    if (!camera.isOpened())
        throw std::runtime_error(std::string("Error - Could not open camera at port ") + std::to_string(CAMERA_NUM));
    else
        std::cerr << "Opened camera at port " << CAMERA_NUM << "\n";

    std::cerr << "\n";
    std::cerr << " ============== NOTICE ============= " << "\n";
    std::cerr << "|                                   |" << "\n";
    std::cerr << "| Press 'q' to quit without saving  |" << "\n";
    std::cerr << "| Press 's' to save parameters      |" << "\n";
    std::cerr << "| Press ' ' to pause                |" << "\n";
    std::cerr << "|                                   |" << "\n";
    std::cerr << " =================================== " << "\n";
    std::cerr << "\n";

    cv::Mat img, rgb;
    char kill = ' ';

    std::chrono::high_resolution_clock::time_point start, end;

    // gnuplot parameters
    // FILE *pipe_gp = popen("gnuplot", "w");  
    // fputs("set terminal png\n", pipe_gp);
    // fputs("plot '-' u 1:2 \n", pipe_gp);
    //
    // FILE *fpsFile = fopen(FPS_FILE.c_str(), "a");
    // FILE *dataFile = fopen(PROC_DATA_FILE.c_str(), "a");

    std::ofstream fpsFile, dataFile;
    fpsFile.open(FPS_FILE.c_str(), std::ios::out | std::ios::app);
    dataFile.open(PROC_DATA_FILE.c_str(), std::ios::out | std::ios::app);

    double avg = 0;
    double fpsTick = 1;

    while (kill != 'q')
    {
        // Press space to pause program, then any key to resume
        if (kill == ' ')
            cv::waitKey(0);

        // Press s to save values into FileStorage
        if (kill == 's')
        {
            cv::FileStorage fs("logs/config.yml", cv::FileStorage::WRITE);

            fs << "Parameters" << "{"

                << "Is Blur Window Open" << blur
                << "Is Color Window Open" << color
                << "Is Dilate Erode Window Open" << dilateErode
                << "Is Edge Window Open" << edge
                << "Is Laplacian Window Open" << laplacian
                << "Is HoughLines Window Open" << houghLines
                << "Is HoughCircles Window Open" << houghCircles
                << "Is uShapeThreshold Window Open" << uShapeThresholdWindow
                << "Is sideRatioThreshold Window Open" << sideRatioThresholdWindow
                << "Is areaRatioThreshold Window Open" << areaRatioThresholdWindow
                << "Is angleThreshold Window Open" << angleThresholdWindow
                << "Is drawStats Open" << drawStats
                << "Is merge Open" << merge

                << "Apply Blur" << applyBlur
                << "Apply Color" << applyColor
                << "Apply DilateErode" << applyDilateErode
                << "Apply Edge" << applyEdge
                << "Apply Laplacian" << applyLaplacian
                << "Apply HoughLines" << applyHoughLines
                << "Apply HoughCircles" << applyHoughCircles
                << "Apply UShapeRatioThreshold" << applyUShapeThreshold
                << "Apply SideRatioThreshold" << applySideRatioThreshold
                << "Apply AreaRatioThreshold" << applyAreaRatioThreshold
                << "Apply AngleThreshold" << applyAngleThreshold
                << "Apply Merge" << applyMerge

                << "Gaussian Blur Kernel Size" << blur_ksize
                << "Guassian Blur Sigma X" << sigmaX
                << "Guassian Blur Sigma Y" << sigmaY

                << "Hue Minimum Threshold" << hMin
                << "Hue Maximum Threshold" << hMax
                << "Saturation Minimum Threshold" << sMin
                << "Saturation Maximum Threshold" << sMax
                << "Value Minimum Threshold" << vMin
                << "Value Maximum Threshold" << vMax

                << "Dilate Erode Holes" << holes
                << "Dilate Erode Noise" << noise
                << "Dilate Erode Size" << size

                << "Canny Lower Threshold" << threshLow
                << "Canny Higher Threshold" << threshHigh

                << "Laplacian Kernel Size" << laplacian_ksize
                << "Laplacian Scale" << scale
                << "Laplacian Delta" << delta

                << "HoughLines Rho" << rho
                << "HoughLines Theta" << theta
                << "HoughLines Threshold" <<  threshold
                << "HoughLines LineMin" << lineMin
                << "HoughLines MaxGap" <<  maxGap

                << "HoughCircles Minimum Distance" << hcMinDist
                << "HoughCircles Minimum Radius" << hcMinRadius
                << "HoughCircles Maximum Radius" << hcMaxRadius

                << "Merge Weight 1" << mergeWeight1
                << "Merge Weight 2" <<  mergeWeight2

                << "Side Ratio Parameter" << sideRatioParam
                << "Area Ratio Parameter" << areaRatioParam
                << "Minimum Area Parameter" << minAreaParam
                << "Maximum Area Parameter" << maxAreaParam
                << "Side Ratio Maximum Deviation Parameter" << sideRatioMaxDeviationParam
                << "Area Ratio Maximum Deviation Parameter" << areaRatioMaxDeviationParam
                << "Angle Max Deviation Parameter" << angleMaxDeviationParam

             << "}";
            fs.release();
        }

        if (FPS) start = std::chrono::high_resolution_clock::now();
        if (argc > 2)
        {
            // TODO: acquire input through imgs / videos
        }
        else
        {
            camera >> img;
            rgb = img.clone();
        }
        //if (!STREAM) cv::imshow("RGB", img);
        if (CALIB && STREAM)
        {
            selectMode(blur, color, dilateErode, edge, laplacian, houghLines, houghCircles, uShapeThresholdWindow, sideRatioThresholdWindow, areaRatioThresholdWindow, angleThresholdWindow, drawStats, merge);
            // MJPG streamer will stream the image after a filter if the filter's window is visible
            // This is currently a messy way of doing this, but it'll work for now
            // Note that only one window can be visible at a time, or else the stream will flicker
            gaussianBlurWindows(img, blur_ksize, sigmaX, sigmaY, applyBlur, blur, STREAM);
            if (blur) mjpgStream(img);

            hsvColorThresholdWindows(img, hMin, hMax, sMin, sMax, vMin, vMax, debugMode, bitAnd, applyColor, color, STREAM);
            if (color) mjpgStream(img);

            dilateErodeWindows(img, element, holes, noise, applyDilateErode, dilateErode, STREAM);
            if (dilateErode) mjpgStream(img);

            cannyEdgeDetectWindows(img, threshLow, threshHigh, applyEdge, edge, STREAM);
            if (edge) mjpgStream(img);

            laplacianSharpenWindows(img, laplacian_ksize, scale, delta, applyLaplacian, laplacian, STREAM);
            if (laplacian) mjpgStream(img);

            houghLinesWindows(img, rho, theta, threshold, lineMin, maxGap, applyHoughLines, houghLines, STREAM);
            if (houghLines) mjpgStream(img);

            houghCirclesWindows(img, hcMinDist, hcMinRadius, hcMaxRadius, applyHoughCircles, houghCircles, STREAM);
            if (houghCircles) mjpgStream(img);
        }
        else if (CALIB)
        {
            selectMode(blur, color, dilateErode, edge, laplacian, houghLines, houghCircles, uShapeThresholdWindow, sideRatioThresholdWindow, areaRatioThresholdWindow, angleThresholdWindow, drawStats, merge);
            gaussianBlurWindows(img, blur_ksize, sigmaX, sigmaY, applyBlur, blur, STREAM);
            hsvColorThresholdWindows(img, hMin, hMax, sMin, sMax, vMin, vMax, debugMode, bitAnd, applyColor, color, STREAM);
            dilateErodeWindows(img, element, holes, noise, applyDilateErode, dilateErode, STREAM);
            cannyEdgeDetectWindows(img, threshLow, threshHigh, applyEdge, edge, STREAM);
            laplacianSharpenWindows(img, laplacian_ksize, scale, delta, applyLaplacian, laplacian, STREAM);
            houghLinesWindows(img, rho, theta, threshold, lineMin, maxGap, applyHoughLines, houghLines, STREAM);
            houghCirclesWindows(img, hcMinDist, hcMinRadius, hcMaxRadius, applyHoughCircles, houghCircles, STREAM);

            // cv::imwrite("images/snapped_images/filtered.jpg", img);

        }
        else
        {
            gaussianBlur(img, blur_ksize, sigmaX, sigmaY);
            hsvColorThreshold(img, hMin, hMax, sMin, sMax, vMin, vMax, debugMode, bitAnd);
        }

        try
        {
            contours = getContours(img, contoursThresh);
        }
        catch (std::exception& e)
        {
            std::cerr << e.what() << "\n";
        }


        // if (contours.size() > 0)
        // {
        //     std::vector< std::vector<cv::Point> > linearContours;
        //     linearContours.resize(contours.size());
        //     for (size_t i = 0; i < contours.size(); ++i)
        //         cv::approxPolyDP(contours[i], contours[i], approximationAccuracy, true);
        //     cv::Mat imgBlank;
        //     imgBlank = rgb.clone();
        //     drawContours(imgBlank, contours, YELLOW);
        //     cv::imshow("Linear Contours", imgBlank);
        // }

        if (contours.size() > 0)
        {
            boundedRects = getBoundedRects(contours);

            if (CALIB)
            {
                uShapeThresholdWindows(img, contours, boundedRects, minDistFromContours, maxDistFromContours, applyUShapeThreshold, uShapeThresholdWindow, STREAM);
                sideRatioThresholdWindows(img, contours, boundedRects, sideRatioParam, sideRatioMaxDeviationParam, applySideRatioThreshold, sideRatioThresholdWindow, STREAM);
                areaRatioThresholdWindows(img, contours, boundedRects, minAreaParam, maxAreaParam, areaRatioParam, areaRatioMaxDeviationParam, applyAreaRatioThreshold, areaRatioThresholdWindow, STREAM);
                angleThresholdWindows(img, contours, boundedRects, angleMaxDeviationParam, applyAngleThreshold, angleThresholdWindow, STREAM);
            }
            else
            {
                // Filter out contours that do not match the 'U' shape
                uShapeThreshold(contours, boundedRects, minDistFromContours, maxDistFromContours);
                sideRatioThreshold(contours, boundedRects, sideRatio, sideRatioMaxDeviation);
                areaRatioThreshold(contours, boundedRects, minArea, maxArea, areaRatio, areaRatioMaxDeviation);
                angleThreshold(contours, boundedRects, angleMaxDeviation);
            }

            // Extract the corners of the target's bounded box
            // TODO: Write an algorithm that decides which index to use as goalInd
            boundedRects[goalInd].points(rectPoints);

            // Temporary conversion of Point2f to Point
            cv::Point rectIntPoints[4];
            int vertices = 4;
            for (int i = 0; i < vertices; ++i)
                rectIntPoints[i] = cv::Point (rectPoints[i].x, rectPoints[i].y);

            cv::Mat mask = cv::Mat::zeros(img.size(), CV_8UC1);
            // Draw a filled in bounded rectangle at the goal
            cv::fillConvexPoly(mask, rectIntPoints, vertices, PURPLE);
            cv::imshow("mask", mask);

            // cv::namedWindow("Linearize Contours");
            // cv::createTrackbar("Epsilon", "Linearize Contours", &approximationAccuracy, 100);
            //
            // std::vector< std::vector<cv::Point> > linearContours;
            // linearContours.resize(contours.size());
            // for (size_t i = 0; i < contours.size(); ++i)
            //     cv::approxPolyDP(contours[i], linearContours[i], (double)approximationAccuracy, true);
            // cv::Mat imgBlank;
            // imgBlank = rgb.clone();
            // drawContours(imgBlank, linearContours, YELLOW);
            // cv::imshow("Linear Contours", imgBlank);
            //

            cv::namedWindow("Corners");
            cv::createTrackbar("Block Size", "Corners", &blockSize, 10);
            cv::createTrackbar("Aperture Size", "Corners", &apertureSize, 10);
            cv::createTrackbar("maxCorners", "Corners", &maxCorners, 20);
            cv::createTrackbar("minQualityRatio", "Corners", &minQualityRatio, 100);
            cv::createTrackbar("minDist", "Corners", &minDist, 100);
            cv::createTrackbar("Shi-Tomasi/Harris(Good)/Harris", "Corners", &isHarris, 2);
            cv::createTrackbar("k", "Corners", &k, 10);
            cv::Mat img_gray, imgBlank;
            std::vector<cv::Point> goodCorners;
            cv::cvtColor(img, img_gray, CV_BGR2GRAY);
            imgBlank = img.clone();
            // Find only the region of the grayscale image inside the bounding box
            cv::Mat rectROI;
            img_gray.copyTo(rectROI, mask);
            // cv::imshow("Region of Interest", rectROI);
            if (apertureSize % 2 == 0) apertureSize++;
            if (maxCorners == 0) maxCorners++;
            if (minQualityRatio == 0) minQualityRatio++;
            if (minDist == 0) minDist++;
            if (k == 0) k++;
            if (isHarris == 2)
                cv::cornerHarris(rectROI, imgBlank, blockSize, apertureSize, (double)k/100, cv::BORDER_DEFAULT);
            else
                cv::goodFeaturesToTrack(rectROI, goodCorners, maxCorners, (double)minQualityRatio/100, minDist, cv::Mat(), 3, isHarris, (double)k/100);
            // Draw in filled circles of radius 3 at the identified corners
            for (int i = 0; i < goodCorners.size(); ++i)
            {
                cv::circle(imgBlank, goodCorners[i], 3, YELLOW, -1);
            }

            cv::imshow("Corners", imgBlank);
            // Find the corners of the target's contours
            std::vector<cv::Point> corners = getCorners(contours[goalInd], SCREEN_WIDTH, SCREEN_HEIGHT);
            // if (corners.size() > 0)
            //     cv::cornerSubPix(img, corners, cv::Size(3,3), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::COUNT, 10, 100));
            angleVec = getAngularDisplacement(img, corners, cameraConfigFile, GAME_ELEMENT_WIDTH, GAME_ELEMENT_HEIGHT);
            cv::Point2f mc = getCenterOfMass(contours[goalInd]);

            // Get the distance in inches and angles in degrees
            double hypotenuse = getShortestDistance(rectPoints, GAME_ELEMENT_WIDTH, focalLength, CALIB_DISTANCE, isFocalLengthCalib);
            yaw = getYaw(SCREEN_WIDTH, hypotenuse, GAME_ELEMENT_WIDTH, corners, mc);
            // pitch = getPitch(height, hypotenuse);

            if (drawStats) putData(img, CALIB_DISTANCE, yaw, pitch);

            if (!std::isfinite(yaw)) yaw = -1;
            // if (!std::isfinite(pitch)) pitch = -1;
            if (!std::isfinite(hypotenuse)) hypotenuse = -1;
            if (!std::isfinite(angleVec[1])) angleVec[1] = -1;

            std::string gnuplotBuf = std::to_string(yaw) 
                // + " " + std::to_string(pitch) 
                + " " + std::to_string(hypotenuse) 
                + " " + std::to_string(angleVec[1]);
        
            // fputs(gnuplotBuf.c_str(), dataFile);
            dataFile << gnuplotBuf.c_str() << std::endl;
            drawContours(img, contours, RED);
            drawBoundedRects(img, boundedRects, PURPLE);

            cv::line(img, cv::Point(SCREEN_WIDTH / 2, mc.y), mc, YELLOW); // Draw line from center of img to center of mass
            cv::circle(img, mc, 5, YELLOW); // Draw center of mass

            // Draw the corners of the target's contour
            char str[30];
            for (int i = 0; i < 4; ++i)
            {
                cv::circle(img, corners[i], 3, LIGHT_GREEN);
                sprintf(str, "%d", i+1);
                cv::putText(img, str, corners[i], CV_FONT_HERSHEY_COMPLEX_SMALL, 0.75, BLUE_GREEN, 1, 8, false);
            }
        }

        if (CALIB)
        {
            if (uShapeThresholdWindow || sideRatioThresholdWindow || areaRatioThresholdWindow || angleThresholdWindow) 
            {
                if (STREAM) mjpgStream(img);
                else cv::imshow("Threshold Output", img);
            }
            else if (!STREAM) cv::destroyWindow("Threshold Output");
            mergeFinalWindows(rgb, img, mergeWeight1, mergeWeight2, applyMerge, merge, STREAM);
            if (STREAM && !blur && !color && !dilateErode && !edge && !laplacian && !houghLines && !houghCircles && !uShapeThresholdWindow && !sideRatioThresholdWindow && !areaRatioThresholdWindow && !angleThresholdWindow && !drawStats)
                mjpgStream(img);
            if (!STREAM) cv::imshow("Final Output", img);
        }
        else if (STREAM)
        {
            // Stream the final merged image
            mergeFinal(rgb, img, mergeWeight1, mergeWeight2);
            mjpgStream(img);
        }

        if (FPS)
        {
            end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> timeElapsed = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
            fps = 1.0 / timeElapsed.count();

            // fprintf(pipe_gp, "%f %f\n", fpsTick, fps);
            std::string gnuplotBuf = std::to_string(fps);
            
            // fputs(gnuplotBuf.c_str(), fpsFile);
            fpsFile << gnuplotBuf.c_str() << std::endl;
            // std::cerr << "Logged: " << gnuplotBuf.c_str() << "\n";
            avg += fps;
            fpsTick++;
        }
        
        kill = cv:: waitKey(5);
    }
    return 0;   
}
