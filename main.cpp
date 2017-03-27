#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <string>
#include <stdio.h>
#include <thread>
#include <chrono>
#include <vector>
#include <fstream>

#include "y2017/common/constants.hpp"

#include "y2017/utils/getAngles.hpp"
#include "y2017/utils/distance.hpp"
#include "y2017/utils/getBoundedRects.hpp"
#include "y2017/utils/getCenterOfMass.hpp"
#include "y2017/utils/getContours.hpp"
#include "y2017/utils/getCorners.hpp"
#include "y2017/utils/netThread.hpp"
#include "y2017/utils/udpClientServer.hpp"
#include "y2017/utils/mjpgStream.hpp"
#include "y2017/utils/gui.hpp"
#include "y2017/utils/enumCvType.hpp"

#include "y2017/filters/selectMode.hpp"
#include "y2017/filters/cannyEdgeDetect.hpp"
#include "y2017/filters/dilateErode.hpp"
#include "y2017/filters/gaussianBlur.hpp"
#include "y2017/filters/houghLines.hpp"
#include "y2017/filters/houghCircles.hpp"
#include "y2017/filters/hsvColorThreshold.hpp"
#include "y2017/filters/laplacianSharpen.hpp"
#include "y2017/filters/mergeFinal.hpp"
#include "y2017/filters/shapeThresholds.hpp"

#include "y2017/filters/cannyEdgeDetectWindows.hpp"
#include "y2017/filters/dilateErodeWindows.hpp"
#include "y2017/filters/gaussianBlurWindows.hpp"
#include "y2017/filters/houghLinesWindows.hpp"
#include "y2017/filters/houghCirclesWindows.hpp"
#include "y2017/filters/hsvColorThresholdWindows.hpp"
#include "y2017/filters/laplacianSharpenWindows.hpp"
#include "y2017/filters/mergeFinalWindows.hpp"
#include "y2017/filters/shapeThresholdsWindows.hpp"

// #include "utilities/accurate_corners.cpp"

cv::RNG rng_(12345);

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
    return rad * 180 / M_PI;
}

// Calculates rotation matrix to euler angles
// The order is (pitch, yaw, roll)
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
cv::Vec3f rotationMatrixToEulerAngles(cv::Mat& R)
{
    if (!isRotationMatrix(R))
    {
        std::cout << "NOT A ROTATION MATRIX\n";
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
    for (int j = 0; j < m.rows; ++j)
    {
        for (int k = 0; k < m.cols; ++k)
        {
            if (cvIsNaN(m.at<double>(j, k)) || cvIsInf(m.at<double>(j, k))) return true;
        }
    }
    return false;
}

// The order is (pitch, yaw, roll)
void getEulerAngles(cv::Mat &rotCameraMat,cv::Vec3d& eulerAngles)
{
    cv::Mat cameraMat,rotMat,transVect,rotMatX,rotMatY,rotMatZ;
    double* _r = rotCameraMat.ptr<double>();
    double projMat[12] = {_r[0],_r[1],_r[2],0,
                          _r[3],_r[4],_r[5],0,
                          _r[6],_r[7],_r[8],0};

    cv::decomposeProjectionMatrix(cv::Mat(3,4,CV_64FC1, projMat),
                               cameraMat,
                               rotMat,
                               transVect,
                               rotMatX,
                               rotMatY,
                               rotMatZ,
                               eulerAngles);
}

int getLeftMostContourIndex(std::vector< std::vector<cv::Point> >& contours)
{
    double minDistance = SCREEN_WIDTH, ind = 0, curr = 0;
    cv::Point leftMost = cv::Point(0, SCREEN_HEIGHT/2);
    // cv::Mat img = cv::Mat::zeros(SCREEN_HEIGHT, SCREEN_WIDTH, CV_8UC3);
    for (size_t i = 0; i < contours.size(); ++i)
    {
        cv::Point mc = getCenterOfMass(contours[i]);
        curr = distance(leftMost, mc);
        // char str[100];
        // sprintf(str, "%d: (%3.0f)", i, curr);
        // cv::putText(img, str, mc, CV_FONT_HERSHEY_COMPLEX_SMALL, 0.75, cv::Scalar(255, 0, 200), 1, 8, false);
        if (curr < minDistance)
        {
            minDistance = curr;
            ind = i;
        }
    }
    // cv::imshow("Distances", img);
    return ind;
}

template <typename T>
cv::Point cvtPoint(const T& src)
{
    cv::Point dest = src;
    return dest;
}

// Converts a vector of vectors of cv::Point_<T> to cv::Point_<int>
template <typename T>
void cvtPoint(const std::vector< std::vector<T> >& src, std::vector< std::vector<cv::Point> >& dest)
{
    for (size_t j = 0; j < src.size(); ++j)
        for (size_t k = 0; k < src.size(); ++k)
            dest[j].push_back(src[k]);
}

// template <typename T>
// std::vector < std::vector<cv::Point2f> > cvtPoint2f(const std::vector< std::vector<T> >& src)
// {
//     // std::copy(src.begin(), src.end(),
//     //         [&dest](const std::vector<cv::Point2f> &pt) {
//     //         dest.push_back(std::vector<cv::Point2f>(pt.x, pt.y)); });
//     std::vector< std::vector<cv::Point2f> > dest (src.begin(), src.end());
//     return dest;
// }

// Converts a vector of cv::Point_<int> to cv::Point_<T>
template <typename T>
void cvPointTo(const std::vector<cv::Point>& src, std::vector<T>& dest)
{
    for (size_t i = 0; i < src.size(); ++i)
        dest.push_back(src[i]);
}

template <typename T1, typename T2, size_t N>
void cvtPoint(T1 (&src)[N], T2 (&dest)[N])
{
    for (size_t i = 0; i < N; ++i)
        dest[i] = src[i];
}

int solvePnPFlag = 0;
int useExtrinsicGuess = 0;
int useRansac = 0;

// Precondition: input image should be grayscale and corners should be a rect
// Return a vector of the angular displacement of the camera from the target
// in degrees from -180 < theta < 180.
// Return (-1.0, -1.0, -1.0) on failure
cv::Vec3f getAngularPosition(cv::Mat& img,
        cv::Mat& rvec, cv::Mat& tvec,
        std::vector<cv::Point2f>& corners,
        const std::string& CAMERA_CONFIG_FILE,
        float targetWidth, float targetHeight)
{
    if (corners.size() != 4) return cv::Vec3f (-1.0, -1.0, -1.0);

    cv::FileStorage fs;
    fs.open(CAMERA_CONFIG_FILE, cv::FileStorage::READ);
    // Read camera matrix and distortion coefficients from file

    cv::Mat intrinsics, distortion;
    fs["Camera_Matrix"] >> intrinsics;
    fs["Distortion_Coefficients"] >> distortion;

    // Close the input file
    fs.release();

    std::vector<cv::Point2f> projectedAxesPoints;
    std::vector<cv::Point3f> targetPoints, axesPoints;

    axesPoints.push_back(cv::Point3f(0.0, 0.0, 0.0));
    axesPoints.push_back(cv::Point3f(3.0, 0.0, 0.0));
    axesPoints.push_back(cv::Point3f(0.0, 3.0, 0.0));
    axesPoints.push_back(cv::Point3f(0.0, 0.0, 3.0));

    // Bottom left, top left, top right, bottom right
    targetPoints.push_back(cv::Point3f(-targetWidth / 2, -targetHeight / 2, 0.0));
    targetPoints.push_back(cv::Point3f(-targetWidth / 2,  targetHeight / 2, 0.0));
    targetPoints.push_back(cv::Point3f( targetWidth / 2,  targetHeight / 2, 0.0));
    targetPoints.push_back(cv::Point3f( targetWidth / 2, -targetHeight / 2, 0.0));

    cv::namedWindow("SolvePnP");
    cv::createTrackbar("Normal/RANSAC", "SolvePnP", &useRansac, 1);
    cv::createTrackbar("Use Extrinsic Guess", "SolvePnP", &useExtrinsicGuess, 1);
    cv::createTrackbar("ITERATIVE/P3P/EPNP", "SolvePnP", &solvePnPFlag, 2);
    int flag = 0;
    switch (solvePnPFlag)
    {
    case 0: flag = CV_ITERATIVE; break;
    case 1: flag = CV_P3P; break;
    case 2: flag = CV_EPNP; break;
    }

    // std::cout << intrinsics << "\n" << distortion << "\n";
    // std::cout << "Corners size: " << corners.size() << "\n";
    // std::cout << rvec.size() << ", " << tvec.size() << "\n";
    // Find the 2D pose estimations in vector representations
    // (pitch, yaw, roll) and (x, y, z) of the target
    // std::vector<cv::Point2f> floatCorners;
    // cvPointTo<cv::Point2f>(corners, floatCorners);
    if (useRansac)
        solvePnPRansac(targetPoints, corners, intrinsics, distortion, rvec, tvec, useExtrinsicGuess, 100, flag);
    else
        solvePnP(targetPoints, corners, intrinsics, distortion, rvec, tvec, useExtrinsicGuess, flag);

    if (containsNaN(rvec) || containsNaN(tvec)) 
    {
        std::cout << "NaN in rvec or tvec" << std::endl; 
        return cv::Vec3f (-1.0, -1.0, -1.0);
    }

    // Project the axes onto the image
    cv::projectPoints(axesPoints, rvec, tvec, intrinsics, distortion, projectedAxesPoints);

    cv::Mat rmat, tmat;
    // Change the vectors to matrices
    cv::Rodrigues(rvec, rmat);

    // cv::Mat camRotVec;
    // cv::Rodrigues(rmat.t(), camRotVec);
    cv::Mat camTransVec = -rmat.t() * tvec;
    double r = std::sqrt(std::pow(camTransVec.at<double>(0, 0), 2) + std::pow(camTransVec.at<double>(0, 1), 2) + std::pow(camTransVec.at<double>(0, 2), 2));

    cv::Vec3f rotAngles = rotationMatrixToEulerAngles(rmat);
    // cv::Vec3d eulerAngles;
    // getEulerAngles(rmat, eulerAngles);

    // Beta is the heading (angle between normal line to camera and line to target)
    // // Beta is negative because counter clockwise is positive
    // double x = tvec.at<double>(0, 0);
    // double z = tvec.at<double>(2, 0);
    // float beta = toDeg(atan(x/z));
    //
    // float phi = toDeg(asin(rmat.at<double>(0, 2)));
    // float theta = -beta - phi; // -beta - phi is the actual angular displacement
    double x = camTransVec.at<double>(0, 0);
    double z = camTransVec.at<double>(0, 2);
    float theta = toDeg(atan(x/z));
    // std::cout << beta + rotAngles[1] << "\n";

    // Put the angle measurements on the image
    cv::Mat statsImg = cv::Mat::zeros(img.size(), CV_8UC3);
    char str [60];
    sprintf(str, "Euclidean Distance (%8.2f)", r);
    cv::putText(statsImg, str, cv::Point(10, 20), CV_FONT_HERSHEY_COMPLEX_SMALL, 0.75, cv::Scalar(255, 0, 200), 1, 8, false);
    sprintf(str, "Camera Transpose (%8.2f, %8.2f, %8.2f)", camTransVec.at<double>(0, 0), camTransVec.at<double>(0, 1), camTransVec.at<double>(0, 2));
    cv::putText(statsImg, str, cv::Point(10, 320), CV_FONT_HERSHEY_COMPLEX_SMALL, 0.75, cv::Scalar(255, 0, 200), 1, 8, false);
    // sprintf(str, "Eu (%8.2f, %8.2f, %8.2f)", eulerAngles[0], eulerAngles[1], eulerAngles[2]);
    // cv::putText(statsImg, str, cv::Point(10, 320), CV_FONT_HERSHEY_COMPLEX_SMALL, 0.75, cv::Scalar(255, 0, 200), 1, 8, false);
    sprintf(str, "RM (%8.2f, %8.2f, %8.2f)", rmat.at<double>(0, 0), rmat.at<double>(0, 1), rmat.at<double>(0, 2));
    cv::putText(statsImg, str, cv::Point(10, 340), CV_FONT_HERSHEY_COMPLEX_SMALL, 0.75, cv::Scalar(255, 0, 200), 1, 8, false);
    sprintf(str, "RV (%8.2f, %8.2f, %8.2f)", rotAngles[0], rotAngles[1], rotAngles[2]);
    cv::putText(statsImg, str, cv::Point(10, 360), CV_FONT_HERSHEY_COMPLEX_SMALL, 0.75, cv::Scalar(255, 0, 200), 1, 8, false);
    sprintf(str, "TV (%8.2f, %8.2f, %8.2f)", tvec.at<double>(0, 0), tvec.at<double>(1,0), tvec.at<double>(2, 0));
    cv::putText(statsImg, str, cv::Point(10, 380), CV_FONT_HERSHEY_COMPLEX_SMALL, 0.75, cv::Scalar(255, 0, 200), 1, 8, false);

    // sprintf(str, "Rot Theta: %.2f", -beta + rotAngles[1]);
    // cv::putText(statsImg, str, cv::Point(10, 410), CV_FONT_HERSHEY_COMPLEX_SMALL, 0.75, cv::Scalar(255, 0, 200), 1, 8, false);
    // sprintf(str, "Phi: %.2f", phi);
    // cv::putText(statsImg, str, cv::Point(10, 430), CV_FONT_HERSHEY_COMPLEX_SMALL, 0.75, cv::Scalar(255, 0, 200), 1, 8, false);
    // sprintf(str, "Beta: %.2f", beta);
    // cv::putText(statsImg, str, cv::Point(10, 450), CV_FONT_HERSHEY_COMPLEX_SMALL, 0.75, cv::Scalar(255, 0, 200), 1, 8, false);
    sprintf(str, "Theta: %.2f", theta);
    cv::putText(statsImg, str, cv::Point(10, 470), CV_FONT_HERSHEY_COMPLEX_SMALL, 0.75, cv::Scalar(255, 0, 200), 1, 8, false);

    cv::imshow("Stats", statsImg);
    // Draw the axes of rotation at the bottom left corner
    cv::circle(img, projectedAxesPoints[0], 4, cv::Scalar(255, 0, 0), -1);
    cv::line(img, projectedAxesPoints[0], projectedAxesPoints[1], cv::Scalar(0, 0, 255), 2);
    cv::line(img, projectedAxesPoints[0], projectedAxesPoints[2], cv::Scalar(0, 255, 0), 2);
    cv::line(img, projectedAxesPoints[0], projectedAxesPoints[3], cv::Scalar(255, 0, 0), 2);

    // rotAngles[1] -= beta;
    // Temporary replacement for changing the return type
    // rotAngles[0] = phi;
    // rotAngles[1] = beta;
    rotAngles[0] = -x; // Negative because counter clockwise is positive by pose
    rotAngles[1] = z;
    rotAngles[2] = theta;
    // std::cout << rotAngles << std::endl;
    return rotAngles;
}

double getFocalLenMM()
{
    cv::FileStorage fs;
    fs.open(CAMERA_CONFIG_FILE, cv::FileStorage::READ);
    // Read camera matrix and distortion coefficients from file

    cv::Mat intrinsics, distortion;
    fs["Camera_Matrix"] >> intrinsics;

    // Close the input file
    fs.release();

    std::cout << "X0: " << intrinsics.at<double>(0, 2) << "\n";

    // Average focal length x (f_x) and focal length y (f_y)
    return (intrinsics.at<double>(0, 0) + intrinsics.at<double>(1, 1)) / 2.0;
}

double getScalePixToMM()
{
    // Intrinsic width of Logitech C920 sensor in mm
    return 6.22 / SCREEN_WIDTH;
}

// Converts inches to pixels based on the intrinsic sensor width
double inchesToPixels(double in)
{
    // Inches -> MM -> Pixels
    return in * INCHES_TO_MM / getScalePixToMM();
}

// Returns the angular position of the center of mass relative to the perspective line
// of the camera, in degrees
// All distances must be in inches
double getYawToCenterOfMass(cv::Point& mc, double euclidDist)
{
    double distInPixels = (mc.x - SCREEN_WIDTH/2);

    // Numerator is pixels -> mm -> inches
    return std::asin((distInPixels * getScalePixToMM() / INCHES_TO_MM)/ euclidDist) * 180 / M_PI;
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
    int distanceCalib = 0;
    int merge = 0;

    // Parameters for applying filters even if windows are closed
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
    int showDistanceCalib = 1;
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

    int contoursThresh = 140;
    std::vector< std::vector<cv::Point> > contours;
    std::vector<cv::RotatedRect> boundedRects;
    cv::Point2f rectPoints[4];
    int goalInd = 0;

    double focalLength = 600; // Intrinsic property
    int applyDistanceCalib = 0;
    // Measurements in inches
    int heightDisplacement = TOWER_HEIGHT - CAMERA_HEIGHT;
    int calibDistance = 24;

    // Angular displacement parameters
    // cv::Vec3f angleVec;
    cv::Mat rvec, tvec;

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
    CornerExtractor::CornerParams cornerParams = DEFAULT_CORNER_PARAMS;
    CornerExtractor gamePiece (cornerParams);

    // Gnuplot parameters
    std::ofstream fpsFile, dataFile;
    fpsFile.open(FPS_FILE.c_str(), std::ios::out);
    dataFile.open(PROC_DATA_FILE.c_str(), std::ios::out);

    double avg = 0;
    double fpsTick = 1;

    int lineThickness = 1;

    // udp_client_server::udp_client client(TARGET_ADDR, UDP_PORT);
    // udp_client_server::udp_server server(HOST_ADDR, UDP_PORT);

    // std::thread netSend (sendData, std::ref(client));
    // netSend.detach();
    // std::thread netReceive (receiveData, std::ref(server));
    // netReceive.detach();

    // No video passed in
    cv::VideoCapture camera;
    if (argc == 1)
    {
        std::cout << "Using camera\n";
        camera = cv::VideoCapture (CAMERA_NUM);
        camera.set(CV_CAP_PROP_EXPOSURE, 0.01);
    }
    else
    {
        std::cout << "Opening video from " << argv[1] << "\n";
        camera = cv::VideoCapture (argv[1]);
    }
    if (!camera.isOpened())
        throw std::runtime_error(std::string("Error - Could not open camera at port ") + std::to_string(CAMERA_NUM));

    std::cout << "Opened camera at port " << CAMERA_NUM << "\n";
    std::cout << "Camera exposure: " << camera.get(CV_CAP_PROP_EXPOSURE) << std::endl;
    std::cout << "Camera fps: " << camera.get(CV_CAP_PROP_FPS) << std::endl;
    std::cout << "Camera frame count: " << camera.get(CV_CAP_PROP_FRAME_COUNT) << std::endl;

    int frameInd = 0;
    int totalFrames = camera.get(CV_CAP_PROP_FRAME_COUNT);

    // if (argc == 2)
    //     std::cout << "Opening image from " << argv[1] << "\n";
    // else
    //     std::cout << "Using camera\n";

    std::cout << "\n";
    std::cout << " ============== NOTICE ============= " << "\n";
    std::cout << "|                                   |" << "\n";
    std::cout << "| Press 'q' to quit without saving  |" << "\n";
    std::cout << "| Press 'c' to save the input image |" << "\n";
    std::cout << "| Press 's' to save parameters      |" << "\n";
    std::cout << "| Press 'l' to load parameters      |" << "\n";
    std::cout << "| Press 'r' to restart video        |" << "\n";
    std::cout << "| Press ' ' to pause                |" << "\n";
    std::cout << "|                                   |" << "\n";
    std::cout << " =================================== " << "\n";
    std::cout << "\n";

    cv::Mat img, rgb;
    char kill = 'l';

    std::chrono::high_resolution_clock::time_point start, end;

    bool isOutputColored = true;
    int imgSizeX = 640;
    int imgSizeY = 480;
    int stream_fps = 30;
    cv::VideoWriter os ("images/snapped_images/main.avi", CV_FOURCC('P', 'I', 'M', '1'), stream_fps, cv::Size(imgSizeX, imgSizeY), isOutputColored);
    // int picId = 0;

    while (kill != 'q')
    {
#if FPS
    start = std::chrono::high_resolution_clock::now();
#endif
        // Press s to save values into FileStorage
        if (kill == 's')
        {
            std::cout << "Saving config to logs/config.yml\n";

            cv::FileStorage fs("logs/config.yml", cv::FileStorage::WRITE);
            fs << "Is Blur Window Open" << blur
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
                << "Corner Extractor Parameters" 
                << "{"
                  << "Window Name" << cornerParams.windowName
                  << "Apply Filter" << cornerParams.applyFilter
                  << "Show Windows" << cornerParams.showWindows
                  << "Quality Level" << cornerParams.qualityLevel
                  << "Minimum Distance" << cornerParams.minDist
                  << "k" << cornerParams.k
                  << "Block Size" << cornerParams.blockSize
                  << "Max Corners" << cornerParams.maxCorners
                  << "Win Size" << cornerParams.winSize
                  << "Zero Zone" << cornerParams.zeroZone
                  // << "Term Criteria" << criteria
                << "}";
            fs.release();
        }

        if (kill == 'l')
        {
            std::cout << "Loading config from logs/config.yml\n";

            cv::FileStorage fs("logs/config.yml", cv::FileStorage::READ);
            fs["Is Blur Window Open"] >> blur;
            fs["Is Color Window Open"] >> color;

            fs["Is Color Window Open"] >> color;
            fs["Is Dilate Erode Window Open"] >> dilateErode;
            fs["Is Edge Window Open"] >> edge;
            fs["Is Laplacian Window Open"] >> laplacian;
            fs["Is HoughLines Window Open"] >> houghLines;
            fs["Is HoughCircles Window Open"] >> houghCircles;
            fs["Is uShapeThreshold Window Open"] >> uShapeThresholdWindow;
            fs["Is sideRatioThreshold Window Open"] >> sideRatioThresholdWindow;
            fs["Is areaRatioThreshold Window Open"] >> areaRatioThresholdWindow;
            fs["Is angleThreshold Window Open"] >> angleThresholdWindow;
            fs["Is drawStats Open"] >> drawStats;
            fs["Is merge Open"] >> merge;

            fs["Apply Blur"] >> applyBlur;
            fs["Apply Color"] >> applyColor;
            fs["Apply DilateErode"] >> applyDilateErode;
            fs["Apply Edge"] >> applyEdge;
            fs["Apply Laplacian"] >> applyLaplacian;
            fs["Apply HoughLines"] >> applyHoughLines;
            fs["Apply HoughCircles"] >> applyHoughCircles;
            fs["Apply UShapeRatioThreshold"] >> applyUShapeThreshold;
            fs["Apply SideRatioThreshold"] >> applySideRatioThreshold;
            fs["Apply AreaRatioThreshold"] >> applyAreaRatioThreshold;
            fs["Apply AngleThreshold"] >> applyAngleThreshold;
            fs["Apply Merge"] >> applyMerge;

            fs["Gaussian Blur Kernel Size"] >> blur_ksize;
            fs["Guassian Blur Sigma X"] >> sigmaX;
            fs["Guassian Blur Sigma Y"] >> sigmaY;

            fs["Hue Minimum Threshold"] >> hMin;
            fs["Hue Maximum Threshold"] >> hMax;
            fs["Saturation Minimum Threshold"] >> sMin;
            fs["Saturation Maximum Threshold"] >> sMax;
            fs["Value Minimum Threshold"] >> vMin;
            fs["Value Maximum Threshold"] >> vMax;

            fs["Dilate Erode Holes"] >> holes;
            fs["Dilate Erode Noise"] >> noise;
            fs["Dilate Erode Size"] >> size;

            fs["Canny Lower Threshold"] >> threshLow;
            fs["Canny Higher Threshold"] >> threshHigh;

            fs["Laplacian Kernel Size"] >> laplacian_ksize;
            fs["Laplacian Scale"] >> scale;
            fs["Laplacian Delta"] >> delta;

            fs["HoughLines Rho"] >> rho;
            fs["HoughLines Theta"] >> theta;
            fs["HoughLines Threshold"] >>  threshold;
            fs["HoughLines LineMin"] >> lineMin;
            fs["HoughLines MaxGap"] >>  maxGap;

            fs["HoughCircles Minimum Distance"] >> hcMinDist;
            fs["HoughCircles Minimum Radius"] >> hcMinRadius;
            fs["HoughCircles Maximum Radius"] >> hcMaxRadius;

            fs["Merge Weight 1"] >> mergeWeight1;
            fs["Merge Weight 2"] >>  mergeWeight2;

            fs["Side Ratio Parameter"] >> sideRatioParam;
            fs["Area Ratio Parameter"] >> areaRatioParam;
            fs["Minimum Area Parameter"] >> minAreaParam;
            fs["Maximum Area Parameter"] >> maxAreaParam;
            fs["Side Ratio Maximum Deviation Parameter"] >> sideRatioMaxDeviationParam;
            fs["Area Ratio Maximum Deviation Parameter"] >> areaRatioMaxDeviationParam;
            fs["Angle Max Deviation Parameter"] >> angleMaxDeviationParam;
            fs["Corner Extractor Parameters"]["Window Name"] >> cornerParams.windowName;
            fs["Corner Extractor Parameters"]["Apply Filter"] >> cornerParams.applyFilter;
            fs["Corner Extractor Parameters"]["Show Windows"] >> cornerParams.showWindows;
            fs["Corner Extractor Parameters"]["Quality Level"] >> cornerParams.qualityLevel;
            fs["Corner Extractor Parameters"]["Minimum Distance"] >> cornerParams.minDist;
            fs["Corner Extractor Parameters"]["k"] >> cornerParams.k;
            fs["Corner Extractor Parameters"]["Block Size"] >> cornerParams.blockSize;
            fs["Corner Extractor Parameters"]["Max Corners"] >> cornerParams.maxCorners;
            fs["Corner Extractor Parameters"]["Win Size"] >> cornerParams.winSize;
            fs["Corner Extractor Parameters"]["Zero Zone"] >> cornerParams.zeroZone;

            fs.release();
        }

#if FPS
        start = std::chrono::high_resolution_clock::now();
#endif
        // TODO: acquire input through video
        // if (argc == 2)
        // {
        //     img = cv::imread(argv[1]);
        // }
        // else
        // Press space to pause program, then any key to resume
        // if (kill == ' ')
        //     cv::waitKey(0);

        if (kill != ' ')
        {
            frameInd++;
            camera >> img;
            if (frameInd == totalFrames)
            {
                frameInd = 0;
                camera.set(CV_CAP_PROP_POS_FRAMES, 0);
            }
#if IS_CAMERA_UPSIDE_DOWN
            flip(img, img, 0);
#endif
            rgb = img.clone();
        }
        else
        {
            img = rgb.clone();
        }
        if (kill == 'r')
            camera.set(CV_CAP_PROP_POS_FRAMES, 0);

        if (kill == 'p')
        {
            std::cout << "Writing input image to images/snapped_images/main.jpg\n";
            std::string s = "images/snapped_images/main.jpg";
            cv::imwrite(s, img);
        }
        // kill = 'v';
        if (kill == 'v')
        {
            // std::cout << "Writing input video to images/mjpgs\n";
            // std::string s = "images/mjpgs/main" + std::to_string(picId) + ".mjpeg";
            // picId++;
            std::cout << "Writing input video to images/snapped_images/\n";
            if (os.isOpened())
                os.write(img);
        }

#if CALIB && STREAM
        {
            selectMode(blur, color, dilateErode, edge, laplacian, houghLines, houghCircles, uShapeThresholdWindow, sideRatioThresholdWindow, areaRatioThresholdWindow, angleThresholdWindow, distanceCalib, drawStats, merge);
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
#elif CALIB
        {
            selectMode(blur, color, dilateErode, edge, laplacian, houghLines, houghCircles, uShapeThresholdWindow, sideRatioThresholdWindow, areaRatioThresholdWindow, angleThresholdWindow, distanceCalib, drawStats, merge);
            gaussianBlurWindows(img, blur_ksize, sigmaX, sigmaY, applyBlur, blur, STREAM);
            hsvColorThresholdWindows(img, hMin, hMax, sMin, sMax, vMin, vMax, debugMode, bitAnd, applyColor, color, STREAM);
            dilateErodeWindows(img, element, holes, noise, applyDilateErode, dilateErode, STREAM);
            cannyEdgeDetectWindows(img, threshLow, threshHigh, applyEdge, edge, STREAM);
            laplacianSharpenWindows(img, laplacian_ksize, scale, delta, applyLaplacian, laplacian, STREAM);
            // houghLinesWindows(img, rho, theta, threshold, lineMin, maxGap, applyHoughLines, houghLines, STREAM);
            houghCirclesWindows(img, hcMinDist, hcMinRadius, hcMaxRadius, applyHoughCircles, houghCircles, STREAM);
        }
#else
        {
            gaussianBlur(img, blur_ksize, sigmaX, sigmaY);
            hsvColorThreshold(img, hMin, hMax, sMin, sMax, vMin, vMax, debugMode, bitAnd);
        }
#endif

        try
        {
            contours = getContours(img, contoursThresh);
        }
        catch (std::exception& e)
        {
            std::cout << e.what() << "\n";
        }

        if (contours.size() > 0)
        {
            boundedRects = getBoundedRects(contours);
            {
#if CALIB
                {
                    uShapeThresholdWindows(img, contours, boundedRects, minDistFromContours, maxDistFromContours, applyUShapeThreshold, uShapeThresholdWindow, STREAM);
                    sideRatioThresholdWindows(img, contours, boundedRects, sideRatioParam, sideRatioMaxDeviationParam, applySideRatioThreshold, sideRatioThresholdWindow, STREAM);
                    areaRatioThresholdWindows(img, contours, boundedRects, minAreaParam, maxAreaParam, areaRatioParam, areaRatioMaxDeviationParam, applyAreaRatioThreshold, areaRatioThresholdWindow, STREAM);
                    angleThresholdWindows(img, contours, boundedRects, angleMaxDeviationParam, applyAngleThreshold, angleThresholdWindow, STREAM);
                }
#else
                {
                    // Filter out contours that do not match the 'U' shape
                    uShapeThreshold(contours, boundedRects, minDistFromContours, maxDistFromContours);
                    sideRatioThreshold(contours, boundedRects, sideRatio, sideRatioMaxDeviation);
                    areaRatioThreshold(contours, boundedRects, minArea, maxArea, areaRatio, areaRatioMaxDeviation);
                    angleThreshold(contours, boundedRects, angleMaxDeviation);
                }
#endif
            }
            if (contours.size() > 0)
            {
                goalInd = getLeftMostContourIndex(contours);
                boundedRects[goalInd].points(rectPoints); // Extract the corners of the target's bounded box

                cv::Mat mask = cv::Mat::zeros(img.size(), CV_8UC1);
                // Draw a filled in bounded rectangle at the goal
                cv::Point rectIntPoints [MAX_GAME_PIECE_CORNERS];
                cvtPoint(rectPoints, rectIntPoints);
                cv::fillConvexPoly(mask, rectIntPoints, MAX_GAME_PIECE_CORNERS, PURPLE);
                // cv::imshow("mask", mask);

                // cv::namedWindow("Linearize Contours");
                // cv::createTrackbar("Epsilon", "Linearize Contours", &approximationAccuracy, 100);
                //
                // std::vector< std::vector<cv::Point> > linearContours;
                // linearContours.resize(contours.size());
                // for (size_t i = 0; i < contours.size(); ++i)
                //     cv::approxPolyDP(contours[i], linearContours[i], (double)approximationAccuracy, true);
                // cv::Mat imgBlank;
                // imgBlank = rgb.clone();
                // drawContours(imgBlank, linearContours, -1, YELLOW, lineThickness, 8);
                // cv::imshow("Linear Contours", imgBlank);

                cv::Point mc;
                std::vector<cv::Point2f> corners;
                {
                    {
                        // drawContours(tmp, contours, -1, RED, lineThickness, 8);
                        // drawBoundedRects(tmp, boundedRects, PURPLE);
                        // cv::imshow("Contours and Rects", tmp);
                    }
                    cv::Mat img_gray, rectROI;
                    img_gray = img.clone();
                    // cv::cvtColor(rgb, img_gray, CV_BGR2GRAY);
                    cv::cvtColor(img_gray, img_gray, CV_BGR2GRAY);
                    // Find only the region of the grayscale image inside the bounding box
                    img_gray.copyTo(rectROI, mask);
                    // cv::Canny(rectROI, rectROI, threshLow, threshHigh);
                    mc = getCenterOfMass(contours[goalInd]);
                    gamePiece.update(rectROI, contours[goalInd], mc);
                    corners = gamePiece.getCorners();
                    // Draw the corners of the target's contour
                    {
                        char str[100];
                        for( size_t i = 0; i < corners.size(); ++i )
                        {
                            sprintf(str, "%zu", i);
                            cv::putText(img_gray, str, corners[i], CV_FONT_HERSHEY_COMPLEX_SMALL, 0.75, BLUE_GREEN, 1, 8, false);
                            int r = 1;
                            // circle( img_gray, corners[i], r, cv::Scalar(rng_.uniform(0,255), rng_.uniform(0,255), rng_.uniform(0,255)), -1, 8, 0 );
                            circle( img_gray, corners[i], r, cv::Scalar(0, 255, 255), -1, 8, 0 );
                        }
                    }
                    cv::imshow("Good Corners", img_gray);
                    // Draw in filled circles of radius 3 at the identified corners
                    // for (size_t i = 0; i < goodCorners.size(); ++i)
                    // {
                    //     cv::circle(imgBlank, goodCorners[i], 3, YELLOW, -1);
                    // }
                    // cv::imshow("Corners", imgBlank);
                }

// #if 0

                // Find the corners of the target's contours
                // std::vector<cv::Point> corners = getCorners(contours[goalInd], SCREEN_WIDTH, SCREEN_HEIGHT);
                // gamePiece.update(contours[goalInd], mc, 2);
                // if (corners.size() > 0)
                //     cv::cornerSubPix(img, corners, cv::Size(3,3), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::COUNT, 10, 100));

                // cv::Vec3f angleVec = cv::Vec3f(0,0,0);
                // Temporary storage in vector
                // Distances in inches
                cv::Vec3f angleVec = getAngularPosition(img, rvec, tvec, corners, CAMERA_CONFIG_FILE, GAME_ELEMENT_WIDTH, GAME_ELEMENT_HEIGHT);

                // std::cout << getScalePixToMM() << std::endl;
                // std::cout << "\nDistances in pixels\n";
                // std::cout << "X: " << ((mc.x - SCREEN_WIDTH / 2) * getScalePixToMM() / INCHES_TO_MM) << "\n";
                // std::cout << "Z: " << angleVec[1] << "\n";
                // std::cout << "MC: " << (mc.x - SCREEN_WIDTH / 2) << "\n";
                // std::cout << "X from POSE: " << angleVec[0] << "\n";
                // getFocalLenMM();
                // yaw = getYawToCenterOfMass(mc, angleVec[1]);
                const double DFOV = 78; // Diagonal FOV in degrees for Logitech C920
                // const double DFOV = 43.875; // Diagonal FOV in degrees for Logitech C920
                double DSCREEN = std::sqrt(SCREEN_WIDTH * SCREEN_WIDTH + SCREEN_HEIGHT * SCREEN_HEIGHT); // Diagonal FOV of camera screen
                const double PIX_TO_DEG = DFOV / DSCREEN;

                // Get the distance in inches and angles in degrees
                // calibrateFocalLength(img, focalLength, calibDistance, heightDisplacement, contoursThresh, applyDistanceCalib, showDistanceCalib, distanceCalib);
                // double euclidDist = getEuclideanDistance(rectPoints, GAME_ELEMENT_WIDTH, focalLength, calibDistance, applyDistanceCalib);
                // yaw = getYaw(SCREEN_WIDTH, angelVec[1], GAME_ELEMENT_WIDTH, corners, mc);
                // pitch = getPitch(height, euclidDist);


                // I'll refactor everything later, here's the data that
                // will be sent to the roboRIO
                double parallelDist = angleVec[1]; // Distance, parallel to ground
                double theta = angleVec[2]; // Angular position
                yaw = ((mc.x - SCREEN_WIDTH / 2) * PIX_TO_DEG); // Robot heading
                // std::cout << "Yaw: " << yaw << "\n";
                {
                    if (!std::isfinite(yaw)) yaw = -1;
                    // if (!std::isfinite(pitch)) pitch = -1;
                    // if (!std::isfinite(euclidDist)) euclidDist = -1;
                    if (!std::isfinite(angleVec[0])) angleVec[0] = -1;
                    if (!std::isfinite(parallelDist)) parallelDist = -1;
                    if (!std::isfinite(theta)) theta = -1;

                    std::string gnuplotBuf = std::to_string(yaw) 
                        // + " " + std::to_string(euclidDist) 
                        + " " + std::to_string(angleVec[0])
                        + " " + std::to_string(parallelDist)  // Euclidean distance
                        + " " + std::to_string(theta); // Angular position

                    dataFile << gnuplotBuf.c_str() << std::endl;
                }

                // if (drawStats)
                    // putData(img, focalLength, euclidDist, yaw, pitch);

                {
                    // Draw all the contours
                    drawContours(img, contours, -1, RED, lineThickness, 8);
                    drawBoundedRects(img, boundedRects, PURPLE);

                    cv::line(img, cv::Point(SCREEN_WIDTH / 2, mc.y), mc, YELLOW); // Draw line from center of img to center of mass
                    cv::circle(img, mc, 5, YELLOW); // Draw center of mass
                    // // Draw the corners of the target's contour
                    // {
                    //     char str[30];
                    //     for (int i = 0; i < 4; ++i)
                    //     {
                    //         cv::circle(img, corners[i], 3, LIGHT_GREEN);
                    //         sprintf(str, "%d", i+1);
                    //         cv::putText(img, str, corners[i], CV_FONT_HERSHEY_COMPLEX_SMALL, 0.75, BLUE_GREEN, 1, 8, false);
                    //     }
                    // }
                }
// #endif
            }
        }
#if CALIB
        {
            if (uShapeThresholdWindow || sideRatioThresholdWindow || areaRatioThresholdWindow || angleThresholdWindow) 
            {
                if (STREAM) mjpgStream(img);
                else cv::imshow("Threshold Output", img);
            }
            else if (!STREAM) cv::destroyWindow("Threshold Output");
            if (!STREAM) cv::imshow("Final Output", img);
            mergeFinalWindows(rgb, img, mergeWeight1, mergeWeight2, applyMerge, merge, STREAM);
            if (STREAM && !blur && !color && !dilateErode && !edge && !laplacian && !houghLines && !houghCircles && !uShapeThresholdWindow && !sideRatioThresholdWindow && !areaRatioThresholdWindow && !angleThresholdWindow && !drawStats)
                mjpgStream(img);
        }
#elif STREAM
        {
            // Stream the final merged image
            mergeFinal(rgb, img, mergeWeight1, mergeWeight2);
            mjpgStream(img);
        }
#endif
#if FPS
        {
            end = std::chrono::high_resolution_clock::now();
            auto timeElapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
            // fps = 1.0 / timeElapsed.count();

            // std::string gnuplotBuf = std::to_string(fps);
            std::string gnuplotBuf = std::to_string(timeElapsed) + "  " + std::to_string(timeElapsed / ++fpsTick);

            fpsFile << gnuplotBuf.c_str() << std::endl;
            avg += timeElapsed;
            // fpsTick++;
        }
#endif 
#if STREAM
        kill = '~'; // Junk value that should never be used as an action
#else
        // Paused
        if (kill == ' ') 
        {
            kill = cv::waitKey(5); // Only wait x milliseconds if using cv::imshow()
            if (kill == ' ') // Unpause
                kill = '~';
            if (kill == -1) // No key was pressed while program is paused, so stay paused
                kill = ' ';
        }
        else
        {
            kill = cv::waitKey(5); // Only wait x milliseconds if using cv::imshow()
        }
#endif
    }
    os.release();
    return 0;
}
