#include "utils/distance.hpp"
#include <opencv2/highgui/highgui.hpp>

double calibrateFocalLength(cv::Mat& img, double& focalLen, int& dist, int& h, int& contoursThresh, int& isCalib, int& apply, bool visible)
{
    if (visible)
    {
        cv::namedWindow("Distance and Angles", CV_WINDOW_AUTOSIZE);
        cv::createTrackbar("Apply Filter", "Distance and Angles", &apply, 1);
        cv::createTrackbar("Calibration Status", "Distance and Angles", &isCalib, 1);
        cv::createTrackbar("Calibration Distance", "Distance and Angles", &dist, 300);
        cv::createTrackbar("Calibration Height", "Distance and Angles", &h, 200);
        cv::createTrackbar("Contours Threshold", "Distance and Angles", &contoursThresh, 200);
    }
    else
    {
        cv::destroyWindow("Distance and Angles");
    }
    if (apply)
    {
        if (visible)
        {
            cv::namedWindow("Distance and Angles", CV_WINDOW_AUTOSIZE);
            cv::imshow("Distance and Angles", img);
        }
    }
    else
    {
        cv::imshow("Distance and Angles", cv::Mat::zeros(img.size(), CV_8UC1));
    }
}


// Find the shortest distance to a target (in inches) based off of a calculated focal length
double getEuclideanDistance (cv::Point2f rectPoints[4], double widthInInches, double& focalLen, int dist, int isCalib)
{
    int length = distance (rectPoints[0], rectPoints[3]);
    int width = distance (rectPoints[0], rectPoints[1]);

    double widthInPixels = 0;
    if (length > width)
        widthInPixels = length;
    else
        widthInPixels = width;

    double inchesPerPixel = widthInInches / widthInPixels;

    // Recalculate focal length (distance in pixels)
    if (isCalib == 1)
        focalLen = dist * (1 / inchesPerPixel);

    return focalLen * inchesPerPixel;
}
