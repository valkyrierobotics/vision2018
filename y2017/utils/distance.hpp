#ifndef DISTANCE_HPP
#define DISTANCE_HPP

#include "opencv2/imgproc/imgproc.hpp"
#include <math.h>

// Returns the minimum distance (straight line) between b points
template <class T>
double distance(T a, T b)
{
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
    // return cv::norm(a.x - b.x, a.y - b.y);
}

double calibrateFocalLength(cv::Mat& img, double& focalLen, int& dist, int& h, int& contoursThresh, int& isCalib, int& apply, bool visible);
double getEuclideanDistance (cv::Point2f rectPoints[4], double widthInInches, double& focalLen, int dist, int isCalib);

#endif // DISTANCE_HPP
