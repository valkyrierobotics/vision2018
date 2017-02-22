#ifndef DISTANCE_HPP
#define DISTANCE_HPP

#include "opencv2/imgproc/imgproc.hpp"
#include <math.h>

// Returns the Euclidean distance between two points
template <class T1, class T2>
double distance(T1 a, T2 b)
{
  return std::sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y));
}

double calibrateFocalLength(cv::Mat& img, double& focalLen, int& dist, int& h, int& contoursThresh, int& isCalib, int& apply, bool visible);
double getEuclideanDistance (cv::Point2f rectPoints[4], double widthInInches, double& focalLen, int dist, int isCalib);

#endif // DISTANCE_HPP
