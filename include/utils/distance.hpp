#ifndef DISTANCE_HPP
#define DISTANCE_HPP

#include "opencv2/imgproc/imgproc.hpp"
#include <math.h>

double distance(cv::Point one, cv::Point two);
double getShortestDistance (cv::Point2f rectPoints[4], double widthInInches, double focalLen, int dist, int isCalib);

#endif // DISTANCE_HPP
