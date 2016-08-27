#ifndef HOUGH_CIRCLES_HPP
#define HOUGH_CIRCLES_HPP

#include <opencv2/imgproc/imgproc.hpp>

void houghCircles(cv::Mat& img, int minDist, int minRadius, int maxRadius);

#endif // HOUGH_CIRCLES_HPP
