#ifndef HOUGH_CIRCLES_WINDOWS_HPP
#define HOUGH_CIRCLES_WINDOWS_HPP

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "houghCircles.hpp"

void houghCirclesWindows(cv::Mat& img, int& minDist, int& minRadius, int& maxRadius, int& apply, bool visible, const bool isStreaming);

#endif // HOUGH_CIRCLES_WINDOWS_HPP
