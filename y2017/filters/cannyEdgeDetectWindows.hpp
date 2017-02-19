#ifndef CANNY_EDGE_DETECT_WINDOWS_HPP
#define CANNY_EDGE_DETECT_WINDOWS_HPP

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "cannyEdgeDetect.hpp"

void cannyEdgeDetectWindows(cv::Mat &img, int &threshLow, int &threshHigh, int &apply, bool visible, const bool STREAM);

#endif // CANNY_EDGE_DETECT_WINDOWS_HPP
