#ifndef SHAPE_THRESHOLDS_WINDOWS_HPP
#define SHAPE_THRESHOLDS_WINDOWS_HPP

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <vector>

#include "shapeThresholds.hpp"

void sideRatioThresholdWindows(cv::Mat& img, std::vector<std::vector<cv::Point> >& contours, std::vector<cv::RotatedRect>& boundedRects, int& sideRatio, int& sideRatioMaxDeviation, int& apply, bool visible, const bool STREAM);
void angleThresholdWindows(cv::Mat& img, std::vector<std::vector<cv::Point> >& contours, std::vector<cv::RotatedRect>& boundedRects, int& angleMaxDeviation, int& apply, bool visible, const bool STREAM);
void areaRatioThresholdWindows(cv::Mat& img, std::vector<std::vector<cv::Point> >& contours, std::vector<cv::RotatedRect>& boundedRects, int& minArea, int& maxArea, int& areaRatio, int& areaRatioMaxDeviation, int& apply, bool visible, const bool STREAM);
void uShapeThresholdWindows(cv::Mat& img, std::vector<std::vector<cv::Point> >& contours, std::vector<cv::RotatedRect>& boundedRects, int& minDist, int& maxDist, int& apply, bool visible, const bool STREAM);

#endif // SHAPE_THRESHOLDS_WINDOWS_HPP
