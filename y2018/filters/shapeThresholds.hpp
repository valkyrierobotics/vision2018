#ifndef SHAPE_tHRESHOLDS_HPP
#define SHAPE_tHRESHOLDS_HPP

#include <opencv2/imgproc/imgproc.hpp>

#include <vector>

void areaRatioThreshold (std::vector<std::vector<cv::Point> >& contours, std::vector<cv::RotatedRect>& boundedRects, int minArea, int maxArea, double areaRatio, double areaRatioMaxDeviation);
void sideRatioThreshold (std::vector<std::vector<cv::Point> >& contours, std::vector<cv::RotatedRect>& boundedRects, double sideRatio, double sideRatioMaxDeviation);
void angleThreshold (std::vector<std::vector<cv::Point> >& contours, std::vector<cv::RotatedRect>& boundedRects, int angleMaxDeviation);
void uShapeThreshold (std::vector< std::vector<cv::Point> >& contours, std::vector<cv::RotatedRect>& boundedRects, int minDist, int maxDist);

#endif // SHAPE_tHRESHOLDS_HPP
