#ifndef GUI_HPP
#define GUI_HPP

#include <opencv2/imgproc/imgproc.hpp>

void putData(cv::Mat& image, double distance, double yaw, double pitch);
void drawContours(cv::Mat& img, std::vector<std::vector<cv::Point> >& contours, cv::Scalar& color);
void drawBoundedRects(cv::Mat& img, std::vector<cv::RotatedRect>& boundedRects, cv::Scalar& color);

#endif // GUI_HPP
