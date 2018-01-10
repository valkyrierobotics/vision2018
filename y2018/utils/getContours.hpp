#ifndef GET_CONTOURS_HPP
#define GET_CONTOURS_HPP

#include <vector>
#include <opencv2/imgproc/imgproc.hpp>

std::vector< std::vector<cv::Point> > getContours(const cv::Mat& src, int contoursThresh);

#endif // GET_CONTOURS_HPP
