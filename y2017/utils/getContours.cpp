#include "utils/getContours.hpp"
#include "opencv2/highgui/highgui.hpp"

std::vector< std::vector<cv::Point> > getContours(const cv::Mat& src, int contoursThresh)
{
	std::vector< std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
  cv::Mat tmp;

	// Convert src to gray format
	cv::cvtColor(src, tmp, CV_BGR2GRAY);
	cv::findContours(tmp, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
	
	return contours;
}

