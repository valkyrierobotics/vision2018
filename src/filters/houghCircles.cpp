#include "filters/houghCircles.hpp"

void houghCircles(cv::Mat& img, int minDist, int minRadius, int maxRadius)
{    
  	std::vector<cv::Vec3f> circles;

	cv::cvtColor(img, img, CV_BGR2GRAY);
    // cv::HoughCircles calls cv::Canny internally
 	cv::HoughCircles(img, circles, CV_HOUGH_GRADIENT, 2, minDist, maxRadius, minRadius);

  	for (size_t i = 0; i < circles.size(); i++)
  	{
        // Each circle is encoded in a 3 element vector (x, y, radius)
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        // Draw center of circle
        cv::circle(img, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
        // Draw circle
        cv::circle(img, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
   	}
}
