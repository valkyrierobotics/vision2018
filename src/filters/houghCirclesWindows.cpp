#include "filters/houghCirclesWindows.hpp"

void houghCirclesWindows(cv::Mat& img, int& minDist, int& minRadius, int& maxRadius, int& apply, bool visible, const bool STREAM)
{
	if (visible)
	{
		cv::namedWindow("Hough Circles Editor", cv::WINDOW_AUTOSIZE);	
		cv::createTrackbar("Apply Filter", "Hough Circles Editor", &apply, 1);
		cv::createTrackbar("Min Dist Between Circle Centers", "Hough Circles Editor", &minDist, img.rows);
		cv::createTrackbar("Min Radius", "Hough Circles Editor", &maxRadius, img.rows / 2);
		cv::createTrackbar("Max Radius", "Hough Circles Editor", &minRadius, img.rows / 2);
	}
	else
	{
		cv::destroyWindow("Hough Circles Editor");
		cv::destroyWindow("Hough Circles Output");
	}
	if (apply)
	{
		houghCircles(img, minDist, minRadius, maxRadius);
        if (visible && !STREAM)
        {
            cv::namedWindow("Hough Circles Output", CV_WINDOW_AUTOSIZE);
            cv::imshow("Hough Circles Output", img);
        }
	}
	else
	{
		cv::destroyWindow("Hough Circles Output");
	}
}
