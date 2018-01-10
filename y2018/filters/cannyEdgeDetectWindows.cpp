#include "filters/cannyEdgeDetectWindows.hpp"

void cannyEdgeDetectWindows(cv::Mat &img, int &threshLow, int &threshHigh, int &apply, bool visible, const bool isStreaming)
{
	if (visible)
   	{
		cv::namedWindow("Canny Edge Detection Editor", cv::WINDOW_AUTOSIZE);

		cv::createTrackbar("Apply Filter", "Canny Edge Detection Editor", &apply, 1);
		cv::createTrackbar("Bottom Threshold", "Canny Edge Detection Editor", &threshLow, 300);
		cv::createTrackbar("Upper Threshold", "Canny Edge Detection Editor", &threshHigh, 300);
	} 
	else
   	{
		cv::destroyWindow("Canny Edge Detection Editor");	 
		cv::destroyWindow("Canny Edge Detection Output");
	}
	if (apply)	
	{
		cannyEdgeDetect(img, threshLow, threshHigh);
        if (visible && !isStreaming)
        {
            cv::namedWindow("Canny Edge Detection Output", cv::WINDOW_AUTOSIZE);
            cv::imshow("Canny Edge Detection Output", img);
        }
	}
	else
	{
		cv::destroyWindow("Canny Edge Detection Output");
	}
}
