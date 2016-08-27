#include "filters/mergeFinalWindows.hpp"

// 8UC3 color, 8UC1 depth
void mergeFinalWindows(cv::Mat& img1, cv::Mat& img2, int& weight1, int& weight2, int& apply, bool visible, const bool STREAM)
{
	if (visible)
	{
		cv::namedWindow("Merge Final Editor", cv::WINDOW_AUTOSIZE);

		cv::createTrackbar("Apply Filter", "Merge Final Editor", &apply, 1);
		cv::createTrackbar("Weight of Original Image", "Merge Final Editor", &weight1, 100);
		cv::createTrackbar("Weight of Filtered Image", "Merge Final Editor", &weight2, 100);
	}
	else
	{
		cv::destroyWindow("Merge Final Editor");
		cv::destroyWindow("Merge Final Output");
	}
	if (apply)	
	{
		mergeFinal(img1, img2, weight1, weight2);
        if (visible && !STREAM)
        {
            cv::namedWindow("Merge Final Output", cv::WINDOW_AUTOSIZE);
            cv::imshow("Merge Final Output", img2);
        }
	}
	else
	{
		cv::destroyWindow("Merge Final Output");
	}
}
