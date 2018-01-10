#include "filters/gaussianBlurWindows.hpp"

void gaussianBlurWindows(cv::Mat &img, int &blur_ksize, int &sigmaX, int &sigmaY, int &apply, bool visible, const bool isStreaming)
{
	if (visible)
   	{
		cv::namedWindow("Gaussian Blur Editor", cv::WINDOW_AUTOSIZE);
		cv::createTrackbar("Apply Filter", "Gaussian Blur Editor", &apply, 1);
		cv::createTrackbar("Kernel Size", "Gaussian Blur Editor", &blur_ksize, 10);
		cv::createTrackbar("Sigma X", "Gaussian Blur Editor", &sigmaX, 100);
		cv::createTrackbar("Sigma Y", "Gaussian Blur Editor", &sigmaY, 100);
	}
   	else
	{
		cv::destroyWindow("Gaussian Blur Editor");	
		cv::destroyWindow("Gaussian Blur Output");
	}
	if (apply)
	{
		gaussianBlur(img, blur_ksize, sigmaX, sigmaY);
        if (visible && !isStreaming)
        {
            //cv::namedWindow("Gaussian Blur Output", cv::WINDOW_AUTOSIZE);
            //cv::imshow("Gaussian Blur Output", img);
            cv::imshow("Gaussian Blur Editor", img);
        }
	}
	else
	{
		cv::destroyWindow("Gaussian Blur Output");
	}
}
