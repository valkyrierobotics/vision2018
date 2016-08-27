#include "filters/laplacianSharpenWindows.hpp"

void laplacianSharpenWindows(cv::Mat &img, int &ksize, int &scale, int &delta, int &apply, bool visible, const bool STREAM)
{
	if (visible)
   	{
		cv::namedWindow("Laplacian Sharpen Editor", cv::WINDOW_AUTOSIZE);

		cv::createTrackbar("Apply Filter", "Laplacian Sharpen Editor", &apply, 1);
		cv::createTrackbar("Kernel Size (Rounded to Upper Odd Number)", "Laplacian Sharpen Editor", &ksize, 9);
		cv::createTrackbar("Scale", "Laplacian Sharpen Editor", &scale, 9);
		cv::createTrackbar("Delta", "Laplacian Sharpen Editor", &delta, 9);
	}
   	else
   	{
		cv::destroyWindow("Laplacian Sharpen Editor");	 
		cv::destroyWindow("Laplacian Sharpen Output");
	}
	if (apply)
	{
		laplacianSharpen(img, ksize, scale, delta);
        if (visible && !STREAM)
        {
            cv::namedWindow("Laplacian Sharpen Output", cv::WINDOW_AUTOSIZE);
            cv::imshow("Laplacian Sharpen Output", img);
        }
	}
	else
	{
		cv::destroyWindow("Laplacian Sharpen Output");
	}
}
