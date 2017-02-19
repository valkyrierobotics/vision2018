#include "filters/laplacianSharpen.hpp"

void laplacianSharpen(cv::Mat &img, int ksize, int scale, int delta)
{
    // If kernel size is even, round to nearest upper odd number
    // because laplacian kernel size must be odd
	if (ksize % 2 == 0)
		ksize++;

    if (img.channels() == 3)
        cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);

    // Assuming input is CV_8U, use CV_16S for ddepth to prevent overflow
    int ddepth = CV_16S;
 	cv::Laplacian(img, img, ddepth, ksize, scale, delta, cv::BORDER_DEFAULT);
  	cv::convertScaleAbs(img, img);
}
