#include "filters/cannyEdgeDetect.hpp"

void cannyEdgeDetect(cv::Mat& img, int threshLow, int threshHigh)
{
    // if (img.channels() == 3)
    //     cv::cvtColor(img, img, CV_BGR2GRAY);

    cv::cvtColor(img, img, CV_BGR2GRAY);
	cv::Canny(img, img, threshLow, threshHigh);
    cv::cvtColor(img, img, CV_GRAY2BGR);
}
