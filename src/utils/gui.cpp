#include "utils/gui.hpp"

void putData(cv::Mat& img, double distance, double yaw, double pitch)
{
    char str[30];

	sprintf(str, "Distance = %4.2f", distance);
	cv::putText(img, str, cv::Point(10, 430), CV_FONT_HERSHEY_COMPLEX_SMALL, 0.75, cv::Scalar(255, 0, 200), 1, 8, false);
	sprintf(str, "Yaw      = %4.2f", yaw);
	cv::putText(img, str, cv::Point(10, 450), CV_FONT_HERSHEY_COMPLEX_SMALL, 0.75, cv::Scalar(255, 0, 200), 1, 8, false);
	sprintf(str, "Pitch    = %4.2f", pitch);
	cv::putText(img, str, cv::Point(10, 470), CV_FONT_HERSHEY_COMPLEX_SMALL, 0.75, cv::Scalar(255, 0, 200), 1, 8, false);
}

void drawContours(cv::Mat& img, std::vector<std::vector<cv::Point> >& contours, cv::Scalar& color)
{
	std::vector<cv::Vec4i> hierarchy;
	
    for(size_t i = 0; i < contours.size(); ++i)
        cv::drawContours(img, contours, i, color, 2, 8, hierarchy, 0, cv::Point());
}

void drawBoundedRects(cv::Mat& img, std::vector<cv::RotatedRect>& boundedRects, cv::Scalar& color)
{
    int thickness = 1;
    int lineType = 8;
    for (size_t i = 0; i < boundedRects.size(); ++i)
    {
        cv::Point2f rectPoints[4];
        boundedRects[i].points(rectPoints);

        for (int p = 0; p < 4; ++p)
            cv::line(img, rectPoints[p], rectPoints[(p+1) % 4], color, thickness, lineType);
    }
}
