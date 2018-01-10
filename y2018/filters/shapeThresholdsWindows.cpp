#include "filters/shapeThresholdsWindows.hpp"

void sideRatioThresholdWindows(cv::Mat& img, std::vector<std::vector<cv::Point> >& contours, std::vector<cv::RotatedRect>& boundedRects, int& sideRatio, int& sideRatioMaxDeviation, int& apply, bool visible, const bool isStreaming)
{
	if (visible)
	{
		cv::namedWindow("Side Ratio Threshold Editor", CV_WINDOW_NORMAL);
        cv::resizeWindow("Side Ratio Threshold Editor", 500, 500);
		cv::createTrackbar("Apply Filter", "Side Ratio Threshold Editor", &apply, 1);
		cv::createTrackbar("Percent Length to Width Ratio", "Side Ratio Threshold Editor", &sideRatio, 500);
		cv::createTrackbar("+- Length to Width Ratio Deviation (Divide by 100)", "Side Ratio Threshold Editor", &sideRatioMaxDeviation, 100);
    }
	else
	{
		cv::destroyWindow("Side Ratio Threshold Editor");
		cv::destroyWindow("Side Ratio Threshold Output");
	}
	if (apply)
	{
		sideRatioThreshold(contours, boundedRects, static_cast<double>(sideRatio) / 100, static_cast<double>(sideRatioMaxDeviation) / 100);
        if (visible && !isStreaming)
        {
            cv::namedWindow("Side Ratio Threshold Output", CV_WINDOW_AUTOSIZE);
            cv::imshow("Side Ratio Threshold Output", img);
        }
	}	
	else
	{
		cv::destroyWindow("Side Ratio Threshold Output");
	}
}

void angleThresholdWindows(cv::Mat& img, std::vector<std::vector<cv::Point> >& contours, std::vector<cv::RotatedRect>& boundedRects, int& angleMaxDeviation, int& apply, bool visible, const bool isStreaming)
{
	if (visible)
	{
		cv::namedWindow("Angle Threshold Editor", CV_WINDOW_AUTOSIZE);
		cv::createTrackbar("Apply Filter", "Angle Threshold Editor", &apply, 1);
		cv::createTrackbar("+- Angle Deviation From 0", "Angle Threshold Editor", &angleMaxDeviation, 90);
    }
	else
	{
		cv::destroyWindow("Angle Threshold Editor");
		cv::destroyWindow("Angle Threshold Output");
	}
	if (apply)
	{
		angleThreshold(contours, boundedRects, angleMaxDeviation);
        if (visible && !isStreaming)
        {
            cv::namedWindow("Angle Threshold Output", CV_WINDOW_AUTOSIZE);
            cv::imshow("Angle Threshold Output", img);
        }
	}	
	else
	{
		cv::destroyWindow("Angle Threshold Output");
	}
}

void areaRatioThresholdWindows(cv::Mat& img, std::vector<std::vector<cv::Point> >& contours, std::vector<cv::RotatedRect>& boundedRects, int& minArea, int& maxArea, int& areaRatio, int& areaRatioMaxDeviation, int& apply, bool visible, const bool isStreaming)
{
	if (visible)
	{
		cv::namedWindow("Area Ratio Threshold Editor", CV_WINDOW_AUTOSIZE);
		cv::createTrackbar("Apply Filter", "Area Ratio Threshold Editor", &apply, 1);
		cv::createTrackbar("Minimum Rect Area", "Area Ratio Threshold Editor", &minArea, 20000);
		cv::createTrackbar("Maximum Rect Area", "Area Ratio Threshold Editor", &maxArea, 200000);
		cv::createTrackbar("Percent Contour to Rect Area Ratio", "Area Ratio Threshold Editor", &areaRatio, 100);
		cv::createTrackbar("+- Area Ratio Deviation (Divide by 100)", "Area Ratio Threshold Editor", &areaRatioMaxDeviation, 100);
    }
	else
	{
		cv::destroyWindow("Area Ratio Threshold Editor");
		cv::destroyWindow("Area Ratio Threshold Output");
	}
	if (apply)
	{
		areaRatioThreshold(contours, boundedRects, minArea, maxArea, static_cast<double>(areaRatio) / 100, static_cast<double>(areaRatioMaxDeviation) / 100);
        if (visible && !isStreaming)
        {
            cv::namedWindow("Area Ratio Threshold Output", CV_WINDOW_AUTOSIZE);
            cv::imshow("Area Ratio Threshold Output", img);
        }
	}	
	else
	{
		cv::destroyWindow("Area Ratio Threshold Output");
	}
}

void uShapeThresholdWindows(cv::Mat& img, std::vector<std::vector<cv::Point> >& contours, std::vector<cv::RotatedRect>& boundedRects, int& minDist, int& maxDist, int& apply, bool visible, const bool isStreaming)
{
	if (visible)
	{
		cv::namedWindow("U Shape Threshold Editor", CV_WINDOW_AUTOSIZE);
		cv::createTrackbar("Apply Filter", "U Shape Threshold Editor", &apply, 1);
		cv::createTrackbar("Min Dist Between Center of Mass and Contours", "U Shape Threshold Editor", &minDist, 100);
		cv::createTrackbar("Max Dist Between Center of Mass and Contours", "U Shape Threshold Editor", &maxDist, 100);
    }
	else
	{
		cv::destroyWindow("U Shape Threshold Editor");
		cv::destroyWindow("U Shape Threshold Output");
	}
	if (apply)
	{
        uShapeThreshold(contours, boundedRects, minDist, maxDist);
        if (visible && !isStreaming)
        {
            cv::namedWindow("U Shape Threshold Output", CV_WINDOW_AUTOSIZE);
            cv::imshow("U Shape Threshold Output", img);
        }
	}	
	else
	{
		cv::destroyWindow("U Shape Threshold Output");
	}
}
