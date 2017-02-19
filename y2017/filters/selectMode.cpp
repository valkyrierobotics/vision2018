#include "filters/selectMode.hpp"

// Interface for selecting filters to view and apply
void selectMode(int &blur, int &colorThreshold, int &dilateErode, int &edge, int &laplacian, int &houghLines, int& houghCircles, int& uShapeThreshold, int &sideRatioThreshold, int& areaRatioThreshold, int& angleThreshold, int& distanceCalib, int& drawStats, int& merge)
{
	cv::namedWindow("Filter Options");
    cv::resizeWindow("Filter Options", 500, 1000);

	cv::createTrackbar("Blur Filter", "Filter Options", &blur, 1);
	cv::createTrackbar("Color Filter", "Filter Options", &colorThreshold, 1);
	cv::createTrackbar("Dilate Erode Filter", "Filter Options", &dilateErode, 1);
	cv::createTrackbar("Edge Detection Filter", "Filter Options", &edge, 1);
	cv::createTrackbar("Laplacian Filter", "Filter Options", &laplacian, 1);
	cv::createTrackbar("Hough Lines Filter", "Filter Options", &houghLines, 1);
	cv::createTrackbar("Hough Circles Filter", "Filter Options", &houghCircles, 1);
    cv::createTrackbar("U Shape Threshold", "Filter Options", &uShapeThreshold, 1);
	cv::createTrackbar("Side Ratio Threshold", "Filter Options", &sideRatioThreshold, 1);
	cv::createTrackbar("Area Ratio Threshold", "Filter Options", &areaRatioThreshold, 1);
	cv::createTrackbar("Angle Threshold", "Filter Options", &angleThreshold, 1);
	cv::createTrackbar("Distance Calib", "Filter Options", &distanceCalib, 1);
	cv::createTrackbar("Draw Statistics", "Filter Options", &drawStats, 1);
	cv::createTrackbar("Merge After Filtering", "Filter Options", &merge, 1);
}
