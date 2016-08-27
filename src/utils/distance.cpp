#include "utils/distance.hpp"

// Returns the minimum distance (straight line) between two points
double distance(cv::Point one, cv::Point two)
{
    return std::sqrt(std::pow(one.x - two.x, 2) + std::pow(one.y - two.y, 2));
}

// Find the shortest distance to a target (in inches) based off of a calculated focal length
double getShortestDistance (cv::Point2f rectPoints[4], double widthInInches, double focalLen, int dist, int isCalib)
{
	double widthInPixels = 0;

	int length = distance (rectPoints[0], rectPoints[3]);
	int width = distance (rectPoints[0], rectPoints[1]);
	
	if (length > width)
		widthInPixels = length;
	else
		widthInPixels = width;

    double inchesPerPixel = widthInInches / widthInPixels;

    // Recalculate focal length (distance in pixels)
    if (isCalib == 1)
        focalLen = dist * (1 / inchesPerPixel);
		
    return focalLen * inchesPerPixel;
}
