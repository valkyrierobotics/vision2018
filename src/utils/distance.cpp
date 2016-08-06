#include "utils/distance.hpp"

// Returns the minimum distance (straight line) between two points
double distance(cv::Point one, cv::Point two)
{
    return std::sqrt(std::pow(one.x - two.x, 2) + std::pow(one.y - two.y, 2));
}

double getShortestDistance (cv::Point2f rectPoints[4], double focalLen, int dist, int height)
{
	// The target's base is 20 inches wide in real life
	int realWidth = 20;
	int pixelWidth = 0;
	double theta = 0;
	double d = 0;

	int length = static_cast<int>(distance (rectPoints[0], rectPoints[3]));
	int width = static_cast<int>(distance (rectPoints[0], rectPoints[1]));
	
	if (length > width)
		pixelWidth = length;
	else
		pixelWidth = width;
		
	d = (realWidth * focalLen) / pixelWidth;
	return d;
}
