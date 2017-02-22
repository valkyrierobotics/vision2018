#include "utils/getAngles.hpp"
#include <math.h>

double getYaw(const int SCREEN_WIDTH, double hypotenuse, double widthInInches, std::vector<cv::Point>& corners, cv::Point& mc)
{
    // Take distance from bottom left to bottom right corners
	double widthInPixels = std::abs(corners[0].x - corners[3].x);
	double center = SCREEN_WIDTH / 2;
	double distFromCenter = mc.x - center;
	double inchesPerPixel = widthInInches / widthInPixels;
	double horizDeviation = distFromCenter * inchesPerPixel;

	return std::asin(horizDeviation / hypotenuse) * 180 / M_PI;
}

double getPitch(double height, double hypotenuse)
{
	return std::asin(height / hypotenuse) * 180 / M_PI;
}

// Returns the atan(y/x) with respect to the x axis, from 0 to 360 degrees
double atan_wrt_x_axis(double y, double x)
{
  return fmod(((atan2(y, x) * 180 / M_PI) + 360), 360);
}
