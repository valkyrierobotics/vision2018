#include "utils/getAngles.hpp"

double getYaw(const int SCREEN_WIDTH, double hypotenuse, double widthInInches, std::vector<cv::Point>& corners, cv::Point2f& mc)
{
    // Take distance from bottom left to bottom right corners
	double widthInPixels = std::abs(corners[0].x - corners[3].x);
	double center = SCREEN_WIDTH / 2;
	double distFromCenter = mc.x - center;
	double inchesPerPixel = widthInInches / widthInPixels;
	double horizDeviation = distFromCenter * inchesPerPixel;

	return std::asin(horizDeviation / hypotenuse) * 180 / PI;
}

double getPitch(double height, double hypotenuse)
{
	return std::asin(height / hypotenuse) * 180 / PI;
}
