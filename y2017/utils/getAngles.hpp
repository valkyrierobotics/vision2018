#ifndef ANGLES_HPP
#define ANGLES_HPP

#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <math.h>

double atan_wrt_x_axis(double y, double x);

// Returns the angle between two points with respect to the x axis,
// from 0 to 360 degrees.
// Note that OpenCV has the origin at the top left corner
template <class T1, class T2>
double angle_wrt_x_axis(T1 a, T2 b)
{
  return atan_wrt_x_axis(a.y - b.y, a.x - b.x);
}

double getYaw(const int SCREEN_WIDTH, double hypotenuse, double widthInInches, std::vector<cv::Point>& corners, cv::Point& mc);
double getPitch(double height, double hypotenuse);

#endif // ANGLES_HPP

