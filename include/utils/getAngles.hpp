#ifndef ANGLES_HPP
#define ANGLES_HPP

#include <opencv2/imgproc/imgproc.hpp>

#include <vector>

const double PI = 3.14159265;

double getYaw(const int SCREEN_WIDTH, double hypotenuse, double widthInInches, std::vector<cv::Point>& corners, cv::Point2f& mc);
double getPitch(double height, double hypotenuse);

#endif // ANGLES_HPP

