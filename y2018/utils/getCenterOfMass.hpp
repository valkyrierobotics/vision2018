#ifndef GET_CENTER_OF_MASS_HPP
#define GET_CENTER_OF_MASS_HPP

#include <opencv2/imgproc/imgproc.hpp>

#include <vector>
#include <exception>

cv::Point getCenterOfMass(std::vector<cv::Point>& contour);

#endif // GET_CENTER_OF_MASS
