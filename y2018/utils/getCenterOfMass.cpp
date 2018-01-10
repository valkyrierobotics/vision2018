#include "utils/getCenterOfMass.hpp"
#include <iostream>

cv::Point getCenterOfMass(std::vector<cv::Point>& contour)
{
    if (contour.size() <= 0)
    {
        std::cerr << "No points in contour to get center of mass" << std::endl;
        return cv::Point();
    }

    cv::Moments mu = cv::moments(contour, false);

    // Find center of mass from the contour vector
    return cv::Point(mu.m10 / mu.m00, mu.m01 / mu.m00);
}
