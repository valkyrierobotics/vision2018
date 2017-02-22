#include "utils/getCenterOfMass.hpp"

cv::Point getCenterOfMass(std::vector<cv::Point>& contour)
{
    if (contour.size() <= 0)
        throw std::runtime_error("No points in contour to get center of mass\n");

    cv::Moments mu = cv::moments(contour, false);

    // Find center of mass from the contour vector
    return cv::Point(mu.m10 / mu.m00, mu.m01 / mu.m00);
}
