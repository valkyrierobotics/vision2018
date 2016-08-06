#include "filters/sideAngleAreaThreshold.hpp"

// Probably split this into three thresholds
int sideAngleAreaThreshold (std::vector<std::vector<cv::Point> >& contours, std::vector<cv::RotatedRect>& rect, double sideRatio, double areaRatio, double minArea, double maxArea, double sideThreshold, double areaThreshold, double angleThreshold)
{
    for (int i = 0; i < rect.size(); i++)
    {
        double height = rect[i].size.height;
        double width = rect[i].size.width;
        double rectArea = height * width;
        double contourArea = cv::contourArea(contours[i]);

        // If any of the following are true, the shape is not the goal
        if (std::abs((height / width) - sideRatio) > sideThreshold || 
            std::abs((contourArea / rectArea) - areaRatio) > areaThreshold || 
            rectArea < minArea || 
            rectArea > maxArea || 
            (width > height && (std::abs(rect[i].angle) > angleThreshold)) || 
            (width < height && (std::abs(std::abs(rect[i].angle) - 90) > angleThreshold)))
        {
            contours.erase(contours.begin() + i);
            rect.erase(rect.begin() + i);
            // Size of vector got smaller
            i--;
        }
        else
        {
            goalInd = i;
        }
    }
    return goalInd;
}