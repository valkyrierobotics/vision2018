#include "filters/shapeThresholds.hpp"

#include <iostream>

void areaRatioThreshold (std::vector<std::vector<cv::Point> >& contours, std::vector<cv::RotatedRect>& boundedRects, int minArea, int maxArea, double areaRatio, double areaRatioMaxDeviation)
{
    for (size_t i = 0; i < boundedRects.size(); ++i)
    {
        double boundedRectsArea = boundedRects[i].size.height * boundedRects[i].size.width;
        double contourArea = cv::contourArea(contours[i]);

        // Determine if the bounded boundedRects area is correct and
        // if the ratio between the contour area and rect's area is correct
        if (boundedRectsArea < minArea || 
            boundedRectsArea > maxArea || 
            std::abs((contourArea / boundedRectsArea) - areaRatio) > areaRatioMaxDeviation)
        {
            contours.erase(contours.begin() + i);
            boundedRects.erase(boundedRects.begin() + i);
            // Size of vector got smaller
            i--;
        }
    }
}

void sideRatioThreshold (std::vector<std::vector<cv::Point> >& contours, std::vector<cv::RotatedRect>& boundedRects, double sideRatio, double sideRatioMaxDeviation)
{
    for (size_t i = 0; i < boundedRects.size(); ++i)
    {
        double height = boundedRects[i].size.height;
        double width = boundedRects[i].size.width;

        // Swap height and width so that height is always longest
        if (height < width)
        {
            double tmp = height;
            height = width;
            width = tmp;
        }

        // Determine if the ratio between the height and width is correct
        if (std::abs((height / width) - sideRatio) > sideRatioMaxDeviation)
        {
            contours.erase(contours.begin() + i);
            boundedRects.erase(boundedRects.begin() + i);
            // Size of vector got smaller
            i--;
        }
    }
}

void angleThreshold (std::vector<std::vector<cv::Point> >& contours, std::vector<cv::RotatedRect>& boundedRects, int angleMaxDeviation)
{
    for (size_t i = 0; i < boundedRects.size(); ++i)
    {
        double height = boundedRects[i].size.height;
        double width = boundedRects[i].size.width;

        // Determine the orientation based off of the longer side and 
        // correct it by subtracting 90 if the rect's angle is on its side
        if ((width < height && (std::abs(boundedRects[i].angle) > angleMaxDeviation)) || 
            (width > height && (std::abs(std::abs(boundedRects[i].angle) - 90) > angleMaxDeviation)))
        {
            contours.erase(contours.begin() + i);
            boundedRects.erase(boundedRects.begin() + i);
            // Size of vector got smaller
            i--;
        }
    }
}

// Threshold for a square 'U' shape
void uShapeThreshold (std::vector< std::vector<cv::Point> >& contours, std::vector<cv::RotatedRect>& boundedRects, int minDist, int maxDist)
{
    for (size_t i = 0; i < contours.size(); ++i)
    {
        cv::Moments mu = cv::moments(contours[i], false);
        // Negative distance because outside of contour area, so change back to positive
        double dist = -1 * cv::pointPolygonTest(contours[i], cv::Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00), true);
        // Remove if center of mass is not the correct distance away from the contour area
        if (dist < minDist || dist > maxDist)
        {
            contours.erase(contours.begin() + i);
            boundedRects.erase(boundedRects.begin() + i);
            // Vector got smaller, decrement loop counter
            i--;
        }
    }
}

