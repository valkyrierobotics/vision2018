#include <iostream>

#include "utils/getBoundedRects.hpp"
#include "utils/vector_ordering.hpp"

// TODO: change this method to get the vector passed in by reference
::std::vector<::cv::RotatedRect> getBoundedRects(::std::vector< ::std::vector<::cv::Point> >& contours)
{
	::std::vector<::cv::RotatedRect> boundedRects (contours.size());

	// Get the minimal area bounded rects
	for (size_t i = 0; i < contours.size(); ++i)
		boundedRects[i] = ::cv::minAreaRect(contours[i]);

	return boundedRects;
}

::cv::RotatedRect mergedBoundedRect(::std::vector < ::std::vector<::cv::Point> >& contours)
{
    size_t numContours = 3;
    // Merge up to three contours only for now
    if (contours.size() > numContours)
        ::std::cout << "WARNING - Unsupported number (" << contours.size() 
            << ") of contours detected\n";
    else
        numContours = contours.size();

    std::vector<::cv::Point> mergedContour;

    // Pull top N contours
    for (size_t i = 0; i < numContours; ++i)
        mergedContour += contours[i];

    return ::cv::minAreaRect(mergedContour);
}
