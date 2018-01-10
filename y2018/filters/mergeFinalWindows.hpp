#ifndef MERGE_FINAL_WINDOWS_HPP 
#define MERGE_FINAL_WINDOWS_HPP

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "mergeFinal.hpp"

void mergeFinalWindows(cv::Mat& img1, cv::Mat& img2, int& weight1, int& weight2, int& apply, bool visible, const bool isStreaming);

#endif // MERGE_FINAL_WINDOWS_HPP
