#ifndef MJPG_isStreaming_HPP
#define MJPG_isStreaming_HPP

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

void mjpgStream(std::string& outFile, cv::Mat& img);
void mjpgStream(cv::Mat& img);

#endif // MJPG_isStreaming_HPP
