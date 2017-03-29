#ifndef GAUSSIAN_BLUR_HPP
#define GAUSSIAN_BLUR_HPP

#include <opencv2/imgproc/imgproc.hpp>

void gaussianBlur(cv::Mat &img, int blur_ksize, int sigmaX, int sigmaY); 

#endif // GAUSSIAN_BLUR_HPP  
