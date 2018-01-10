#ifndef LAPLACIAN_SHARPEN_WINDOWS_HPP
#define LAPLACIAN_SHARPEN_WINDOWS_HPP

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "laplacianSharpen.hpp"

void laplacianSharpenWindows(cv::Mat &img, int &ksize, int &scale, int &delta, int &apply, bool visible, const bool isStreaming);

#endif // LAPLACIAN_SHARPEN_WINDOWS_HPP
