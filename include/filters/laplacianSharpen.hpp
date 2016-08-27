#ifndef LAPACIAN_SHARPEN_HPP
#define LAPACIAN_SHARPEN_HPP

#include <opencv2/imgproc/imgproc.hpp>

void laplacianSharpen(cv::Mat &img, int ksize, int scale, int delta);

#endif // LAPLACIAN_SHARPEN_HPP
