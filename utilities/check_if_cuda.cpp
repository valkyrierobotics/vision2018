#include <opencv2/gpu/gpu.hpp>
#include <iostream>

int main()
{
    if (cv::gpu::getCudaEnabledDeviceCount() == 0)
        std::cout << "No CUDA enabled devices available, CUDA is not being used" 
            << std::endl;
    else
        std::cout << "Number of CUDA enabled devices: " 
            << cv::gpu::getCudaEnabledDeviceCount()
            << std::endl;
    return 0;
}
