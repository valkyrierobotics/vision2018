#include "utils/mjpgStream.hpp"
#include <iostream>

void mjpgStream(std::string& outFile, cv::Mat& img)
{
    // Video writing for mjpgStreamer
    bool isOutputColored = true;
    int imgSizeX = 640;
    int imgSizeY = 480;
    int stream_fps = 30;

    cv::VideoWriter os (outFile.c_str(), CV_FOURCC('M', 'J', 'P', 'G'), stream_fps, cv::Size(imgSizeX, imgSizeY), isOutputColored);

    if (os.isOpened())
        os.write(img);
    else
        std::cerr << "ERROR - Could not write to " << outFile << std::endl;
}

void mjpgStream(cv::Mat& img)
{
    std::string s = "images/mjpgs/main.mjpeg";
    mjpgStream(s, img);
}
