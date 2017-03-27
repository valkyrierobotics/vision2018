#include "utils/mjpgStream.hpp"

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
        throw std::runtime_error(std::string("Error: Could not write to ") + outFile);
}

void mjpgStream(cv::Mat& img)
{
    std::string s = "images/mjpgs/main.mjpeg";
    mjpgStream(s, img);
}
