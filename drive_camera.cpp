#include <opencv2/highgui/highgui.hpp>
#include <iostream>

#include "y2017/utils/mjpgStream.hpp"

const int CAM_ID = 0;

int main ()
{
    ::std::cout << "Opening camera at " << CAM_ID << "\n";

    cv::VideoCapture cam (CAM_ID);
    cv::Mat img;
    while (true)
    {
        cam >> img;

        // Stream with mjpg_streamer command
        cv::flip(img, img, 0);
        mjpgStream(img);
        // cv::waitKey(10);
    }

    return 0;
}
