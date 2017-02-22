#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <string>
#include <iostream>

using namespace std;

void onMouse(int evt, int x, int y, int flags, void* param) 
{
    if (evt == CV_EVENT_LBUTTONDOWN) 
	{
        std::vector<cv::Point3f>* ptPtr = (std::vector<cv::Point3f>*)param;
        ptPtr->push_back(cv::Point3f(x,y, 0)); // Z axis is empty because the target is flat
    }
}

// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(cv::Mat &R)
{
    cv::Mat Rt;
    cv::transpose(R, Rt);
    cv::Mat shouldBeIdentity = Rt * R;
    cv::Mat I = cv::Mat::eye(3,3, shouldBeIdentity.type());

    return  cv::norm(I, shouldBeIdentity) < 1e-6;
}

int main(int argc, char** argv)
{
    if (argc < 3) 
    {
        cout << "INCORRECT USAGE (TWO IMAGE FILE NAMES)\n";
        return -1;
    }

    std::string image1 = argv[1];
    std::string image2 = argv[2];

    cv::Mat img1, img2;
    img1 = cv::imread(image1);
    img2 = cv::imread(image2);

	std::vector<cv::Point3f> points;
	std::vector<cv::Point3f> saved_points;
	cv::namedWindow("Image Clicker");
	cv::setMouseCallback("Image Clicker", onMouse, (void*)&points);

	int X = 0, Y = 0;

    char c = ' ';
    while (c != 'q')
    {
		cv::imshow ("Image Clicker", img1);
        // cv::imshow("Image 1", img1);
        cv::imshow("Image 2", img2);

        if (points.size() > 0)
        {
            int count = 0;
            for (auto it = points.begin(); it != points.end(); ++it)
            {
                count++;
                X = (*it).x;
                Y = (*it).y;

                cv::circle(img1, cv::Point(X, Y), 3, cv::Scalar(0,255,0), -1);

                cout << "Point " << count << " (X, Y): (" << X << ", " << Y << ")\n";
                saved_points.push_back(*it);
                points.erase(it);
                --it;
            }
        }
        c = cv::waitKey(10);
    }

    cout << "\nThe following points were saved\n\n";
    int count = 0;
    for (auto it = saved_points.begin(); it != saved_points.end(); ++it)
    {
        count++;
        X = (*it).x;
        Y = (*it).y;

        cv::circle(img1, cv::Point(X, Y), 3, cv::Scalar(0,255,0), -1);

        cout << "Point " << count << " (X, Y): (" << X << ", " << Y << ")\n";
    }

	if (saved_points.size() >= 8)
	{
		// auto beg = saved_points.begin();
		// cv::Mat transform = cv::getPerspectiveTransform(std::vector<cv::Point3f>(beg, (beg + 4)), std::vector<cv::Point2f>((beg + 4), (beg + 8)));
        
        // cv::Mat distortion = cv::Mat::zeros(4, 1, CV_64FC1);
        // cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
        // cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);
        // bool useExtrinsicGuess = false;
		// cv::Mat transform = cv::solvePnP(std::vector<cv::Point3f>(beg, (beg + 4)), std::vector<cv::Point2f>((beg + 4), (beg + 8)), cam_intrins, distortion, rvec, tvec, useExtrinsicGuess, CV_EPNP);
        
		// cout << isRotationMatrix(transform) << "\n";
		// cout << transform << "\n";	
	}

    return 0;
}
