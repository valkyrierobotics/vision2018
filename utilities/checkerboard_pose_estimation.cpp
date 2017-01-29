//Title:  coordinate_system.cpp
//Author: Nicholas Ballard

#include <stdio.h>
#include <string>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui.hpp>
#include <iostream>
#include <iomanip>

using namespace cv;
using namespace std;



// Globals ----------------------------------------------------------------------------------------

int boardHeight = 6;
int boardWidth = 9;
Size cbSize = Size(boardHeight,boardWidth);

string filename = "out_camera_data.xml";

bool doneYet = false;

//default image size
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;
const float PI = 3.141592;

//function prototypes
//void generate_boardPoints();

// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(Mat &R)
{
    Mat Rt;
    transpose(R, Rt);
    Mat shouldBeIdentity = Rt * R;
    Mat I = Mat::eye(3,3, shouldBeIdentity.type());
     
    return  norm(I, shouldBeIdentity) < 1e-6;
}
 
float toDeg (float rad)
{
    return rad * 180 / PI;
}

// Calculates rotation matrix to euler angles
// The order is (pitch, yaw, roll)
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
Vec3f rotationMatrixToEulerAngles(Mat &R)
{
    assert(isRotationMatrix(R));

    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
    bool singular = sy < 1e-6; // If
    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    return Vec3f(toDeg(x), toDeg(y), toDeg(z));
    // return Vec3f(x, y, z);
}


// Main -------------------------------------------------------------------------------------------
int main(int argc, char* argv[])
{
	
	//set up a FileStorage object to read camera params from file
	FileStorage fs;
    if (argc > 1) filename = argv[1];
	fs.open(filename, FileStorage::READ);
	// read camera matrix and distortion coefficients from file
	Mat intrinsics, distortion;
	fs["Camera_Matrix"] >> intrinsics;
	fs["Distortion_Coefficients"] >> distortion;
	// close the input file
	fs.release();




	//set up matrices for storage
	Mat webcamImage, gray, one;
	// Mat rvec = Mat(Size(3,1), CV_64F);
	// Mat tvec = Mat(Size(3,1), CV_64F);
    Mat rvec, tvec;

	//setup vectors to hold the chessboard corners in the chessboard coordinate system and in the image
	vector<Point2f> imagePoints, imageFramePoints, imageOrigin;
	vector<Point3f> boardPoints, framePoints;


	//generate vectors for the points on the chessboard
	for (int i = -boardWidth / 2.0; i < boardWidth / 2.0; i++)
	{
		for (int j=-boardHeight / 2.0; j < boardHeight / 2.0; j++)
		{
			boardPoints.push_back( Point3f( double(i), double(j), 0.0) );
		}
	}
	//generate points in the reference frame
	framePoints.push_back( Point3f( 0.0, 0.0, 0.0 ) );
	framePoints.push_back( Point3f( 3.0, 0.0, 0.0 ) );
	framePoints.push_back( Point3f( 0.0, 3.0, 0.0 ) );
	framePoints.push_back( Point3f( 0.0, 0.0, 3.0 ) );


	//set up VideoCapture object to acquire the webcam feed from location 0 (default webcam location)
	VideoCapture capture;
	capture.open(0);
	//set the capture frame size
	capture.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);

    char kill = ' ';
	while(kill != 'q')
	{
		 //store image to matrix
		 // capture.read(webcamImage);
         capture >> webcamImage;

		 //make a gray copy of the webcam image
		 cvtColor(webcamImage,gray,COLOR_BGR2GRAY);


		 //detect chessboard corners
		 bool found = findChessboardCorners(gray, cbSize, imagePoints, CALIB_CB_FAST_CHECK);
         // if (found) cout << imagePoints << "\n";
		 drawChessboardCorners(webcamImage, cv::Size(boardHeight, boardWidth), imagePoints, found);

		 //find camera orientation if the chessboard corners have been found
		 if ( found )
		 {
             // imagePoints will store the pixel coordinates of the board's squares
			 solvePnP( boardPoints, imagePoints, intrinsics, distortion, rvec, tvec, false );

			 // Project the axes onto the image
			 projectPoints(framePoints, rvec, tvec, intrinsics, distortion, imageFramePoints );

             cv::Mat rmat, tmat;

             // Convert the vectors to matrices
             cv::Rodrigues(rvec, rvec);
             cv::Rodrigues(tvec, tmat);

			 cv::Vec3f rotAngles = rotationMatrixToEulerAngles(rvec);

			 // cout << rotAngles << "\n";

             float phi = toDeg(atan(tvec.at<double>(0, 0)/tvec.at<double>(2, 0)));
             char str[30];

             sprintf(str, "(%.2f, %.2f, %.2f)", rotAngles[0], rotAngles[1], rotAngles[2]);
             cv::putText(webcamImage, str, cv::Point(10, 430), CV_FONT_HERSHEY_COMPLEX_SMALL, 0.75, cv::Scalar(0, 255, 0), 1, 8, false);
             sprintf(str, "Phi: %.2f", phi);
             cv::putText(webcamImage, str, cv::Point(10, 450), CV_FONT_HERSHEY_COMPLEX_SMALL, 0.75, cv::Scalar(0, 255, 0), 1, 8, false);
             sprintf(str, "Theta - Phi: %.2f", rotAngles[1] - phi);
             cv::putText(webcamImage, str, cv::Point(10, 470), CV_FONT_HERSHEY_COMPLEX_SMALL, 0.75, cv::Scalar(0, 255, 0), 1, 8, false);

			 //DRAWING
			 // Draw the origin of the axes on the image
			 circle(webcamImage, imageFramePoints[0], 4 ,CV_RGB(255,0,0) );
			 
			 // Point one, two, three;
			 // one.x=10; one.y=10;
			 // two.x = 60; two.y = 10;
			 // three.x = 10; three.y = 60;
             //
			 // line(webcamImage, one, two, CV_RGB(255,0,0) );
			 // line(webcamImage, one, three, CV_RGB(0,255,0) );
             //
             // Draw the projected axes
			 line(webcamImage, imageFramePoints[0], imageFramePoints[1], CV_RGB(255,0,0), 2 );
			 line(webcamImage, imageFramePoints[0], imageFramePoints[2], CV_RGB(0,255,0), 2 );
			 line(webcamImage, imageFramePoints[0], imageFramePoints[3], CV_RGB(0,0,255), 2 );

			 //show the pose estimation data
             /*
			 cout << fixed << setprecision(2) << "rvec = ["
				  << rvec.at<double>(0,0) << ", "
				  << rvec.at<double>(1,0) << ", "
				  << rvec.at<double>(2,0) << "] \t" << "tvec = ["
				  << tvec.at<double>(0,0) << ", "
				  << tvec.at<double>(1,0) << ", "
				  << tvec.at<double>(2,0) << "]" << endl;
                */
			
		 }

		 //show the image on screen
		 namedWindow("OpenCV Webcam", cv::WINDOW_AUTOSIZE);
		 imshow("OpenCV Webcam", webcamImage);

		 kill = waitKey(10);
	}

	return 0;
}
