/**
 * @function cornerSubPix_Demo.cpp
 * @brief Demo code for refining corner locations
 * @author OpenCV team
 */

// #include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>

using namespace cv;
using namespace std;

/// Global variables
// Mat src, gray;
//
// int maxCorners = 10;
// int maxTrackbar = 25;
//
Mat gray;
RNG rng_(12345);
// const char* source_window = "Image";

/// Function header
// void goodFeaturesToTrack_Demo( int, void* );
void goodFeaturesToTrack_Demo(cv::Mat& src, vector<Point2f>& corners, int maxCorners);

/**
 * @function main
 */
// int main( int, char** argv )
// {
//   VideoCapture cap (0);
//   while (true)
//   {
//     cap >> src;
//     /// Load source image and convert it to gray
//     // src = imread( argv[1], IMREAD_COLOR );
//     cvtColor( src, gray, COLOR_BGR2GRAY );
//
//     /// Create Window
//     namedWindow( source_window, WINDOW_AUTOSIZE );
//
//     /// Create Trackbar to set the number of corners
//     // createTrackbar( "Max  corners:", source_window, &maxCorners, maxTrackbar, goodFeaturesToTrack_Demo );
//
//     imshow( source_window, src );
//
//     // goodFeaturesToTrack_Demo( 0, 0 );
//
//     waitKey(10);
//   }
//   return 0;
// }

/**
 * @function goodFeaturesToTrack_Demo.cpp
 * @brief Apply Shi-Tomasi corner detector
 */
void goodFeaturesToTrack_Demo(cv::Mat& src, vector<Point2f>& corners, int maxCorners)
{
  if( maxCorners < 1 ) { maxCorners = 1; }

  /// Parameters for Shi-Tomasi algorithm
  // vector<Point2f> corners;
  double qualityLevel = 0.01;
  double minDistance = 10;
  int blockSize = 3;
  bool useHarrisDetector = false;
  double k = 0.04;

  /// Copy the source image
  Mat copy;
  copy = src.clone();

  /// Apply corner detection
  goodFeaturesToTrack(src,
               corners,
               maxCorners,
               qualityLevel,
               minDistance,
               Mat(),
               blockSize,
               useHarrisDetector,
               k );


  /// Draw corners detected
  cout<<"** Number of corners detected: "<<corners.size()<<endl;
  if (corners.size() == 0) return;
  int r = 4;
  for( size_t i = 0; i < corners.size(); i++ )
     { circle( copy, corners[i], r, Scalar(rng_.uniform(0,255), rng_.uniform(0,255), rng_.uniform(0,255)), -1, 8, 0 ); }

  /// Show what you got
  namedWindow( "Subpix", WINDOW_AUTOSIZE );
  imshow( "Subpix", copy );

  /// Set the neeed parameters to find the refined corners
  Size winSize = Size( 5, 5 );
  Size zeroZone = Size( -1, -1 );
  TermCriteria criteria = TermCriteria( TermCriteria::EPS + TermCriteria::COUNT, 40, 0.001 );

  /// Calculate the refined corner locations
  cornerSubPix( src, corners, winSize, zeroZone, criteria );

  /// Write them down
  for( size_t i = 0; i < corners.size(); i++ )
     { cout<<" -- Refined Corner ["<<i<<"]  ("<<corners[i].x<<","<<corners[i].y<<")"<<endl; }
}

