#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

#include <string>
#include <stdio.h>
#include <thread>
#include <chrono>
#include <vector>
#include <fstream>

#include "y2017/common/constants.hpp"

#include "y2017/utils/getAngles.hpp"
#include "y2017/utils/distance.hpp"
#include "y2017/utils/getBoundedRects.hpp"
#include "y2017/utils/getCenterOfMass.hpp"
#include "y2017/utils/getContours.hpp"
#include "y2017/utils/getCorners.hpp"
#include "y2017/utils/netThread.hpp"
#include "y2017/utils/udpClientServer.hpp"
#include "y2017/utils/mjpgStream.hpp"
#include "y2017/utils/gui.hpp"
#include "y2017/utils/enumCvType.hpp"

#include "y2017/filters/selectMode.hpp"
#include "y2017/filters/cannyEdgeDetect.hpp"
#include "y2017/filters/dilateErode.hpp"
#include "y2017/filters/gaussianBlur.hpp"
#include "y2017/filters/houghLines.hpp"
#include "y2017/filters/houghCircles.hpp"
#include "y2017/filters/hsvColorThreshold.hpp"
#include "y2017/filters/laplacianSharpen.hpp"
#include "y2017/filters/mergeFinal.hpp"
#include "y2017/filters/shapeThresholds.hpp"

#include "y2017/filters/cannyEdgeDetectWindows.hpp"
#include "y2017/filters/dilateErodeWindows.hpp"
#include "y2017/filters/gaussianBlurWindows.hpp"
#include "y2017/filters/houghLinesWindows.hpp"
#include "y2017/filters/houghCirclesWindows.hpp"
#include "y2017/filters/hsvColorThresholdWindows.hpp"
#include "y2017/filters/laplacianSharpenWindows.hpp"
#include "y2017/filters/mergeFinalWindows.hpp"
#include "y2017/filters/shapeThresholdsWindows.hpp"

#include "y2017/vision_data.pb.h"

#include "aos/udp.h"

namespace chrono = ::std::chrono;

namespace aos {

class monotonic_clock {
  public:
    typedef ::std::chrono::nanoseconds::rep rep;
    typedef ::std::chrono::nanoseconds::period period;
    typedef ::std::chrono::nanoseconds duration;
    typedef ::std::chrono::time_point<monotonic_clock> time_point;

    static monotonic_clock::time_point now() noexcept {
      struct timespec current_time;
      if (clock_gettime(CLOCK_MONOTONIC, &current_time) != 0) {
        printf("clock_gettime(%jd, %p) failed",
            static_cast<uintmax_t>(CLOCK_MONOTONIC), &current_time);
      }
      // const chrono::nanoseconds offset =
      //   (&global_core == nullptr || global_core == nullptr ||
      //    global_core->mem_struct == nullptr)
      //   ? chrono::nanoseconds(0)
      //   : chrono::nanoseconds(global_core->mem_struct->time_offset);
      const chrono::nanoseconds offset = chrono::nanoseconds(0);

      return time_point(::std::chrono::seconds(current_time.tv_sec) +
          ::std::chrono::nanoseconds(current_time.tv_nsec)) + offset;
    }

    static constexpr bool is_steady = true;

    // Returns the epoch (0).
    static constexpr monotonic_clock::time_point epoch() {
      return time_point(zero());
    }

    static constexpr monotonic_clock::duration zero() { return duration(0); }

    static constexpr time_point min_time{
      time_point(duration(::std::numeric_limits<duration::rep>::min()))};
};

}; // namespace aos

// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(cv::Mat& R)
{
  cv::Mat Rt;
  transpose(R, Rt);
  cv::Mat shouldBeIdentity = Rt * R;
  cv::Mat I = cv::Mat::eye(3,3, shouldBeIdentity.type());

  return  norm(I, shouldBeIdentity) < 1e-6;
}

float toDeg (float rad)
{
  return rad * 180 / M_PI;
}

// Calculates rotation matrix to euler angles
// The order is (pitch, yaw, roll)
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
cv::Vec3f rotationMatrixToEulerAngles(cv::Mat& R)
{
  if (!isRotationMatrix(R))
  {
    ::std::cout << "NOT A ROTATION MATRIX\n";
    return cv::Vec3f (-1.0, -1.0, -1.0);
  }

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
  return cv::Vec3f(toDeg(x), toDeg(y), toDeg(z));
  // return Vec3f(x, y, z);
}

bool containsNaN(cv::Mat& m)
{
  for (int j = 0; j < m.rows; ++j)
  {
    for (int k = 0; k < m.cols; ++k)
    {
      if (cvIsNaN(m.at<double>(j, k)) || cvIsInf(m.at<double>(j, k))) return true;
    }
  }
  return false;
}

int getLeftMostContourIndex(::std::vector< ::std::vector<cv::Point> >& contours)
{
  double minDistance = camera::SCREEN_WIDTH, ind = 0, curr = 0;
  cv::Point leftMost = cv::Point(0, camera::SCREEN_HEIGHT/2);
  for (size_t i = 0; i < contours.size(); ++i)
  {
    cv::Point mc = getCenterOfMass(contours[i]);
    curr = distance(leftMost, mc);
    if (curr < minDistance)
    {
      minDistance = curr;
      ind = i;
    }
  }
  return ind;
}

  template <typename T>
cv::Point cvtPoint(const T& src)
{
  cv::Point dest = src;
  return dest;
}

// Converts a vector of vectors of cv::Point_<T> to cv::Point_<int>
  template <typename T>
void cvtPoint(const ::std::vector< ::std::vector<T> >& src, ::std::vector< ::std::vector<cv::Point> >& dest)
{
  for (size_t j = 0; j < src.size(); ++j)
    for (size_t k = 0; k < src.size(); ++k)
      dest[j].push_back(src[k]);
}
// Converts a vector of cv::Point_<int> to cv::Point_<T>
  template <typename T>
void cvPointTo(const ::std::vector<cv::Point>& src, ::std::vector<T>& dest)
{
  for (size_t i = 0; i < src.size(); ++i)
    dest.push_back(src[i]);
}

  template <typename T1, typename T2, size_t N>
void cvtPoint(T1 (&src)[N], T2 (&dest)[N])
{
  for (size_t i = 0; i < N; ++i)
    dest[i] = src[i];
}

int solvePnPFlag = 0;
int useExtrinsicGuess = 0;
int useRansac = 0;

// Precondition: input image should be grayscale and corners should be a rect
// Return a vector of the angular displacement of the camera from the target
// in degrees from -180 < theta < 180.
// Return (-1.0, -1.0, -1.0) on failure
cv::Vec3f getAngularPosition(cv::Mat& img,
    cv::Mat& rvec, cv::Mat& tvec,
    ::std::vector<cv::Point2f>& corners,
    cv::Mat& intrinsics, cv::Mat& distortion,
    camera::GlobalCoordinates& coords,
    const ::std::string& CAMERA_CONFIG_FILE,
    double targetWidth, double targetHeight)
{
  if (corners.size() != 4) return cv::Vec3f (-1.0, -1.0, -1.0);

  ::std::vector<cv::Point2f> projectedAxesPoints;
  ::std::vector<cv::Point3f> targetPoints, axesPoints;

  axesPoints.push_back(cv::Point3f(0.0, 0.0, 0.0));
  axesPoints.push_back(cv::Point3f(3.0, 0.0, 0.0));
  axesPoints.push_back(cv::Point3f(0.0, 3.0, 0.0));
  axesPoints.push_back(cv::Point3f(0.0, 0.0, 3.0));

  // Bottom left, top left, top right, bottom right
  targetPoints.push_back(cv::Point3f(-targetWidth / 2, -targetHeight / 2, 0.0));
  targetPoints.push_back(cv::Point3f(-targetWidth / 2,  targetHeight / 2, 0.0));
  targetPoints.push_back(cv::Point3f( targetWidth / 2,  targetHeight / 2, 0.0));
  targetPoints.push_back(cv::Point3f( targetWidth / 2, -targetHeight / 2, 0.0));

  cv::namedWindow("SolvePnP");
  cv::createTrackbar("Normal/RANSAC", "SolvePnP", &useRansac, 1);
  cv::createTrackbar("Use Extrinsic Guess", "SolvePnP", &useExtrinsicGuess, 1);
  cv::createTrackbar("ITERATIVE/P3P/EPNP", "SolvePnP", &solvePnPFlag, 2);
  int flag = 0;
  switch (solvePnPFlag)
  {
    case 0: flag = CV_ITERATIVE; break;
    case 1: flag = CV_P3P; break;
    case 2: flag = CV_EPNP; break;
  }

  if (useRansac)
    solvePnPRansac(targetPoints, corners, intrinsics, distortion, rvec, tvec, useExtrinsicGuess, 100, flag);
  else
    solvePnP(targetPoints, corners, intrinsics, distortion, rvec, tvec, useExtrinsicGuess, flag);

  if (containsNaN(rvec) || containsNaN(tvec)) 
  {
    ::std::cout << "NaN in rvec or tvec" << "\n"; 
    return cv::Vec3f (-1.0, -1.0, -1.0);
  }

  // Project the axes onto the image
  cv::projectPoints(axesPoints, rvec, tvec, intrinsics, distortion, projectedAxesPoints);

  cv::Mat rmat, tmat;
  cv::Rodrigues(rvec, rmat); // Change the vectors to matrices

  cv::Mat camTransVec = -rmat.t() * tvec;
  // double r = ::std::sqrt(::std::pow(camTransVec.at<double>(0, 0), 2) + ::std::pow(camTransVec.at<double>(0, 1), 2) + ::std::pow(camTransVec.at<double>(0, 2), 2));

  coords.x = camTransVec.at<double>(0, 0);
  coords.y = camTransVec.at<double>(0, 1);
  coords.z = camTransVec.at<double>(0, 2);
  coords.euclidDist = ::std::sqrt(::std::pow(coords.x, 2) + ::std::pow(coords.y, 2) + ::std::pow(coords.z, 2));
  coords.theta = toDeg(atan(coords.x / coords.z));

#if CALIB
  // Put the angle measurements on the image
  cv::Mat statsImg = cv::Mat::zeros(img.size(), CV_8UC3);
  char str [60];
  sprintf(str, "Euclidean Distance (%8.2f)", coords.euclidDist);
  cv::putText(statsImg, str, cv::Point(10, 20), CV_FONT_HERSHEY_COMPLEX_SMALL, 0.75, cv::Scalar(255, 0, 200), 1, 8, false);
  sprintf(str, "Displacement (X, Y, Z) (%8.2f, %8.2f, %8.2f)", camTransVec.at<double>(0, 0), camTransVec.at<double>(0, 1), camTransVec.at<double>(0, 2));
  cv::putText(statsImg, str, cv::Point(10, 40), CV_FONT_HERSHEY_COMPLEX_SMALL, 0.75, cv::Scalar(255, 0, 200), 1, 8, false);
  sprintf(str, "Theta: %.2f", coords.theta);
  cv::putText(statsImg, str, cv::Point(10, 60), CV_FONT_HERSHEY_COMPLEX_SMALL, 0.75, cv::Scalar(255, 0, 200), 1, 8, false);
  cv::imshow("Stats", statsImg);

  // Draw the axes of rotation at the bottom left corner
  cv::circle(img, projectedAxesPoints[0], 4, cv::Scalar(255, 0, 0), -1);
  cv::line(img, projectedAxesPoints[0], projectedAxesPoints[1], cv::Scalar(0, 0, 255), 2);
  cv::line(img, projectedAxesPoints[0], projectedAxesPoints[2], cv::Scalar(0, 255, 0), 2);
  cv::line(img, projectedAxesPoints[0], projectedAxesPoints[3], cv::Scalar(255, 0, 0), 2);
#endif

  return cv::Vec3f(camTransVec.at<double>(0, 0), camTransVec.at<double>(0, 1), camTransVec.at<double>(0, 2));
  // cv::Vec3f rotAngles;
  //
  // rotAngles[0] = -x; // Negative because counter clockwise is positive by pose
  // rotAngles[1] = z;
  // rotAngles[2] = theta;
  //
  // return rotAngles;
}

// Returns the angular position of the center of mass relative to the perspective line
// of the camera, in degrees
// All distances must be in inches
double getYawToCenterOfMass(cv::Point& mc, double euclidDist)
{
  double distInPixels = (mc.x - camera::SCREEN_WIDTH/2);

  // Numerator is pixels -> mm -> inches
  return ::std::asin((distInPixels * camera::MM_OVER_PIXELS / INCHES_OVER_MM)/ euclidDist) * 180 / M_PI;
}

// Translates a point by a specified distance x and y
void translatePoint(cv::Point& pt, double dispX = 0, double dispY = 0)
{
  // ::std::cout << "Start\n";
  // ::std::cout << "pt.x: " << pt.x << "\n";
  // ::std::cout << "pt.y: " << pt.y << "\n";
  // pt += cv::Point(dispX, dispY);
  pt.x += dispX;
  pt.y += dispY;
  // ::std::cout << "pt.x: " << pt.x << "\n";
  // ::std::cout << "pt.y: " << pt.y << "\n";
}

int main (int argc, char *argv[])
{
  // Parameters for selecting which filter windows to see
  int blur = 0;
  int color = 0;
  int dilateErode = 0;
  int edge = 0;
  int laplacian = 0;
  int houghLines = 0;
  int houghCircles = 0;
  int uShapeThresholdWindow = 0;
  int sideRatioThresholdWindow = 0;
  int areaRatioThresholdWindow = 0;
  int angleThresholdWindow = 0;
  int drawStats = 0;
  int distanceCalib = 0;
  int merge = 0;

  // Parameters for applying filters even if windows are closed
  int applyBlur = 1;
  int applyColor = 1;
  int applyDilateErode = 0;
  int applyEdge = 0;
  int applyLaplacian = 0;
  int applyHoughLines = 0;
  int applyHoughCircles = 0;
  int applyUShapeThreshold = 0;
  int applySideRatioThreshold = 1;
  int applyAreaRatioThreshold = 1;
  int applyAngleThreshold = 1;
  int showDistanceCalib = 1;
  int applyMerge = 1; 

  // gaussianBlur parameters
  int blur_ksize = 5;
  int sigmaX = 10;
  int sigmaY = 10;

  // hsvColorThreshold parameters
  int hMin = 0;
  int hMax = 360;
  int sMin = 0;
  int sMax = 35;
  int vMin = 50;
  int vMax = 100;
  int debugMode = 0;
  // 0 is none, 1 is bitAnd between h, s, and v
  int bitAnd = 1;

  // dilateErode parameters
  int holes = 0;
  int noise = 0;
  int size = 1;
  cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS,
      cv::Size(2 * size + 1, 2 * size + 1), 
      cv::Point(size, size) );

  // cannyEdgeDetect parameters
  int threshLow = 100;
  int threshHigh = 245;

  // laplacian paraeters
  int laplacian_ksize = 3;
  int scale = 1;
  int delta = 0;

  // houghLines parameters
  int rho = 1;
  int theta = 180;
  int threshold = 50;
  int lineMin = 50;
  int maxGap = 10;

  // houghCircles parameters
  int hcMinDist = camera::SCREEN_HEIGHT / 8;
  int hcMinRadius = 116;
  int hcMaxRadius = 212;

  // mergeFinal parameters
  int mergeWeight1 = 20;
  int mergeWeight2 = 100;

  // Shape threshold parameters
  // Params need to be ints for opencv windows
  int sideRatioParam = 270;
  int areaRatioParam = 100;
  int minAreaParam = 800;
  int maxAreaParam = 20000;
  int sideRatioMaxDeviationParam = 80;
  int areaRatioMaxDeviationParam = 90;
  int angleMaxDeviationParam = 20;
  double sideRatio = (double) sideRatioParam / 100;
  double areaRatio = (double) areaRatioParam / 100;
  double minArea = (double) minAreaParam;
  double maxArea = (double) maxAreaParam;
  double sideRatioMaxDeviation = (double) sideRatioMaxDeviationParam / 100;
  double areaRatioMaxDeviation = (double) areaRatioMaxDeviationParam / 100;
  double angleMaxDeviation = (double) angleMaxDeviationParam;

  // uShape
  int minDistFromContours = 5;
  int maxDistFromContours = 15;

  int contoursThresh = 140;
  ::std::vector< ::std::vector<cv::Point> > contours;
  ::std::vector<cv::RotatedRect> boundedRects;
  cv::Point2f rectPoints[4];
  int goalInd = 0;

  double focalLength = 600; // Intrinsic property
  int applyDistanceCalib = 0;
  // Measurements in inches
  int heightDisplacement = game_piece::ELEVATION - camera::ELEVATION;
  int calibDistance = 24;

  // Angular displacement parameters
  // cv::Vec3f angleVec;
  cv::Mat rvec, tvec;

  // Linearize contours parameters
  int approximationAccuracy = 3;

  // Corner parameters
  int blockSize = 2;
  int apertureSize = 7;
  int k = 1;
  int isHarris = 0;
  int maxCorners = 4;
  int minQualityRatio = 80;
  int minDist = 10;

  // Probably change POD struct to just initialize these
  CornerExtractor::CornerParams cornerParams = 
  {
    .windowName = "Corner Extractor", 
    .showWindows = 1, 
    .applyFilter = 1, 
    .qualityLevel = 1, 
    .minDist = 10, 
    .k = 0.04, 
    .blockSize = 3, 
    .maxCorners = MAX_GAME_PIECE_CORNERS, 
    .useHarrisDetector = false, 
    .winSize = cv::Size(5, 5), 
    .zeroZone = cv::Size(-1, -1), 
    .criteria = cv::TermCriteria( 
        cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 40, 0.001) 
  };

  CornerExtractor gamePiece (cornerParams);

  // Gnuplot parameters
  ::std::ofstream fpsFile, dataFile;
  fpsFile.open(FPS_FILE.c_str(), ::std::ios::out);
  dataFile.open(PROC_DATA_FILE.c_str(), ::std::ios::out);

  double avg = 0;
  double fpsTick = 1;
  double fps = 0;

  int lineThickness = 1;

  // Camera intrinsic parameters from calibration
  cv::FileStorage fs;
  fs.open(camera::CONFIG_FILE, cv::FileStorage::READ);

  camera::GlobalCoordinates camera_coords;

  // Read camera matrix and distortion coefficients from file
  cv::Mat intrinsics, distortion;
  fs["Camera_Matrix"] >> intrinsics;
  fs["Distortion_Coefficients"] >> distortion;

  // Close the input file
  fs.release();

  // std::cout << TARGET_ADDR << std::endl;
  // udp_client_server::udp_client client(TARGET_ADDR, UDP_PORT);
  // udp_client_server::udp_server server(HOST_ADDR, UDP_PORT);
  aos::events::TXUdpSocket client (TARGET_ADDR, UDP_PORT);

  // ::std::thread netSend (sendData, ::std::ref(client));
  // netSend.detach();
  // ::std::thread netReceive (receiveData, ::std::ref(server));
  // netReceive.detach();

  // Protobuf message to send to roboRIO
  // ::std::unique_ptr<y2017::vision::VisionData> msg (new y2017::vision::VisionData);
  y2017::vision::VisionData msg;

  // No video passed in
  cv::VideoCapture cap;
  if (argc == 1)
  {
    ::std::cout << "Using camera at port " << camera::ID << "\n";
    cap = cv::VideoCapture (camera::ID);
    // cap.set(CV_CAP_PROP_EXPOSURE, 0.01);
  }
  else // Use video from filename passed in
  {
    ::std::cout << "Opening video from " << argv[1] << "\n";
    cap = cv::VideoCapture (argv[1]);
  }
  // To make sure that input video loops when it gets to the end
  int frameInd = 0;
  int totalFrames = argc == 1 ? 0 : cap.get(CV_CAP_PROP_FRAME_COUNT);

  if (!cap.isOpened())
  {
    ::std::cerr << "ERROR - Could not open camera at port " << camera::ID << ::std::endl;
    return -1;
  }

  // Writing filtered video to filesystem
  bool isOutputColored = true;
  cv::VideoWriter os (camera::OUT_VIDEO_FILE, 
      CV_FOURCC('P', 'I', 'M', '1'), 
      camera::FPS, 
      cv::Size(camera::SCREEN_WIDTH, camera::SCREEN_HEIGHT), 
      isOutputColored);

  ::std::cout << "Opened camera at port " << camera::ID << "\n";
  ::std::cout << "Camera exposure: " << cap.get(CV_CAP_PROP_EXPOSURE) << "\n";
  if (argc > 1)
  {
    ::std::cout << "Camera fps: " << cap.get(CV_CAP_PROP_FPS) << "\n";
    ::std::cout << "Camera frame count: " << cap.get(CV_CAP_PROP_FRAME_COUNT) << "\n";
  }

  ::std::cout << "\n";
  ::std::cout << " ============== NOTICE ============= " << "\n";
  ::std::cout << "|                                   |" << "\n";
  ::std::cout << "| Press 'q' to quit without saving  |" << "\n";
  ::std::cout << "| Press 'c' to save the input image |" << "\n";
  ::std::cout << "| Press 's' to save parameters      |" << "\n";
  ::std::cout << "| Press 'l' to load parameters      |" << "\n";
  ::std::cout << "| Press 'r' to restart video        |" << "\n";
  ::std::cout << "| Press ' ' to pause                |" << "\n";
  ::std::cout << "|                                   |" << "\n";
  ::std::cout << " =================================== " << "\n";
  ::std::cout << "\n";

  cv::Mat img, rgb;
  char kill = 'l';

  ::std::chrono::high_resolution_clock::time_point start, end;

#if TRACK_FPS
  start = ::std::chrono::high_resolution_clock::now();
#endif
  while (kill != 'q')
  {
    // Control the program with user input
    if (kill == ' ')
    {
      if (img.empty() || rgb.empty()) cap >> rgb;
      img = rgb.clone();
    }
    else
    {
      frameInd++;
      cap >> img;
      aos::monotonic_clock::time_point tp = aos::monotonic_clock::now();
      // std::cout << chrono::duration_cast<chrono::nanoseconds>(tp.time_since_epoch()).count() << std::endl;
      msg.set_image_timestamp(chrono::duration_cast<chrono::nanoseconds>(tp.time_since_epoch()).count());
      if (frameInd == totalFrames)
      {
        frameInd = 0;
        cap.set(CV_CAP_PROP_POS_FRAMES, 0);
      }
#if IS_CAMERA_UPSIDE_DOWN
      flip(img, img, 0);
#endif
      rgb = img.clone();
    }

    // Press s to save values into FileStorage
    if (kill == 's')
    {
      ::std::cout << "Saving config to logs/config.yml\n";

      cv::FileStorage fs("logs/config.yml", cv::FileStorage::WRITE);
      fs << "Is Blur Window Open" << blur
      << "Is Color Window Open" << color
      << "Is Dilate Erode Window Open" << dilateErode
      << "Is Edge Window Open" << edge
      << "Is Laplacian Window Open" << laplacian
      << "Is HoughLines Window Open" << houghLines
      << "Is HoughCircles Window Open" << houghCircles
      << "Is uShapeThreshold Window Open" << uShapeThresholdWindow
      << "Is sideRatioThreshold Window Open" << sideRatioThresholdWindow
      << "Is areaRatioThreshold Window Open" << areaRatioThresholdWindow
      << "Is angleThreshold Window Open" << angleThresholdWindow
      << "Is drawStats Open" << drawStats
      << "Is merge Open" << merge

      << "Apply Blur" << applyBlur
      << "Apply Color" << applyColor
      << "Apply DilateErode" << applyDilateErode
      << "Apply Edge" << applyEdge
      << "Apply Laplacian" << applyLaplacian
      << "Apply HoughLines" << applyHoughLines
      << "Apply HoughCircles" << applyHoughCircles
      << "Apply UShapeRatioThreshold" << applyUShapeThreshold
      << "Apply SideRatioThreshold" << applySideRatioThreshold
      << "Apply AreaRatioThreshold" << applyAreaRatioThreshold
      << "Apply AngleThreshold" << applyAngleThreshold
      << "Apply Merge" << applyMerge

      << "Gaussian Blur Kernel Size" << blur_ksize
      << "Guassian Blur Sigma X" << sigmaX
      << "Guassian Blur Sigma Y" << sigmaY

      << "Hue Minimum Threshold" << hMin
      << "Hue Maximum Threshold" << hMax
      << "Saturation Minimum Threshold" << sMin
      << "Saturation Maximum Threshold" << sMax
      << "Value Minimum Threshold" << vMin
      << "Value Maximum Threshold" << vMax

      << "Dilate Erode Holes" << holes
      << "Dilate Erode Noise" << noise
      << "Dilate Erode Size" << size

      << "Canny Lower Threshold" << threshLow
      << "Canny Higher Threshold" << threshHigh

      << "Laplacian Kernel Size" << laplacian_ksize
      << "Laplacian Scale" << scale
      << "Laplacian Delta" << delta

      << "HoughLines Rho" << rho
      << "HoughLines Theta" << theta
      << "HoughLines Threshold" <<  threshold
      << "HoughLines LineMin" << lineMin
      << "HoughLines MaxGap" <<  maxGap

      << "HoughCircles Minimum Distance" << hcMinDist
      << "HoughCircles Minimum Radius" << hcMinRadius
      << "HoughCircles Maximum Radius" << hcMaxRadius

      << "Merge Weight 1" << mergeWeight1
      << "Merge Weight 2" <<  mergeWeight2

      << "Side Ratio Parameter" << sideRatioParam
      << "Area Ratio Parameter" << areaRatioParam
      << "Minimum Area Parameter" << minAreaParam
      << "Maximum Area Parameter" << maxAreaParam
      << "Side Ratio Maximum Deviation Parameter" << sideRatioMaxDeviationParam
      << "Area Ratio Maximum Deviation Parameter" << areaRatioMaxDeviationParam
      << "Angle Max Deviation Parameter" << angleMaxDeviationParam
      << "Corner Extractor Parameters" 
      << "{"
      << "Window Name" << cornerParams.windowName
      << "Apply Filter" << cornerParams.applyFilter
      << "Show Windows" << cornerParams.showWindows
      << "Quality Level" << cornerParams.qualityLevel
      << "Minimum Distance" << cornerParams.minDist
      << "k" << cornerParams.k
      << "Block Size" << cornerParams.blockSize
      << "Max Corners" << cornerParams.maxCorners
      << "Win Size" << cornerParams.winSize
      << "Zero Zone" << cornerParams.zeroZone
      // << "Term Criteria" << criteria
      << "}";
      fs.release();
    }
    else if (kill == 'l')
    {
      ::std::cout << "Loading config from logs/config.yml\n";
      kill = '~'; // Junk value that shouldn't be used

      cv::FileStorage fs("logs/config.yml", cv::FileStorage::READ);
      fs["Is Blur Window Open"] >> blur;
      fs["Is Color Window Open"] >> color;

      fs["Is Color Window Open"] >> color;
      fs["Is Dilate Erode Window Open"] >> dilateErode;
      fs["Is Edge Window Open"] >> edge;
      fs["Is Laplacian Window Open"] >> laplacian;
      fs["Is HoughLines Window Open"] >> houghLines;
      fs["Is HoughCircles Window Open"] >> houghCircles;
      fs["Is uShapeThreshold Window Open"] >> uShapeThresholdWindow;
      fs["Is sideRatioThreshold Window Open"] >> sideRatioThresholdWindow;
      fs["Is areaRatioThreshold Window Open"] >> areaRatioThresholdWindow;
      fs["Is angleThreshold Window Open"] >> angleThresholdWindow;
      fs["Is drawStats Open"] >> drawStats;
      fs["Is merge Open"] >> merge;

      fs["Apply Blur"] >> applyBlur;
      fs["Apply Color"] >> applyColor;
      fs["Apply DilateErode"] >> applyDilateErode;
      fs["Apply Edge"] >> applyEdge;
      fs["Apply Laplacian"] >> applyLaplacian;
      fs["Apply HoughLines"] >> applyHoughLines;
      fs["Apply HoughCircles"] >> applyHoughCircles;
      fs["Apply UShapeRatioThreshold"] >> applyUShapeThreshold;
      fs["Apply SideRatioThreshold"] >> applySideRatioThreshold;
      fs["Apply AreaRatioThreshold"] >> applyAreaRatioThreshold;
      fs["Apply AngleThreshold"] >> applyAngleThreshold;
      fs["Apply Merge"] >> applyMerge;

      fs["Gaussian Blur Kernel Size"] >> blur_ksize;
      fs["Guassian Blur Sigma X"] >> sigmaX;
      fs["Guassian Blur Sigma Y"] >> sigmaY;

      fs["Hue Minimum Threshold"] >> hMin;
      fs["Hue Maximum Threshold"] >> hMax;
      fs["Saturation Minimum Threshold"] >> sMin;
      fs["Saturation Maximum Threshold"] >> sMax;
      fs["Value Minimum Threshold"] >> vMin;
      fs["Value Maximum Threshold"] >> vMax;

      fs["Dilate Erode Holes"] >> holes;
      fs["Dilate Erode Noise"] >> noise;
      fs["Dilate Erode Size"] >> size;

      fs["Canny Lower Threshold"] >> threshLow;
      fs["Canny Higher Threshold"] >> threshHigh;

      fs["Laplacian Kernel Size"] >> laplacian_ksize;
      fs["Laplacian Scale"] >> scale;
      fs["Laplacian Delta"] >> delta;

      fs["HoughLines Rho"] >> rho;
      fs["HoughLines Theta"] >> theta;
      fs["HoughLines Threshold"] >>  threshold;
      fs["HoughLines LineMin"] >> lineMin;
      fs["HoughLines MaxGap"] >>  maxGap;

      fs["HoughCircles Minimum Distance"] >> hcMinDist;
      fs["HoughCircles Minimum Radius"] >> hcMinRadius;
      fs["HoughCircles Maximum Radius"] >> hcMaxRadius;

      fs["Merge Weight 1"] >> mergeWeight1;
      fs["Merge Weight 2"] >>  mergeWeight2;

      fs["Side Ratio Parameter"] >> sideRatioParam;
      fs["Area Ratio Parameter"] >> areaRatioParam;
      fs["Minimum Area Parameter"] >> minAreaParam;
      fs["Maximum Area Parameter"] >> maxAreaParam;
      fs["Side Ratio Maximum Deviation Parameter"] >> sideRatioMaxDeviationParam;
      fs["Area Ratio Maximum Deviation Parameter"] >> areaRatioMaxDeviationParam;
      fs["Angle Max Deviation Parameter"] >> angleMaxDeviationParam;
      fs["Corner Extractor Parameters"]["Window Name"] >> cornerParams.windowName;
      fs["Corner Extractor Parameters"]["Apply Filter"] >> cornerParams.applyFilter;
      fs["Corner Extractor Parameters"]["Show Windows"] >> cornerParams.showWindows;
      fs["Corner Extractor Parameters"]["Quality Level"] >> cornerParams.qualityLevel;
      fs["Corner Extractor Parameters"]["Minimum Distance"] >> cornerParams.minDist;
      fs["Corner Extractor Parameters"]["k"] >> cornerParams.k;
      fs["Corner Extractor Parameters"]["Block Size"] >> cornerParams.blockSize;
      fs["Corner Extractor Parameters"]["Max Corners"] >> cornerParams.maxCorners;
      fs["Corner Extractor Parameters"]["Win Size"] >> cornerParams.winSize;
      fs["Corner Extractor Parameters"]["Zero Zone"] >> cornerParams.zeroZone;

      fs.release();
    }
    else if (kill == 'r')
    {
      ::std::cout << "Restarting video\n";
      cap.set(CV_CAP_PROP_POS_FRAMES, 0);
    }
    else if (kill == 'p')
    {
      ::std::cout << "Writing input image to " << camera::OUT_IMAGE_FILE << "\n";
      cv::imwrite(camera::OUT_IMAGE_FILE, img);
    }
    else if (kill == 'v')
    {
      ::std::cout << "Writing input video to " << camera::OUT_VIDEO_FILE << "\n";
      if (os.isOpened()) os.write(img);
    }
    if (img.empty())
    {
      ::std::cout << "ERROR - Image is bad (" << img.rows << ", " << img.cols << ")\n";
      continue;
    }
    {
      double yaw = 11;
      msg.set_yaw(yaw);
      aos::monotonic_clock::time_point tp = aos::monotonic_clock::now();
      msg.set_send_timestamp(chrono::duration_cast<chrono::nanoseconds>(tp.time_since_epoch()).count());
      sendProtobuf(msg, client);
    }

#if CALIB && STREAM
    {
      selectMode(blur, color, dilateErode, edge, laplacian, houghLines, houghCircles, uShapeThresholdWindow, sideRatioThresholdWindow, areaRatioThresholdWindow, angleThresholdWindow, distanceCalib, drawStats, merge);
      // MJPG streamer will stream the image after a filter if the filter's window is visible
      // This is currently a messy way of doing this, but it'll work for now
      // Note that only one window can be visible at a time, or else the stream will flicker
      gaussianBlurWindows(img, blur_ksize, sigmaX, sigmaY, applyBlur, blur, STREAM);
      if (blur) mjpgStream(img);

      hsvColorThresholdWindows(img, hMin, hMax, sMin, sMax, vMin, vMax, debugMode, bitAnd, applyColor, color, STREAM);
      if (color) mjpgStream(img);

      dilateErodeWindows(img, element, holes, noise, applyDilateErode, dilateErode, STREAM);
      if (dilateErode) mjpgStream(img);

      cannyEdgeDetectWindows(img, threshLow, threshHigh, applyEdge, edge, STREAM);
      if (edge) mjpgStream(img);

      laplacianSharpenWindows(img, laplacian_ksize, scale, delta, applyLaplacian, laplacian, STREAM);
      if (laplacian) mjpgStream(img);

      houghLinesWindows(img, rho, theta, threshold, lineMin, maxGap, applyHoughLines, houghLines, STREAM);
      if (houghLines) mjpgStream(img);

      houghCirclesWindows(img, hcMinDist, hcMinRadius, hcMaxRadius, applyHoughCircles, houghCircles, STREAM);
      if (houghCircles) mjpgStream(img);
    }
#elif CALIB
    {
      selectMode(blur, color, dilateErode, edge, laplacian, houghLines, houghCircles, uShapeThresholdWindow, sideRatioThresholdWindow, areaRatioThresholdWindow, angleThresholdWindow, distanceCalib, drawStats, merge);
      gaussianBlurWindows(img, blur_ksize, sigmaX, sigmaY, applyBlur, blur, STREAM);
      hsvColorThresholdWindows(img, hMin, hMax, sMin, sMax, vMin, vMax, debugMode, bitAnd, applyColor, color, STREAM);
      dilateErodeWindows(img, element, holes, noise, applyDilateErode, dilateErode, STREAM);
      cannyEdgeDetectWindows(img, threshLow, threshHigh, applyEdge, edge, STREAM);
      laplacianSharpenWindows(img, laplacian_ksize, scale, delta, applyLaplacian, laplacian, STREAM);
      // houghLinesWindows(img, rho, theta, threshold, lineMin, maxGap, applyHoughLines, houghLines, STREAM);
      houghCirclesWindows(img, hcMinDist, hcMinRadius, hcMaxRadius, applyHoughCircles, houghCircles, STREAM);
    }
#else
    {
      gaussianBlur(img, blur_ksize, sigmaX, sigmaY);
      hsvColorThreshold(img, hMin, hMax, sMin, sMax, vMin, vMax, debugMode, bitAnd);
    }
#endif

    try
    {
      contours = getContours(img, contoursThresh);
    }
    catch (::std::exception& e)
    {
      ::std::cout << "No contours found\n" << e.what() << "\n";
    }

    if (contours.size() > 0)
    {
      boundedRects = getBoundedRects(contours);
      {
#if CALIB
        {
          // Filter out contours that do not match the 'U' shape
          uShapeThresholdWindows(img, contours, boundedRects, minDistFromContours, maxDistFromContours, applyUShapeThreshold, uShapeThresholdWindow, STREAM);
          sideRatioThresholdWindows(img, contours, boundedRects, sideRatioParam, sideRatioMaxDeviationParam, applySideRatioThreshold, sideRatioThresholdWindow, STREAM);
          areaRatioThresholdWindows(img, contours, boundedRects, minAreaParam, maxAreaParam, areaRatioParam, areaRatioMaxDeviationParam, applyAreaRatioThreshold, areaRatioThresholdWindow, STREAM);
          angleThresholdWindows(img, contours, boundedRects, angleMaxDeviationParam, applyAngleThreshold, angleThresholdWindow, STREAM);
        }
#else
        {
          sideRatioThreshold(contours, boundedRects, sideRatio, sideRatioMaxDeviation);
          areaRatioThreshold(contours, boundedRects, minArea, maxArea, areaRatio, areaRatioMaxDeviation);
          angleThreshold(contours, boundedRects, angleMaxDeviation);
        }
#endif
      }
      if (contours.size() > 0)
      {
        goalInd = getLeftMostContourIndex(contours);
        boundedRects[goalInd].points(rectPoints); // Extract the corners of the target's bounded box

        cv::Point mc = getCenterOfMass(contours[goalInd]);

        {
          double yaw = ((mc.x - camera::SCREEN_WIDTH / 2) * camera::PIX_TO_DEG); // Robot heading
          if (!::std::isfinite(yaw)) yaw = -1;
          ::std::string gnuplotBuf = ::std::to_string(yaw);
          dataFile << gnuplotBuf.c_str() << "\n";

          msg.set_yaw(yaw);
          aos::monotonic_clock::time_point tp = aos::monotonic_clock::now();
          msg.set_send_timestamp(chrono::duration_cast<chrono::nanoseconds>(tp.time_since_epoch()).count());
          sendProtobuf(msg, client);
        }
#if CALIB
        // Draw all the contours
        drawContours(img, contours, -1, RED, lineThickness, 8);
        drawBoundedRects(img, boundedRects, PURPLE);
#endif
      }
    }
#if CALIB
    {
      if (uShapeThresholdWindow || sideRatioThresholdWindow || areaRatioThresholdWindow || angleThresholdWindow) 
      {
        if (STREAM) mjpgStream(img);
        else cv::imshow("Threshold Output", img);
      }
      else if (!STREAM) cv::destroyWindow("Threshold Output");
      if (!STREAM) cv::imshow("Final Output", img);
      mergeFinalWindows(rgb, img, mergeWeight1, mergeWeight2, applyMerge, merge, STREAM);
      if (STREAM && !blur && !color && !dilateErode && !edge && !laplacian && !houghLines && !houghCircles && !uShapeThresholdWindow && !sideRatioThresholdWindow && !areaRatioThresholdWindow && !angleThresholdWindow && !drawStats)
        mjpgStream(img);
    }
#elif STREAM
    {
      // Stream the final merged image
      mergeFinal(rgb, img, mergeWeight1, mergeWeight2);
      mjpgStream(img);
    }
#endif
#if TRACK_FPS
    {
      end = ::std::chrono::high_resolution_clock::now();
      // auto timeElapsed = ::std::chrono::duration_cast<::std::chrono::duration<double>>(end - start).count();
      auto timeElapsed = ::std::chrono::duration_cast<::std::chrono::milliseconds>(end - start).count();
      start = ::std::chrono::high_resolution_clock::now();
      // ::std::cout << timeElapsed << "\n";
      // fps = 1.0 / timeElapsed;
      // ::std::string gnuplotBuf = ::std::to_string(fps);

      // ::std::cout << "Time elapsed in milliseconds: " << timeElapsed << "\n";
      ::std::string gnuplotBuf = ::std::to_string(timeElapsed) + "  " + ::std::to_string(timeElapsed / ++fpsTick);

      fpsFile << gnuplotBuf.c_str() << "\n";
      avg += timeElapsed;
    }
#endif 
#if CALIB
    // Paused
    if (kill == ' ') 
    {
      kill = cv::waitKey(5); // Only wait x milliseconds if using cv::imshow()
      if (kill == ' ') // Unpause
        kill = '~';
      if (kill == -1) // No key was pressed while program is paused, so stay paused
        kill = ' ';
    }
    else
    {
      kill = cv::waitKey(5); // Only wait x milliseconds if using cv::imshow()
    }
#elif STREAM
    kill = '~'; // Junk value that should never be used as an action
#endif
  }
  os.release();
  return 0;
}
