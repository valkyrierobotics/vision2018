#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <string>
#include <stdio.h>
#include <thread>
#include <chrono>
#include <vector>
#include <fstream>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <unistd.h>
#include <libv4l2.h>
#include <fcntl.h>

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

#include "y2017/vision_data.pb.h" // Protobuf 2.5 file from vision_data.protoc

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
  int mergeWeight1 = 100;
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

  // uShape
  int minDistFromContours = 5;
  int maxDistFromContours = 15;

  int contoursThresh = 140;
  ::std::vector< ::std::vector<cv::Point> > contours;
  ::std::vector<cv::RotatedRect> boundedRects;

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

  int lineThickness = 1;

  // Camera intrinsic parameters from calibration
  cv::FileStorage fs;
  fs.open(camera::CONFIG_FILE, cv::FileStorage::READ);

  // Read camera matrix and distortion coefficients from file
  cv::Mat intrinsics, distortion;
  fs["Camera_Matrix"] >> intrinsics;
  fs["Distortion_Coefficients"] >> distortion;

  // Close the input file
  fs.release();

  ::aos::events::TXUdpSocket client (TARGET_ADDR, UDP_PORT);

  // Protobuf message to send to roboRIO
  y2017::vision::VisionData msg;

  // No video passed in
  cv::VideoCapture cap;
  if (argc == 1)
  {
    int fd;
    if ((fd = open("/dev/video1", O_RDWR)) < 0)
    {
      perror("Failed to open /dev/video1");
      exit(1);
    }
    struct v4l2_queryctrl queryctrl;
    struct v4l2_control control;

    // memset (&queryctrl, 0, sizeof(queryctrl));
    // memset(&control, 0, sizeof(control));
    //
    // control.id = V4L2_CID_AUTOGAIN;
    // control.value = false;
    // ioctl(fd, VIDIOC_S_CTRL, &control);
    //
    memset (&queryctrl, 0, sizeof(queryctrl));
    memset(&control, 0, sizeof(control));

    control.id = V4L2_CID_EXPOSURE_AUTO;
    control.value = 0;
    ioctl(fd, VIDIOC_S_CTRL, &control);

    memset (&queryctrl, 0, sizeof(queryctrl));
    memset(&control, 0, sizeof(control));

    control.id = V4L2_CID_EXPOSURE;
    control.value = 1;
    ioctl(fd, VIDIOC_S_CTRL, &control);

    close (fd);
    ::std::cout << "Using camera at port " << camera::ID << "\n";
    cap = cv::VideoCapture (camera::ID);
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
      ::aos::monotonic_clock::time_point tp = ::aos::monotonic_clock::now();
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
      selectMode(blur, color, dilateErode, edge, laplacian, houghLines, houghCircles, uShapeThresholdWindow, sideRatioThresholdWindow, areaRatioThresholdWindow, angleThresholdWindow, drawStats, merge);
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
        // Divide by 100 because OpenCV windows require ints
        sideRatioThreshold(contours, boundedRects, 
            static_cast<double>(sideRatioParam) / 100,
            static_cast<double>(sideRatioMaxDeviationParam) / 100);

        areaRatioThreshold(contours, boundedRects, 
            minAreaParam, maxAreaParam,
            static_cast<double>(areaRatioParam) / 100, 
            static_cast<double>(areaRatioMaxDeviationParam) / 100);

        angleThreshold(contours, boundedRects, angleMaxDeviationParam);
      }
#endif
    }

    if (contours.size() > 0)
    {

      cv::RotatedRect mergedRect = mergedBoundedRect(contours);
      cv::Point2f mc = mergedRect.center;
      {
        // Left handed coordinate system (counter clockwise is negative)
        double yaw = (((mc.x - camera::SCREEN_WIDTH / 2) * camera::PIX_TO_DEG) * M_PI) / 180;
        if (!::std::isfinite(yaw)) yaw = -1;
        // ::std::string gnuplotBuf = ::std::to_string(yaw);
        // dataFile << gnuplotBuf.c_str() << "\n";

        msg.set_yaw(yaw); 
        ::aos::monotonic_clock::time_point tp = ::aos::monotonic_clock::now();
        msg.set_send_timestamp(chrono::duration_cast<chrono::nanoseconds>(tp.time_since_epoch()).count());
        sendProtobuf(msg, client);
      }
#if CALIB
      drawContours(img, contours, -1, RED, lineThickness, 8); // Draw all the contours
#endif
#if CALIB || STREAM
      char str [30];
      sprintf(str, "Yaw (%4.2f)", msg.yaw());
      cv::putText(img, str, mc, CV_FONT_HERSHEY_COMPLEX_SMALL, 0.75, cv::Scalar(255, 0, 200), 1, 8, false);

      cv::circle(img, mc, 3, cv::Scalar(255, 0, 0), -1);

      // Draw the merged bounded rectangle
      cv::Point2f rectPoints[4];
      mergedRect.points(rectPoints);
      for (size_t p = 0; p < 4; ++p)
          cv::line(img, rectPoints[p], rectPoints[(p+1) % 4], PURPLE, lineThickness, 8);
#endif
    }

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
      char str [30];
      sprintf(str, "Loop Period (ms) (%-5lli)", timeElapsed);
      cv::putText(img, str, cv::Point(10, 460), CV_FONT_HERSHEY_COMPLEX_SMALL, 0.75, cv::Scalar(255, 0, 200), 1, 8, false);

      // ::std::string gnuplotBuf = ::std::to_string(timeElapsed) + "  " + ::std::to_string(timeElapsed / ++fpsTick);
      // fpsFile << gnuplotBuf.c_str() << "\n";
      // avg += timeElapsed;
    }
#endif 
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
      if (STREAM 
          && !blur 
          && !color 
          && !dilateErode 
          && !edge 
          && !laplacian 
          && !houghLines 
          && !houghCircles 
          && !uShapeThresholdWindow 
          && !sideRatioThresholdWindow 
          && !areaRatioThresholdWindow 
          && !angleThresholdWindow)
        mjpgStream(img);
    }
#elif STREAM
    {
      // Stream the final merged image
      mergeFinal(rgb, img, mergeWeight1, mergeWeight2);
      mjpgStream(img);
    }
#endif
#if CALIB
    // Paused
    if (kill == ' ') 
    {
      kill = cv::waitKey(5); // Only wait x milliseconds if using cv::imshow()

      if (kill == ' ') // Unpause
        kill = '~';
      else if (kill != 'q') // No key was pressed while program is paused, so stay paused
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
