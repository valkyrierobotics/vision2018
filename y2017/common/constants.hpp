#ifndef _Y2017_COMMON_CONSTANTS_H_
#define _Y2017_COMMON_CONSTANTS_H_

#include <cmath>
#include <opencv2/opencv.hpp>

#define TRACK_FPS 1 // Calculating fps
#define CALIB 0 // Calibrating with windows instead of deployment
#define STREAM 1 // Streaming to mjpg-streamer instead of cv::imshow
#define IS_CAMERA_UPSIDE_DOWN 0 // Flip image if camera is upside down
#define MAX_GAME_PIECE_CORNERS 4

namespace camera
{
  const int ID = 0; // Index of camera
  const int SCREEN_WIDTH = 640;
  const int SCREEN_HEIGHT = 480;
  const int FPS = 30;

  // Measurements in inches
  const double ELEVATION = 0.5; // Mounting position above ground level

  // Measurements in millimeters
  const double SENSOR_WIDTH_MM = 6.22;
  const double MM_OVER_PIXELS = SENSOR_WIDTH_MM / SCREEN_WIDTH;

  // Measurements in degrees
  const double DIAG_FOV = 78;

  // Measurements in pixels
  // Diagonal FOV of camera screen
  const double DIAG_SCREEN = std::sqrt(
      camera::SCREEN_WIDTH * camera::SCREEN_WIDTH 
      + camera::SCREEN_HEIGHT * camera::SCREEN_HEIGHT); 

  const double PIX_TO_DEG = DIAG_FOV / DIAG_SCREEN;

  const std::string CONFIG_FILE = "logs/camera_calibration_data/best_camera_calibration_data.xml"; // Symlinked from appropriate camera
  const std::string OUT_VIDEO_FILE = "images/snapped_images/main.avi";
  const std::string OUT_IMAGE_FILE = "images/snapped_images/main.jpg";

  // Relative to target
  struct GlobalCoordinates
  {
    // x and y are coplanar, z is along perspective line
    // https://i.stack.imgur.com/mzEZy.jpg
    double x, y, z;
    double theta;
    double euclidDist;
  };
}

const std::string FPS_FILE = "logs/fps.log";
const std::string PROC_DATA_FILE = "logs/processed_data.log";

// const std::string TARGET_ADDR = "10.2.99.2"; // Static IP address of roboRIO
const std::string TARGET_ADDR = "127.0.0.1"; // Localhost for testing
const std::string HOST_ADDR = "localhost";
const int UDP_PORT = 5810;

cv::Scalar GREEN (0, 255, 0);
cv::Scalar BLUE_GREEN (255, 255, 0);
cv::Scalar PURPLE (255, 0, 255);
cv::Scalar LIGHT_GREEN (255, 100, 100);
cv::Scalar RED (0, 0, 255);
cv::Scalar YELLOW (0, 255, 255);

namespace game_piece
{
  // Measurements in inches
  const double WIDTH = 2;
  const double HEIGHT = 5;
  const double SPACING = 6.25; // Spacing between the inner sides of the tapes
  const double ELEVATION = 10.75;
}

#endif // __Y2017_COMMON_CONSTANTS_H_
