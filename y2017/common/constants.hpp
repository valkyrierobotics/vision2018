#ifndef _Y2017_COMMON_CONSTANTS_H_
#define _Y2017_COMMON_CONSTANTS_H_

// Calculating fps
#define FPS 1
// Calibrating with windows instead of deployment
#define CALIB 1
// Streaming to mjpg-streamer instead of cv::imshow
const bool STREAM = false;
// Flip image if camera is upside down
#define IS_CAMERA_UPSIDE_DOWN 0
// Index of camera
#define CAMERA_NUM 0

#define MAX_GAME_PIECE_CORNERS 4

const int SCREEN_WIDTH = 640;
const int SCREEN_HEIGHT = 480;

// Measurements are in inches
const double TOWER_HEIGHT = 11.0;
const double CAMERA_HEIGHT = 0.5;
const double GAME_ELEMENT_WIDTH = 2;
const double GAME_ELEMENT_HEIGHT = 5;
#define GAME_ELEMENT_SPACING 6.25 // Spacing between the two parts of the tape
#define INCHES_TO_MM 25.4

double pitch = 0;
double yaw = 0;
double fps = 0;

const std::string FPS_FILE = "logs/fps.log";
const std::string PROC_DATA_FILE = "logs/processed_data.log";
const std::string CAMERA_CONFIG_FILE = "logs/camera_calibration_data/best_camera_calibration_data.xml";

const std::string TARGET_ADDR = "10.2.99.2"; // Static IP address of roboRIO
const std::string HOST_ADDR = "localhost";
const int UDP_PORT = 5810;

cv::Scalar GREEN (0, 255, 0);
cv::Scalar BLUE_GREEN (255, 255, 0);
cv::Scalar PURPLE (255, 0, 255);
cv::Scalar LIGHT_GREEN (255, 100, 100);
cv::Scalar RED (0, 0, 255);
cv::Scalar YELLOW (0, 255, 255);

#endif // __Y2017_COMMON_CONSTANTS_H_
