#include <iostream>
#include <algorithm>
#include <opencv2/highgui/highgui.hpp>

#include "utils/getCorners.hpp"
#include "utils/getCenterOfMass.hpp"
#include "utils/getAngles.hpp"
#include "utils/vector_ordering.hpp"

CornerExtractor::CornerExtractor(CornerExtractor::CornerParams& params) :
  params_(&params) {}

CornerExtractor::CornerExtractor(CornerExtractor::CornerParams& params,
    std::vector<cv::Point>& pts,
    cv::Point& cm) :
  CornerExtractor::CornerExtractor(params)
{
  pts_ = &pts;
  cm_ = &cm;
}

void CornerExtractor::update(cv::Mat& img,
    std::vector<cv::Point>& pts,
    cv::Point& cm)
{
  imgGray_ = &img;
  pts_ = &pts;
  cm_ = &cm;
}

// Returns a vector of points that contains the four corners of a rectangle
// by finding the best features.
// Automatically refines the corners.
// Currently only tested with four points (rectangle)
// Returns empty vector when filter is not applied / successful.
std::vector<cv::Point2f> CornerExtractor::getCorners()
{
#if CALIB && !STREAM
  // TODO: set this up with callbacks that somehow call update()
  startWindows();
#endif

  if (!(params_->applyFilter))
      return std::vector<cv::Point2f>();

  // if (params_->applyFilter)
  {
    findGoodCorners();

    // Refine the corners so that distances are absolute minima
    refineCorners();

    orderCorners();

    return corners_;
  }
}

void CornerExtractor::startWindows()
{
  if (params_->showWindows)
  {
    // TODO: implement the DoubleTrack trackbars
    cv::namedWindow(params_->windowName);
    cv::createTrackbar("Apply Filter", params_->windowName, &(params_->applyFilter), 1);
    cv::createTrackbar("Max Corners", params_->windowName, &(params_->maxCorners), 20);
    cv::createTrackbar("Block Size", params_->windowName, &(params_->blockSize), 10);
    cv::createTrackbar("Min Dist", params_->windowName, &(params_->minDist), 100);
    cv::createTrackbar("Quality Level (divide 100)", params_->windowName, &(params_->qualityLevel), 100);
  }
  else
  {
    cv::destroyWindow(params_->windowName);
  }
}

void CornerExtractor::findGoodCorners()
{
  if (params_->qualityLevel == 0) params_->qualityLevel++;
  if (params_->blockSize == 0) params_->blockSize++;

  cv::goodFeaturesToTrack(*imgGray_,
      corners_,
      params_->maxCorners,
      // params_->qualityLevel,
      params_->qualityLevel / 100.0,
      params_->minDist,
      cv::Mat(),
      params_->blockSize,
      params_->useHarrisDetector,
      params_->k);
}

// Checks + or - numPointsToCheck points from the current corner
// in order to see if there is a better corner (norm is absolute minima)
void CornerExtractor::refineCorners()
{
  if (corners_.size() == 0) return;
  if (imgGray_->empty()) return;

  // Calculate the refined corner locations
  cornerSubPix(*imgGray_, corners_, params_->winSize, params_->zeroZone, params_->criteria);
}

// Gets the top (MAX_GAME_PIECE_CORNERS) corners from the possible corners
// TODO: unit test this
// TODO: figure out how to make this more memory efficient, it's currently copying twice
void CornerExtractor::getTopCorners()
{
  // Partial sort the vector from greatest to least norms
  // O(N log(M)) is faster than O(N) when M is small
  std::partial_sort(norms_.begin(), 
      norms_.begin()+MAX_GAME_PIECE_CORNERS,
      norms_.end(), comp);

  cv::Point tmp [MAX_GAME_PIECE_CORNERS];
  for (size_t i = 0; i < MAX_GAME_PIECE_CORNERS; ++i)
    tmp[i] = corners_[norms_[i].ind];

  corners_.clear();
  for (size_t i = 0; i < MAX_GAME_PIECE_CORNERS; ++i)
    corners_.push_back(tmp[i]);
}

// TODO: make this faster, currently copying twice
void CornerExtractor::orderCorners()
{
  std::vector<double> angles;

  // Find the angle between the 
  // vector of the center of mass to the corner and the x axis
  for (size_t i = 0; i < corners_.size(); ++i)
    angles.push_back(angle_wrt_x_axis(corners_[i], *cm_));

  std::vector<size_t> idx = sort_indexes(angles, comp_double);
  std::vector<cv::Point> tmp;
  for (auto i : idx)
    tmp.push_back(corners_[i]);

  corners_.clear();
  // Convert to OpenCV system where bottom left is index 0
  for (size_t i = 0; i < tmp.size(); ++i)
    corners_.push_back(tmp[(i+1) % 4]);
}
