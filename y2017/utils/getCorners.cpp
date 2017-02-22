#include <iostream>
#include <algorithm>
#include <opencv2/highgui/highgui.hpp>

#include "utils/getCorners.hpp"
#include "utils/getCenterOfMass.hpp"
#include "utils/getAngles.hpp"
#include "utils/vector_ordering.hpp"

CornerExtractor::CornerExtractor(CornerExtractor::CornerParams& params) :
  params_(&params)
{
  // imgGray_ = cv::MatAllocator().allocate();
}

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

// CornerExtractor::CornerExtractor(double minDist) : 
//   minDist_(minDist), indices_{0}
// {
//   // Bottom left, top left, top right, bottom right
//   targets_[0] = cv::Point(0, SCREEN_HEIGHT);
//   targets_[1] = cv::Point(0, 0);
//   targets_[2] = cv::Point(SCREEN_WIDTH, 0);
//   targets_[3] = cv::Point(SCREEN_WIDTH, SCREEN_HEIGHT);
// }

// CornerExtractor::CornerExtractor(std::vector<cv::Point>& pts,
//     cv::Point& cm,
//     size_t numPointsToCheck,
//     double minDist) :
//   CornerExtractor::CornerExtractor(minDist)
// {
//   pts_ = &pts;
//   cm_ = &cm;
//   numPointsToCheck_ = numPointsToCheck;
// }

// void CornerExtractor::update(cv::Mat& img,
//     std::vector<cv::Point>& pts,
//     cv::Point& cm,
//     size_t numPointsToCheck)
// {
//   cv::cvtColor(img, *imgGray_, CV_8UC1);
//   pts_ = &pts;
//   cm_ = &cm;
//   numPointsToCheck_ = numPointsToCheck;
// }

// Returns a vector of points that contains the four corners of a rectangle
// by finding the points closest to each of the four corners of the image
// Automatically refines the corners
// Currently only supports MAX_GAME_PIECE_CORNERS points (rectangle)
std::vector<cv::Point2f> CornerExtractor::getCorners()
{
  // TODO: set this up with callbacks that somehow call update()
  startWindows();
  // if (params_->applyFilter)
  {
    // Find the approximate locations of the corners
    // findApproxCorners();
    findGoodFeatures();

    // Refine the corners so that distances are absolute minima
    refineCorners();

    orderCorners();

    return corners_;
    // for (size_t i = 0; i < MAX_GAME_PIECE_CORNERS; ++i)
    // {
    //   corners_.push_back((*pts_)[indices_[i]]);
    // }
    // // std::cout << "\n\n";
    // // {
    // //   cv::Mat img = cv::Mat::zeros(SCREEN_HEIGHT, SCREEN_WIDTH, CV_8UC3);
    // //   std::vector<std::vector<cv::Point> > c;
    // //   c.push_back(corners);
    // //   drawContours(img, c, -1, cv::Scalar(0, 0, 255), 5, 8);
    // //   cv::imshow("test", img);
    // // }
    // return corners_;
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

void CornerExtractor::findGoodFeatures()
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
  std::cout << "** Number of corners detected: " << corners_.size() << "\n";
}

// Checks + or - numPointsToCheck points from the current corner
// in order to see if there is a better corner (norm is absolute minima)
void CornerExtractor::refineCorners()
{
  if (corners_.size() == 0) return;

  cv::Mat img = imgGray_->clone();
  {
    std::vector<std::vector<cv::Point> > c;
    c.push_back(*pts_);
    drawContours(img, c, -1, cv::Scalar(255, 0, 0), 1, 8);
  }

  // Calculate the refined corner locations
  cornerSubPix(*imgGray_, corners_, params_->winSize, params_->zeroZone, params_->criteria);

  // Print the corners
  std::cout << "Center of Mass: (" << cm_->x << "," << cm_->y << ")\n";
  for( size_t i = 0; i < corners_.size(); i++ )
  { std::cout << " -- Refined Corner [" << i << "]  (" << corners_[i].x << "," << corners_[i].y << ")" << "\n"; }

  double maxNorm = 0;
  cv::Scalar color (255, 255, 255);
  for (size_t i = 0; i < corners_.size(); ++i)
  {
    maxNorm = distance(corners_[i], *cm_);
    norms_.push_back({maxNorm, i});
    {
      cv::line(img, corners_[i], *cm_, color, 1, 8);
      char str[50];
      sprintf(str, "%zu", i);
      cv::putText(img, str, corners_[i], CV_FONT_HERSHEY_COMPLEX_SMALL, 0.75, cv::Scalar(255, 255, 255), 1, 8, false);
    }
  }
  // getTopCorners();
  cv::imshow("test", img);
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

  {
    cv::Mat img = cv::Mat::zeros(imgGray_->size(), CV_8UC1);
    for (size_t i = 0; i < angles.size(); ++i)
    {
      char str[100];
      sprintf(str, "%zu: %3.0f", i, angles[i]);
      cv::putText(img, str, corners_[i], CV_FONT_HERSHEY_COMPLEX_SMALL, 0.75, cv::Scalar(255, 255, 255), 1, 8, false);
    }
    cv::imshow("Angles", img);
  }
  std::vector<size_t> idx = sort_indexes(angles, comp_double);
  {
    std::cout << "Indexes: ";
    for (auto i: idx)
      std::cout << i << " ";
    std::cout << "\n";
  }
  // reorder_destructive(idx.begin(), idx.end(), corners_.begin());
  std::vector<cv::Point> tmp;
  for (auto i : idx)
    tmp.push_back(corners_[i]);

  corners_.clear();
  // Convert to OpenCV system where bottom left is index 0
  for (size_t i = 0; i < tmp.size(); ++i)
    corners_.push_back(tmp[(i+1) % 4]);
  {
    cv::Mat img = cv::Mat::zeros(imgGray_->size(), CV_8UC1);
    for (size_t i = 0; i < angles.size(); ++i)
    {
      char str[100];
      sprintf(str, "%zu: %3.0f", i, angles[i]);
      cv::putText(img, str, corners_[i], CV_FONT_HERSHEY_COMPLEX_SMALL, 0.75, cv::Scalar(255, 255, 255), 1, 8, false);
    }
    cv::imshow("Post Angles", img);
  }
}

// void CornerExtractor::findApproxCorners()
// {
//   std::fill_n(norms_, MAX_GAME_PIECE_CORNERS,
//       static_cast<double>(SCREEN_WIDTH));
//   double minNorm = SCREEN_WIDTH;
//   for (size_t pix = 0; pix < pts_->size(); ++pix)
//   {
//     for (size_t corner = 0; corner < MAX_GAME_PIECE_CORNERS; ++corner)
//     {
//       if ((minNorm = distance((*pts_)[pix], targets_[corner])) < norms_[corner])
//       {
//         // std::cout << "Point: " << pix << ", " << pts_[pix] <<"\n";
//         // std::cout << "target: " << corner << ", " << targets_[corner] << "\n";
//         // std::cout << "Norm: " << corner << ", " << norms_[corner] << "\n";
//         norms_[corner] = minNorm;
//         indices_[corner] = pix;
//       }
//     }
//   }
// }

// void refineCorners()
// {
//   cv::Mat img = cv::Mat::zeros(SCREEN_HEIGHT, SCREEN_WIDTH, CV_8UC3);
//   std::vector<cv::Point> corners;
//   for (size_t i = 0; i < pts_->size(); ++i)
//   {
//     corners.push_back((*pts_)[i]);
//   }
//   std::vector<std::vector<cv::Point> > c;
//   c.push_back(corners);
//   drawContours(img, c, -1, cv::Scalar(255, 0, 0), 1, 8);
//
//   double maxNorm = 0;
//   cv::Scalar color (0, 0, 0);
//   for (size_t corner = 0; corner < MAX_GAME_PIECE_CORNERS; ++corner)
//   {
//     maxNorm = distance((*pts_)[indices_[corner]], *cm_);
//     norms_[corner] = maxNorm;
//     switch (corner)
//     {
//       case 0: color = CV_RGB(255, 0, 0); break;
//       case 1: color = CV_RGB(0, 255, 0); break;
//       case 2: color = CV_RGB(0, 0, 255); break;
//       case 3: color = CV_RGB(255, 255, 255); break;
//     }
//
//     // Check each point and see if the norm to the target is smallest
//     for (size_t pix = indices_[corner] - numPointsToCheck_;
//         pix < indices_[corner] + numPointsToCheck_ + 1;
//         ++pix)
//     {
//       // std::cout << indices_[corner] - numPointsToCheck_ << " to " << indices_[corner] + numPointsToCheck_ <<"\n";
//       if (pix < pts_->size()) 
//       {
//         cv::line(img, (*pts_)[pix], *cm_, color, 1, 8);
//         char str[50];
//         sprintf(str, "%d", pix);
//         cv::putText(img, str, (*pts_)[pix], CV_FONT_HERSHEY_COMPLEX_SMALL, 0.75, cv::Scalar(255, 255, 255), 1, 8, false);
//       }
//       // Check if the euclidean distance to the center of mass is the greatest
//       if (pix < pts_->size()
//           && (maxNorm = distance((*pts_)[pix], *cm_)) > norms_[corner]
//           && minDistanceToAdjacent(pix) > minDist_)
//       {
//         norms_[corner] = maxNorm;
//         indices_[corner] = pix;
//       }
//     }
//   }
//   cv::imshow("test", img);
// }

// double CornerExtractor::minDistanceToAdjacent(size_t pix)
// {
//   // Distance to point counterclockwise current
//   double a = distance((*pts_)[pix], (*pts_)[(pix+3) % 4]);
//
//   // Distance to point clockwise current
//   double b = distance((*pts_)[pix], (*pts_)[(pix+1) % 4]);
//
//   return a < b ? a : b;
// }
