#ifndef GET_CORNERS_HPP
#define GET_CORNERS_HPP

#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <string>

#include "distance.hpp"
#include "../common/constants.hpp"

#define DEFAULT_CORNER_PARAMS \
{ \
  .windowName = "Corner Extractor", \
  .showWindows = 1, \
  .applyFilter = 1, \
  .qualityLevel = 1, \
  .minDist = 10, \
  .k = 0.04, \
  .blockSize = 3, \
  .maxCorners = MAX_GAME_PIECE_CORNERS, \
  .useHarrisDetector = false, \
  .winSize = cv::Size(5, 5), \
  .zeroZone = cv::Size(-1, -1), \
  .criteria = cv::TermCriteria( \
      cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 40, 0.001) \
}

// static void write(cv::FileStorage& fs, const std::string&, const CornerExtractor::CornerParams& p)
// {
//   p.write(fs);
// }
//
// class CornerExtractor
// {
//  public:
//   struct CornerParams
//   {
//     std::string windowName;
//     // bool showWindows;
//     // bool applyFilter;
//     int showWindows;
//     int applyFilter;
//
//     // double qualityLevel;
//     int qualityLevel;
//     int minDist;
//     // double minDist;
//     double k;
//     int blockSize;
//     int maxCorners;
//     bool useHarrisDetector;
//
//     // For cornerSubPix
//     cv::Size winSize;
//     cv::Size zeroZone;
//     cv::TermCriteria criteria;
//
//     CornerParams() :
//       windowName("Corner Extractor"),
//       qualityLevel(#<{(|0.01|)}># 1),
//       minDist(10),
//       k(0.04),
//       blockSize(3),
//       maxCorners(MAX_GAME_PIECE_CORNERS),
//       useHarrisDetector(false),
//       showWindows(1),
//       applyFilter(1),
//       winSize(cv::Size(5, 5)),
//       zeroZone(cv::Size(-1, -1)),
//       criteria(cv::TermCriteria(
//           cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 40, 0.001)) {}
//
//     void write(cv::FileStorage& fs) const
//     {
//       fs << "{" 
//         << "Window Name" << windowName
//         << "Quality Level" << qualityLevel
//         << "Minimum Distance" << minDist
//         << "k" << k
//         << "Block Size" << blockSize
//         << "Max Corners" << maxCorners
//         << "Show Windows" << showWindows
//         << "Win Size" << winSize
//         << "Zero Zone" << zeroZone
//         // << "Term Criteria" << criteria
//         << "}";
//     }
//   };
//
// static void write(cv::FileStorage& fs, const std::string&, const CornerExtractor::CornerParams& p)
// {
//   p.write(fs);
// }
//
class CornerExtractor
{
 public:
  struct CornerParams
  {
    std::string windowName;
    // bool showWindows;
    // bool applyFilter;
    int showWindows;
    int applyFilter;

    // double qualityLevel;
    int qualityLevel;
    int minDist;
    // double minDist;
    double k;
    int blockSize;
    int maxCorners;
    bool useHarrisDetector;

    // For cornerSubPix
    cv::Size winSize;
    cv::Size zeroZone;
    cv::TermCriteria criteria;
  };

  // CornerExtractor();
  CornerExtractor(CornerParams& params);
  // CornerExtractor(std::vector<cv::Point>& pts, cv::Point& cm, size_t numPointsToCheck=5, CornerParams);
  CornerExtractor(CornerParams& params, std::vector<cv::Point>& pts, cv::Point& cm);

  // void update(std::vector<cv::Point>& pts, cv::Point& cm, size_t numPointsToCheck=5);

  /**
   * @brief Update the corner extractor with information from the image.
   *
   * @param[in]     img       BGR image.
   * @param[in,out] contours  The contours of the game piece.
   * @param[in]     cm        The center of masscenter of mass.
   */
  void update(cv::Mat& img, std::vector<cv::Point>& pts, cv::Point& cm);
  std::vector<cv::Point2f> getCorners();

 private:
  void findGoodCorners();
  void refineCorners();
  void getTopCorners();
  void startWindows();
  void orderCorners();
  // void findApproxCorners();
  // double minDistanceToAdjacent(size_t pix);

  struct EuclidNorm
  {
    double norm;
    size_t ind;
  };

  struct CornerParams* params_;
  static bool comp(EuclidNorm& a, EuclidNorm& b) { return (a.norm > b.norm); }
  cv::Mat* imgGray_;

  std::vector<cv::Point>* pts_;
  std::vector<cv::Point2f> corners_;
  cv::Point* cm_; // Center of mass of the pts

  std::vector<EuclidNorm> norms_;
  // Distances to bottom left, top left, top right, bottom right
  // double norms_[MAX_GAME_PIECE_CORNERS];
  // double minDist_;

  // Indices of the contour of pts that was passed in the constructor
  // size_t indices_[MAX_GAME_PIECE_CORNERS];
  // Refine the corners using + or - numPointsToCheck_ from the corner
  // size_t numPointsToCheck_;

  // Points at the corners of the image
  // Bottom left, top left, top right, bottom right
  // cv::Point targets_[MAX_GAME_PIECE_CORNERS];
};

#endif // GET_CORNERS_HPP
