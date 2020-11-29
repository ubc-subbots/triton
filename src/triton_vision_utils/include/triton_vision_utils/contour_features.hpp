#ifndef TRITON_VISION_UTILS__CONTOUR_FEATURES
#define TRITON_VISION_UTILS__CONTOUR_FEATURES

#include <unistd.h>

#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

namespace triton_vision_utils
{
class ContourFeatures
{
public:
  ContourFeatures();

  /**
   * Produces ellipse features of the contour
   * @param cnt A convex hull contour
   * @return A vector containing the major asix (MA), minor axis (ma) and angle of
   *           contour in that order
   */
  std::vector<float> ellispe_features(std::vector<cv::Point> cnt);

  /**
   * Produces area features of the contour
   * @param cnt A convex hull contour
   * @return A vecotr containing the contour area, bounding rect area, aspect ratio.
   */
  std::vector<float> area_features(std::vector<cv::Point> cnt);

  /**
   * Produces min area features of the contour
   * @param cnt A convex hull contour
   * @return A vector of vectors of cv::Point: min area rect, min area triangle,
   */
  std::vector<std::vector<cv::Point>> min_area_features(std::vector<cv::Point> cnt);

  /**
   * Produces min area circle radius of the contour
   * @param cnt A convex hull contour
   * @return radius of min circ
   */
  float min_area_feature_circ(std::vector<cv::Point> cnt);

  /**
   * Produces the log of the hu moment features of the contour
   * @param cnt A convex hull contour
   * @return A vector of 7 hu moments
   */
  std::vector<float> hu_moments_featurize(std::vector<cv::Point> cnt);
};
}  // namespace triton_vision_utils

#endif  // TRITON_VISION_UTILS__CONTOUR_FEATURES