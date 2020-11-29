#ifndef TRITON_GATE__GATE_DETECTOR
#define TRITON_GATE__GATE_DETECTOR

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "triton_gate/pole_featurizer.hpp"
#include "triton_vision_utils/object_detector.hpp"

namespace triton_gate
{
class GateDetector : public rclcpp::Node, public triton_vision_utils::ObjectDetector
{
public:
  GateDetector(const rclcpp::NodeOptions& options);

  /**
   * Detects the gate in a raw image and returns the images associated to the stages
   * of the algorithm.
   *
   * @param src Raw underwater image containing the gate
   * @return Image associated to bounding and posing.
   */
  cv::Mat detect(cv::Mat src);

private:
  /**
   * Segment the image using thresholded saturation gradient and orange/red color mask
   *
   * @param src A preprocessed image
   * @return A segmented grayscale image
   */
  cv::Mat segment(cv::Mat src);

  /**
   * Finds the convex hulls associated to the poles and uses this to draw a bounding box around the poles
   * of the gate onto the raw image
   *
   * @param hulls A set of the convex hulls to search
   * @param src The raw unscaled image
   * @return The raw scaled image with the bounding box around the gate location drawn on
   */
  cv::Mat bound_gate_using_poles(std::vector<std::vector<cv::Point>> hulls, cv::Mat src);

  /**
   * Creates the estimated gate contour from the given hull points, draws debug info on src if activated
   *
   * @param hull_points 2D array of points
   * @param src The raw image
   * @return The gate contour drawn on the src image with debug info if activated
   */
  std::vector<cv::Point> create_gate_contour(std::vector<cv::Point> hull_points, cv::Mat src);

  std::vector<cv::Point> gate_cntr;
  cv::Size gate_dims;  // in m
  std::vector<std::tuple<float, float, float, float, float, float>> estimated_poses;
  int frame_count;
  std::tuple<float, float, float, float, float, float> gate_pose;  // x, y, z, phi, theta, psi
  triton_gate::PoleFeaturizer featurizer;
  char directorybuf[64];
  cv::Mat pre;
  cv::Mat enh;
  cv::Mat seg;
  std::vector<std::vector<cv::Point>> hulls;
  cv::Mat bound;
  cv::Mat bound_and_pose;
  bool debug;
};

}  // namespace triton_gate

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(triton_gate::GateDetector)

#endif  // TRITON_GATE__GATE_DETECTOR