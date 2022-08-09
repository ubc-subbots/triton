#ifndef TRITON_GATE__GATE_DETECTOR
#define TRITON_GATE__GATE_DETECTOR

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
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
  void detect(const sensor_msgs::msg::Image & msg);

private:

  /** Callback used by subscriber to process and publish results
   */
  void subscriberCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

  /**
   * Segment the image using thresholded saturation gradient and orange/red color mask
   *
   * @param src A preprocessed image
   * @return A segmented grayscale image
   */
  cv::Mat segment(cv::Mat& src);

  /**
   * Finds the convex hulls associated to the poles and uses this to draw a bounding box around the poles
   * of the gate onto the raw image
   *
   * @param hulls A set of the convex hulls to search
   * @param src The raw unscaled image
   * @return The raw scaled image with the bounding box around the gate location drawn on
   */
  void boundGateUsingPoles(std::vector<std::vector<cv::Point>> hulls, cv::Mat& src);

  /**
   * Creates the estimated gate contour from the given hull points, draws debug info on src if activated
   *
   * @param hull_points 2D array of points
   * @param src The raw image
   * @return The gate contour drawn on the src image with debug info if activated
   */
  std::vector<cv::Point> createGateContour(std::vector<cv::Point> hull_points, cv::Mat& src);

  /** Helper for publishing debug message
   * 
   * @param src The debug image to publish
   * @param publisher The publisher to use to publish the image
   */
  void debugPublish(cv::Mat& src, image_transport::Publisher& publisher);

  /** Function to store code for using OpenCV SVM 
   * 
   */
  void svmHullPredict(std::vector<std::vector<Point>> hulls);

  bool debug_;
  std::vector<cv::Point> gate_cntr_;
  image_transport::Subscriber subscription_; 
  image_transport::Publisher debug_segment_publisher_;
  image_transport::Publisher debug_detection_publisher_;
  triton_gate::PoleFeaturizer featurizer_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr gate_center_publisher_;  
};

}  // namespace triton_gate

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(triton_gate::GateDetector)

#endif  // TRITON_GATE__GATE_DETECTOR