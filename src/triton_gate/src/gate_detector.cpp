#include "triton_gate/gate_detector.hpp"
#include "triton_gate/pole_featurizer.hpp"
#include <string> 

using namespace std;
using namespace cv;
using namespace cv::ml;
using namespace triton_vision_utils;
using std::placeholders::_1;

namespace triton_gate
{
GateDetector::GateDetector(const rclcpp::NodeOptions& options) : Node("gate_detector", options)
{
  debug_ = true;
  ObjectDetector(1.0, debug_, 400.0);
  gate_cntr_ = {};
  featurizer_ = PoleFeaturizer();
  
  rmw_qos_profile_t sensor_qos_profile = rmw_qos_profile_sensor_data;

  subscription_ = image_transport::create_subscription(this,
      "/triton/drivers/front_camera/image_raw",
      bind(&GateDetector::subscriberCallback, this, _1),
      "raw",
      sensor_qos_profile
  );

  debug_detection_publisher_ = image_transport::create_publisher(this, "detector/debug/detection");
  debug_segment_publisher_ = image_transport::create_publisher(this, "detector/debug/segment");

  RCLCPP_INFO(this->get_logger(), "GateDetector succesfully started!");
}

void GateDetector::subscriberCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  detect(*msg);
}

void GateDetector::detect(const sensor_msgs::msg::Image & msg)
{
  //Get image from message
  cv_bridge::CvImagePtr cv_ptr;
  try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(get_logger(),"cv_bridge exception: %s", e.what());
      return;
  }

  cv::Mat detection = cv_ptr->image;
  cv::Mat segmented = morphological(segment(detection), Size(3, 3), Size(3, 3));
  std::vector<std::vector<cv::Point>> hulls = convexHulls(segmented, detection.rows*detection.cols, 0);
  boundGateUsingPoles(hulls, detection);

  if (debug_)
  {
    debugPublish(detection, debug_detection_publisher_);
    debugPublish(segmented, debug_segment_publisher_);
  }
}

cv::Mat GateDetector::segment(cv::Mat& src)
{
  // Convert to HSV color space
  Mat hsv;
  cvtColor(src, hsv, COLOR_BGR2HSV);

  // Compute gradient threshold on saturation channel of HSV image (seems to have best response)
  Mat hsvChannels[3];
  split(hsv, hsvChannels);
  Mat grad = gradient(hsvChannels[1]);
  return grad;
  Scalar grad_mean, grad_std;
  meanStdDev(grad, grad_mean, grad_std);
  Mat grad_thres;
  // Since grad should be single-channel, grad_mean[0] and grad_std[0] should contain the values we need
  threshold(grad, grad_thres, (double)(grad_mean[0]) + 4 * grad_std[0], 255, THRESH_BINARY);

  // Orange/red hue mask
  Mat upper_hue_mask, lower_hue_mask, color_mask;
  inRange(hsvChannels[0], 175, 180, upper_hue_mask);
  inRange(hsvChannels[0], 0, 30, lower_hue_mask);
  bitwise_or(upper_hue_mask, lower_hue_mask, color_mask);

  // Combine the two binary image, note that the gradient threshold has the best response from farther away
  // and the color mask works best at close distances, so by combining them, we have an image that produces
  // a great response to the poles at all distances to them
  Mat segmented;
  bitwise_or(grad_thres, color_mask, segmented);
  vector<int> compression_params;
  return segmented;
}

void GateDetector::boundGateUsingPoles(std::vector<std::vector<Point>> hulls, cv::Mat& src)
{
  // We can't do anything if we aren't given any hulls
  if (hulls.size() == 0)
  {
    return;
  }

  // Featurize hulls, predict using model and get classified pole hulls
  vector<vector<Point>> pole_hulls;
  for (int i=0; i< (int) hulls.size(); i++)
  {
    vector<Point> hull = hulls.at(i);
    float aspect_ratio = 1.0;
    if (hull.size() >= 3)
    {
      Rect boundingR = boundingRect(hull);
      aspect_ratio = (float) boundingR.width / boundingR.height ;
    }
    float thresh = 0.03;
    float desired_ratio = 0.11;
    if (aspect_ratio > desired_ratio - thresh && aspect_ratio < desired_ratio + thresh)
    {
      pole_hulls.push_back(hull);
    }
  }
  // Ptr<SVM> svm = SVM::load("/home/logan/Projects/triton/src/triton_gate/config/accuracy_opencv_model.xml");
  // vector<float> y_hat;
  // Mat X_hat = featurizer_.featurizeForClassification(hulls);
  // svm->predict(X_hat, y_hat);
  // for (int i = 0; i < hulls.size(); i++)
  // {
  //   if (y_hat.at(i) == 1)
  //   {
  //     pole_hulls.push_back(hulls.at(i));
  //   }
  // }

  // Get 2D array of all the points of the pole hulls (to determine extrema)
  vector<Point> hull_points;
  for (vector<Point> h : pole_hulls)
  {
    for (Point p : h)
    {
      hull_points.push_back(p);
    }
  }

  // If we have detected a hull associated to a pole
  if (hull_points.size() > 0)
  {
    vector<Point> gate_cntr = createGateContour(hull_points, src);

    // Get bounding box of contour to get it's approximate width/height
    Rect box = boundingRect(gate_cntr);

    int w = box.width;
    int h = box.height;

    // If the bounding rectangle is more wide than high, most likely we have detected both poles
    if ((float)w / h >= 1)
    {
      gate_cntr_ = gate_cntr;
    }
  }
  // Draw the gate if we have detected it
  if (gate_cntr_.size() != 0)
  {
    polylines(src, gate_cntr_, true, Scalar(0, 0, 255), 2);
  }

  // Draw all non pole hulls and pole hulls on src for debug purposes
  if (debug_)
  {
    polylines(src, hulls, true, Scalar(255, 255, 255), 1);
    polylines(src, pole_hulls, true, Scalar(0, 128, 256), 1);
  }
  return;
}

std::vector<Point> GateDetector::createGateContour(std::vector<Point> hull_points, cv::Mat& src)
{
  int width = src.cols;

  // Get extrema points of hulls (i.e the points closest/furthest from the top left (0,0) and top right (width, 0) of
  // the image)
  Point top_left = *min_element(hull_points.begin(), hull_points.end(), [](Point a, Point b) {
    return (sqrt(pow(a.x, 2) + pow(a.y, 2)) < sqrt(pow(b.x, 2) + pow(b.y, 2)));
  });
  Point top_right = *min_element(hull_points.begin(), hull_points.end(), [width](Point a, Point b) {
    return (sqrt(pow(a.x - width, 2) + pow(a.y, 2)) < sqrt(pow(b.x - width, 2) + pow(b.y, 2)));
  });
  Point bot_left = *max_element(hull_points.begin(), hull_points.end(), [width](Point a, Point b) {
    return (sqrt(pow(a.x - width, 2) + pow(a.y, 2)) < sqrt(pow(b.x - width, 2) + pow(b.y, 2)));
  });
  Point bot_right = *max_element(hull_points.begin(), hull_points.end(), [](Point a, Point b) {
    return (sqrt(pow(a.x, 2) + pow(a.y, 2)) < sqrt(pow(b.x, 2) + pow(b.y, 2)));
  });

  if (debug_)
  {
    circle(src, top_left, 8, Scalar(0, 128, 0), 4);
    circle(src, top_right, 8, Scalar(0, 128, 0), 4);
    circle(src, bot_left, 8, Scalar(0, 128, 0), 4);
    circle(src, bot_right, 8, Scalar(0, 128, 0), 4);
  }
  std::vector<cv::Point> gate_cntr;
  gate_cntr.push_back(top_left);
  gate_cntr.push_back(top_right);
  gate_cntr.push_back(bot_right);
  gate_cntr.push_back(bot_left);
  return gate_cntr;
}

void GateDetector::debugPublish(cv::Mat& src, image_transport::Publisher& publisher)
{
    sensor_msgs::msg::Image debug_msg;
    cv_bridge::CvImage debug_image;
    cv::Size sz = src.size();
    debug_image.image = src;
    debug_image.toImageMsg(debug_msg);
    debug_msg.width = sz.width;
    debug_msg.height = sz.height;
    debug_msg.encoding = src.channels() == 1 ? sensor_msgs::image_encodings::MONO8 : sensor_msgs::image_encodings::BGR8;
    debug_msg.step = sz.width*(src.channels());
    debug_msg.header.frame_id = "base_link";
    publisher.publish(debug_msg);
}

}  // namespace triton_gate
