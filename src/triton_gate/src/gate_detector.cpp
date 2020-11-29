#include "triton_gate/gate_detector.hpp"
#include "triton_gate/pole_featurizer.hpp"

using namespace std;
using namespace cv;
using namespace cv::ml;
using namespace triton_vision_utils;

namespace triton_gate
{
GateDetector::GateDetector(const rclcpp::NodeOptions& options) : Node("gate_detector", options)
{
  float im_resize = 1.0;
  bool _debug = true;
  float focal = 400.0;
  // TODO: parameterize img_resize, debug, focal
  ObjectDetector(im_resize, _debug, focal);
  debug = _debug;
  gate_cntr = {};
  gate_dims = Size(1.2192, 3.2004);
  estimated_poses = {};
  frame_count = 0;
  gate_pose = tuple<float, float, float, float, float, float>(0, 0, 0, 0, 0, 0);
  featurizer = PoleFeaturizer();
  getcwd(directorybuf, 64);
  string directory(directorybuf);
  RCLCPP_INFO(this->get_logger(), "Gate Detector succesfully started!");
}

cv::Mat GateDetector::detect(cv::Mat src)
{
  pre = preprocess(src);
  enh = enhance(pre, 0, 0, 0, 1);
  seg = morphological(segment(enh), Size(1, 1), Size(1, 1));
  hulls = convex_hulls(seg, 1.0 / 4, 1.0 / 800);
  bound = bound_gate_using_poles(hulls, src);
  return bound;
}

cv::Mat GateDetector::segment(cv::Mat src)
{
  // Convert to HSV color space
  Mat hsv;
  cvtColor(src, hsv, COLOR_BGR2HSV);

  // Compute gradient threshold on saturation channel of HSV image
  // (seems to have best response)
  ObjectDetector objdtr = ObjectDetector(0.5, true, 400);
  Mat hsvChannels[3];
  split(hsv, hsvChannels);
  Mat grad = objdtr.gradient(hsvChannels[1]);
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
  return segmented;
}

cv::Mat GateDetector::bound_gate_using_poles(std::vector<std::vector<Point>> hulls, cv::Mat src)
{
  // ignore featurize first, let all hulls be pole hulls
  // Resize src to match the image the hulls were found on
  // resize(src, src, seg.size(), 0,0,INTER_CUBIC);

  // We can't do anything if we aren't given any hulls
  if (hulls.size() == 0)
  {
    return src;
  }

  // Featurize hulls, predict using model and get classified pole hulls
  vector<vector<Point>> pole_hulls;
  Ptr<SVM> svm = SVM::load("/home/jared/subbot/triton/accuracy_opencv_model.xml");
  vector<float> y_hat;
  Mat X_hat = featurizer.featurize_for_classification(hulls);
  svm->predict(X_hat, y_hat);
  for (int i = 0; i < hulls.size(); i++)
  {
    if (y_hat.at(i) == 1)
    {
      pole_hulls.push_back(hulls.at(i));
    }
  }

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
    vector<Point> _gate_cntr = create_gate_contour(hull_points, src);

    // Get bounding box of contour to get it's approximate width/height
    Rect box = boundingRect(_gate_cntr);
    // Rect box = boundingRect(hull_points);
    int w = box.width;
    int h = box.height;

    // If the bounding rectangle is more wide than high, most likely we have detected both poles
    if ((float)w / h >= 1)
    {
      if (gate_cntr.size() == 0)
      {
        // We make the strong assumption that the first time we detect the gate, it is accurate
        gate_cntr = _gate_cntr;
      }
      else
      {
        // we make sure the area hasn't changed too much (50%) between frames to account for outliers
        vector<Point> prev_cntr = gate_cntr;
        double prev_area = contourArea(prev_cntr);
        double curr_area = contourArea(_gate_cntr);
        if (curr_area <= 1.5 * prev_area && curr_area >= 0.5 * prev_area)
        {
          gate_cntr = _gate_cntr;
        }
      }
    }
  }
  // Draw the gate if we have detected it
  if (gate_cntr.size() != 0)
  {
    polylines(src, gate_cntr, true, Scalar(0, 0, 255), 2);
  }

  // Draw all non pole hulls and pole hulls on src for debug purposes
  if (debug)
  {
    polylines(src, hulls, true, Scalar(0, 0, 255), 2);
    polylines(src, pole_hulls, true, Scalar(255, 255, 255), 2);
  }
  return src;
}

std::vector<Point> GateDetector::create_gate_contour(std::vector<Point> hull_points, cv::Mat src)
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

  if (debug)
  {
    circle(src, top_left, 8, Scalar(0, 128, 0), 4);
    circle(src, top_right, 8, Scalar(0, 128, 0), 4);
    circle(src, bot_left, 8, Scalar(0, 128, 0), 4);
    circle(src, bot_right, 8, Scalar(0, 128, 0), 4);
  }
  gate_cntr.push_back(top_left);
  gate_cntr.push_back(top_right);
  gate_cntr.push_back(bot_right);
  gate_cntr.push_back(bot_left);
  return gate_cntr;
}

}  // namespace triton_gate
