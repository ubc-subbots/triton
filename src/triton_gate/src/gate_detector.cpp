#include <iostream>
#include <opencv2/core/mat.hpp>

#include "triton_vision_utils/vision_utils.hpp"
using namespace vision_utils;

int main()
{
  cv::Mat src;
  ObjectDetector objdtr;

  src = cv::imread("~/subbot/underwater-object-detection/images/gate/19.jpg");

  cv::Mat preprocessed = objdtr.preprocess(src);
  cv::imshow("Test", preprocessed);
  cv::waitKey(0);

  cv::Mat enhanced = objdtr.enhance(preprocessed);
  cv::imshow("Test", enhanced);
  cv::waitKey(0);

  cv::Mat gradiented = objdtr.gradient(enhanced);
  cv::imshow("Test", gradiented);
  cv::waitKey(0);

  cv::Mat morphed = objdtr.morphological(gradiented);
  cv::imshow("Test", morphed);
  cv::waitKey(0);

  return 0;
}
