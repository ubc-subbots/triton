#include <iostream>
#include <opencv2/core/mat.hpp>

#include "triton_vision_utils/vision_utils.hpp"
using namespace vision_utils;

int main()
{
  cv::Mat src;
  ObjectDetector objdtr = ObjectDetector(0.5,true,400);

  src = cv::imread("/home/jared/19.jpg");
  cv::imshow("Src", src);

  cv::Mat gradiented = objdtr.gradient(src);
  cv::imshow("Grad", gradiented);

  cv::Mat morphed = objdtr.morphological(gradiented);
  cv::imshow("Mor", morphed);
  cv::waitKey(0);

  return 0;
}
