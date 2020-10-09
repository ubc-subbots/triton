#include <iostream>
#include <opencv2/core/mat.hpp>

#include "triton_vision_utils/vision_utils.hpp"

int main()
{
  cv::Mat matrix;
  int z = vision_utils::add(5,3);
  std::cout<<z<<std::endl;

  return 0;
}
