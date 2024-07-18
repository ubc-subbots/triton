#ifndef TRITON_VISION_UTILS__OBJECT_DETECTOR
#define TRITON_VISION_UTILS__OBJECT_DETECTOR

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace triton_vision_utils
{
class ObjectDetector
{
public:
  /**
   *  Initializes an object detector
   *
   * @param input_im_resize The scale to resize the image to, default: 1.0
   * @param input_debug If True, adds debug information to output, default: false
   * @param input_focal default: 400.0
   */
  explicit ObjectDetector(float _im_resize = 1, bool _debug = false, float _focal = 400.0);

  /**
   * Detects an object from a raw image, must be implemented by sub classes
   *
   * @param src Raw image
   * @return Three images representing the algorithms at various stages. The last
   *          image must always be the final output fo the algorithm.
   */
  // virtual cv::Mat detect(cv::Mat src)=0;

  /**
   * Preprocess the source image by blurring and resizing
   *
   * @param src A raw unscaled image
   * @return The preprocessed and scaled image
   */
  cv::Mat preprocess(cv::Mat src);

  /**
   * Enhances a raw image to account for underwater artifacts affecting contrast, hue and saturation.
   * Performs CLAHE on the given input color spaces then blends the equally weighted result across
   * all color spaces used.
   * 
   * @param src A preprocessed image
   * @param clahe_clr_space_bgr 1 means to use this CLAHE color space
   * @param clahe_clr_space_hsv 1 means to use this CLAHE color space
   * @param clahe_clr_space_lab 1 means to use this CLAHE color space
   * @param clahe_clip_limt The limit at which CLAHE clips the contrast to prevent over-contrasting
   * @return An enhanced image
   */
  cv::Mat enhance(cv::Mat src, int clahe_clr_space_bgr = 1, int clahe_clr_space_hsv = 1, int clahe_clr_space_lab = 1,
                  int clahe_clip_limit = 1);

  /**
   * Computes the sobel gradient for a source image
   *
   * @param src A grayscale image
   * @return The sobel gradient response of the image
   */
  cv::Mat gradient(cv::Mat src);

  /**
   * Smooths a binary image with morphological operations
   *
   * @param src A binary image
   * @param open_kernel Opening kernel dimensions
   * @param close_kernel Closing kernel dimensions
   */
  cv::Mat morphological(cv::Mat src, cv::Size open_kernel = cv::Size(1, 1), cv::Size close_kernel = cv::Size(1, 1));

  /**
   * Creates a set of convex hulls from the binary segmented image and which are of
   * an appropriate size based on upper and lower area thresholds
   * 
   * @param src A binary segmented grayscale image
   * @param upper_area Upper threshold of area filter
   * @param lower_area Lower threshold of area filter
   */
  std::vector<std::vector<cv::Point>> convexHulls(cv::Mat& src, float upper_area = 1.0 / 2,
                                                   float lower_area = 1.0 / 1000);

private:
  float im_resize = 1.0;
  bool debug = false;
  float focal = 400.0;
  cv::Size im_dims;
  cv::Mat curr_image;
};

}  // namespace triton_vision_utils

#endif  // TRITON_VISION_UTILS__OBJECT_DETECTOR