#include "triton_vision_utils/object_detector.hpp"

using namespace cv;
using namespace std;

namespace triton_vision_utils
{
ObjectDetector::ObjectDetector(float _im_resize, bool _debug, float _focal)
{
  im_resize = _im_resize;
  debug = _debug;
  focal = _focal;
}

Mat ObjectDetector::preprocess(Mat src)
{
  im_dims = Size(src.rows * im_resize, src.cols * im_resize);
  if (im_resize != 1.0)
  {
    Mat srcSplit[3];
    split(src, srcSplit);
    Size kernel = Size(3, 3);
    int sig = 1;
    GaussianBlur(srcSplit[0], srcSplit[0], kernel, sig);
    GaussianBlur(srcSplit[1], srcSplit[1], kernel, sig);
    GaussianBlur(srcSplit[2], srcSplit[2], kernel, sig);
    merge(srcSplit, 3, src);
    resize(src, src, im_dims);
    curr_image = src;
  }
  return src;
}

Mat ObjectDetector::enhance(Mat src, int clahe_clr_space_bgr, int clahe_clr_space_hsv, int clahe_clr_space_lab,
                            int clahe_clip_limit)
{
  Ptr<CLAHE> clahe = createCLAHE();
  clahe->setTilesGridSize(Size(11, 11));
  clahe->setClipLimit(clahe_clip_limit);
  Mat parts[3];
  int partsLen = 0;

  if (clahe_clr_space_bgr == 1)
  {
    Mat bgr[3];
    split(src, bgr);
    clahe->apply(bgr[0], bgr[0]);
    clahe->apply(bgr[1], bgr[1]);
    clahe->apply(bgr[2], bgr[2]);
    Mat bgr_clahe;
    merge(bgr, 3, bgr_clahe);
    parts[partsLen] = bgr_clahe;
    partsLen++;
  }
  if (clahe_clr_space_lab == 1)
  {
    Mat lab[3];
    Mat labMat;
    cvtColor(src, labMat, COLOR_BGR2YCrCb);
    split(labMat, lab);
    clahe->apply(lab[0], lab[0]);
    clahe->apply(lab[1], lab[1]);
    clahe->apply(lab[2], lab[2]);
    Mat lab_clahe;
    merge(lab, 3, lab_clahe);
    cvtColor(lab_clahe, parts[partsLen], COLOR_YCrCb2BGR);
    partsLen++;
  }
  if (clahe_clr_space_hsv == 1)
  {
    Mat hsv[3];
    Mat hsvMat;
    cvtColor(src, hsvMat, COLOR_BGR2HSV);
    split(hsvMat, hsv);
    clahe->apply(hsv[0], hsv[0]);
    clahe->apply(hsv[1], hsv[1]);
    clahe->apply(hsv[2], hsv[2]);
    Mat hsv_clahe;
    merge(hsv, 3, hsv_clahe);
    cvtColor(hsv_clahe, parts[partsLen], COLOR_HSV2BGR);
    partsLen++;
  }
  if (partsLen > 0)
  {
    Mat blended(parts[0].rows, parts[0].cols, 16, Scalar::all(0));
    for (int i = 0; i < partsLen; i++)
    {
      Mat p;
      parts[i].convertTo(p, 0);
      blended += p;
    }
    blended /= partsLen;
    src = blended;
  }
  return src;
}

Mat ObjectDetector::gradient(Mat src)
{
  int scale = 1;
  int delta = 0;
  int ddepth = CV_16S;
  Mat grad_x, abs_grad_x, grad_y, abs_grad_y;
  Sobel(src, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT);
  Sobel(src, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT);
  convertScaleAbs(grad_x, abs_grad_x, scale, 0);
  convertScaleAbs(grad_y, abs_grad_y, scale, 0);
  Mat grad;
  addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);
  return grad;
  // Note: original python version uses np expand_dims(..., axis=2)
}

Mat ObjectDetector::morphological(Mat src, Size open_kernel, Size close_kernel)
{
  Mat open_k = getStructuringElement(MORPH_RECT, open_kernel);
  Mat close_k = getStructuringElement(MORPH_RECT, close_kernel);
  Mat opening, closing;
  morphologyEx(src, opening, MORPH_OPEN, open_k);
  morphologyEx(opening, closing, MORPH_CLOSE, close_k);
  return closing;
}

vector<vector<Point>> ObjectDetector::convexHulls(Mat& src, float upper_area, float lower_area)
{
  vector<vector<Point>> hulls;
  int hullIndex = 0;
  vector<vector<Point>> right_size_hulls;
  int right_s_h_index = 0;

  // Find contours in the image
  vector<vector<Point>> contours;
  findContours(src, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

  // Create a convex hull around each connected contour
  for (vector<Point> j : contours)
  {
    vector<Point> h;
    convexHull(j, h, false);
    hulls.push_back(h);
  }

  // Get the hulls whose area is within some threshold range
  for (vector<Point> hull : hulls)
  {
    if (hull.size() > 0)
    {
      double hull_area = contourArea(hull);
      float im_size = src.cols * src.rows;
      if (hull_area > im_size * lower_area && hull_area < im_size * upper_area)
      {
        right_size_hulls.push_back(hull);
      }
    }
  }
  vector<vector<Point>> right_size_vector_of_hulls;
  for (int i = 0; i < right_size_hulls.size(); i++)
  {
    right_size_vector_of_hulls.push_back(right_size_hulls.at(i));
  }
  return right_size_vector_of_hulls;
}

}  // namespace triton_vision_utils
