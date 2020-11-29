#include "triton_vision_utils/contour_features.hpp"

using namespace cv;
using namespace std;

namespace triton_vision_utils
{
ContourFeatures::ContourFeatures()
{
}

vector<float> ContourFeatures::ellispe_features(vector<Point> cnt)
{
  float angle = 0;
  int MA = 1;
  int ma = 1;
  if (cnt.size() >= 3)
  {
    RotatedRect rr = fitEllipse(cnt);
    angle = rr.angle;
    MA = rr.size.height;
    ma = rr.size.width;
  }
  vector<float> vf;
  vf.push_back(MA);
  vf.push_back(ma);
  vf.push_back(angle);
  return vf;
}

vector<float> ContourFeatures::area_features(vector<Point> cnt)
{
  int cnt_area = contourArea(cnt);
  Rect boundingR = boundingRect(cnt);
  int rect_area = boundingR.area();
  float aspect_ratio = (float)boundingR.width / boundingR.height;
  vector<float> vf;
  vf.push_back(cnt_area);
  vf.push_back(rect_area);
  vf.push_back(aspect_ratio);
  return vf;
}

vector<vector<Point>> ContourFeatures::min_area_features(vector<Point> cnt)
{
  // cout << "before min rect" << endl;
  RotatedRect min_rect_all_points = minAreaRect(cnt);
  vector<Point> min_rect, min_tri;
  // cout << "before box points" << endl;
  Point2f vtx[4];
  min_rect_all_points.points(vtx);  // replaces boxPoints
  for (Point2f p : vtx)
  {
    min_rect.push_back(p);
  }
  // boxPoints(min_rect_all_points, min_rect);
  // cout << "before min tri" << endl;
  minEnclosingTriangle(cnt, min_tri);
  vector<vector<Point>> returnVec;
  returnVec.push_back(min_rect);
  returnVec.push_back(min_tri);
  return returnVec;
}

float ContourFeatures::min_area_feature_circ(vector<Point> cnt)
{
  Point2f center;
  float radius = 0;
  // cout << "before enclosing circ" << endl;
  minEnclosingCircle(cnt, center, radius);
  return radius;
}

vector<float> ContourFeatures::hu_moments_featurize(vector<Point> cnt)
{
  // https://www.learnopencv.com/shape-matching-using-hu-moments-c-python/
  Moments moments = cv::moments(cnt);
  double huMoments[7];
  // cout <<"before humoments" << endl;
  HuMoments(moments, huMoments);
  // cout <<"before log" << endl;
  for (int i = 0; i < 7; i++)
  {
    huMoments[i] = -1 * copysign(1.0, huMoments[i]) * log10(abs(huMoments[i]));
  }
  vector<float> hu;
  // cout <<"before pushing" << endl;
  for (int i = 0; i < 7; i++)
  {
    hu.push_back((float)huMoments[i]);
  }
  return hu;
}

}  // namespace triton_vision_utils
