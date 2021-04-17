#ifndef TRITON_GATE__POLE_FEATURIZER
#define TRITON_GATE__POLE_FEATURIZER

#include <unistd.h>

#include <iostream>
#include <opencv2/opencv.hpp>

#include "triton_vision_utils/contour_features.hpp"

namespace triton_gate
{
  
using namespace std;
using namespace cv;
using namespace cv::ml;

class PoleFeaturizer
{
private:
  triton_vision_utils::ContourFeatures cnt_features;

public:
  PoleFeaturizer()
  {
    cnt_features = triton_vision_utils::ContourFeatures();
  }

  /**
   * Featurizes the hulls and returns the feature matrix X_hat
   * @param hulls The convex hulls to be featurized
   * @return The feature matrix X_hat
   */
  Mat featurizeForClassification(vector<vector<Point>> hulls)
  {
    int vecNum = 0;
    int cols = 10;
    int rows = hulls.size();
    float x_array[rows][10];
    for (vector<Point> hull : hulls)
    {
      vector<float> feat = formFeatureVector(hull);
      for (int feature = 0; feature < 10; feature++)
      {
        x_array[vecNum][feature] = feat.at(feature);
      }
      vecNum++;
    }
    Mat X_hat = Mat(rows, 10, CV_32F, &x_array);
    return X_hat;
  }

  /**
   * Forms the feature vector from a convex hull
   * @param hulls The convex hulls to be featurized
   * @return The feature vector of the hull
   */
  vector<float> formFeatureVector(vector<Point> hull)
  {
    vector<float> features;

    vector<float> ellipse_feat = cnt_features.ellispe_features(hull);
    float MA = ellipse_feat.at(0);
    float ma = ellipse_feat.at(1);
    float angle = ellipse_feat.at(2);

    vector<float> area_feat = cnt_features.area_features(hull);
    float hull_area = area_feat.at(0);
    float rect_area = area_feat.at(1);
    float aspect_ratio = area_feat.at(2);

    vector<vector<Point>> min_area_feat = cnt_features.min_area_features(hull);
    vector<Point> min_rect = min_area_feat.at(0);
    vector<Point> min_tri = min_area_feat.at(1);
    float min_circ_rad = cnt_features.min_area_feature_circ(hull);

    vector<float> hu_moments = cnt_features.hu_moments_featurize(hull);

    float axis_ratio = MA / ma;
    angle = abs(sin(angle * M_PI / 180));
    float extent = contourArea(min_rect) / hull_area;

    // Even though we calculate more features, from testing we find these work the best
    features.push_back(axis_ratio);
    features.push_back(aspect_ratio);
    features.push_back(extent);
    features.insert(features.end(), hu_moments.begin(), hu_moments.end());

    return features;
  }
};

}  // namespace triton_gate

#endif  // TRITON_GATE__POLE_FEATURIZER
