#include <iostream>
#include <unistd.h>
//#include <filesystem> //for filepath, can be used in C++17
#include <opencv2/opencv.hpp>
#include "triton_vision_utils/vision_utils.hpp"
using namespace std;
//namespace fs = std::filesystem;
using namespace cv;
using namespace cv::ml;
using namespace vision_utils;

class PoleFeaturizer
{
Private:
    ContourFeatures cnt_features;
public:
    PoleFeaturizer()
    {
        cnt_features = ContourFeatures();
    }

    vector<int> featurize_for_classification(vector<vector<Point>> hulls)
    {
        vector<int> X_hat;
        return X_hat;
    }

    vector<int> form_feature_vector(vector<Point> hull)
    {
        vector<int> x;
        return x;
    }

    vector<int> scale_data(vector<int> X, bool scale=false)
    {
        return X;
    }
};

/**
 *  A class for contour features
 */
class ContourFeatures
{
public:
    ContourFeatures()
    {

    }

    /**
     * Produces ellipse features of the contour
     * @param cnt: A convex hull contour
     * @returns: A vector containing the major asix (MA), minor axis (ma) and angle of
     *           contour in that order
     */
    vector<float> ellispe_features(vector<Point> cnt)
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
        vf.push_back(angle);
        vf.push_back(MA);
        vf.push_back(ma);
        return vf;
        
    }

    /**
     * Produces area features of the contour
     * @param cnt: A convex hull contour
     * @returns: A vecotr containing the contour area, bounding rect area, aspect ratio.
     */
    vector<float> area_features(vector<Point> cnt)
    {
        int cnt_area = contourArea(cnt);
        Rect boundingR = boundingRect(cnt);
        int rect_area = boundingR.area();
        float aspect_ratio = (float) boundingR.width / boundingR.height;
        vector<float> vf;
        vf.push_back(cnt_area);
        vf.push_back(rect_area);
        vf.push_back(aspect_ratio);
        return vf;
    }

    void min_area_features(vector<Point> cnt)
    {

    }

    void hu_moments_featurize(vector<Point> cnt)
    {

    }

};