#include <iostream>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include "triton_vision_utils/vision_utils.hpp"
using namespace std;
using namespace cv;
using namespace cv::ml;
using namespace vision_utils;

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
        vf.push_back(MA);
        vf.push_back(ma);
        vf.push_back(angle);
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

    /**
     * Produces min area features of the contour
     * @param cnt: A convex hull contour
     * @returns: A vector of vectors of Point: min area rect, min area triangle, 
     */
    vector<vector<Point>> min_area_features(vector<Point> cnt)
    {
        //cout << "before min rect" << endl;
        RotatedRect min_rect_all_points = minAreaRect(cnt);
        vector<Point> min_rect, min_tri;
        //cout << "before box points" << endl;
        Point2f vtx[4];
        min_rect_all_points.points(vtx); //replaces boxPoints
        for (Point2f p : vtx)
        {
            min_rect.push_back(p);
        }
        //boxPoints(min_rect_all_points, min_rect);
        //cout << "before min tri" << endl;
        minEnclosingTriangle(cnt, min_tri);
        vector<vector<Point>> returnVec;
        returnVec.push_back(min_rect);
        returnVec.push_back(min_tri);
        return returnVec;

    }
    /**
     * Produces min area circle radius of the contour
     * @param cnt: A convex hull contour
     * @returns: radius of min circ 
     */
    float min_area_feature_circ(vector<Point> cnt)
    {
        Point2f center;
        float radius = 0;
        //cout << "before enclosing circ" << endl;
        minEnclosingCircle(cnt, center, radius);
        return radius;
    }

    /**
     * Produces the log of the hu moment features of the contour
     * @param cnt: A convex hull contour
     * @returns: A vector of 7 hu moments
     */
    vector<float> hu_moments_featurize(vector<Point> cnt)
    {
        //https://www.learnopencv.com/shape-matching-using-hu-moments-c-python/
        Moments moments = cv::moments(cnt);
        double huMoments[7];
        //cout <<"before humoments" << endl;
	    HuMoments(moments, huMoments);
        //cout <<"before log" << endl;
        for(int i = 0; i < 7; i++)
        {
        huMoments[i] = -1 * copysign(1.0, huMoments[i]) * log10(abs(huMoments[i])); 
        }
        vector<float> hu;
        //cout <<"before pushing" << endl;
        for(int i = 0; i < 7; i++)
        {
            hu.push_back((float) huMoments[i]);
        }
        return hu;
    }

};

/**
 * A class for constructing pole features for building models and classification
 */
class PoleFeaturizer
{
private:
    ContourFeatures cnt_features;
public:
    PoleFeaturizer()
    {
        cnt_features = ContourFeatures();
    }

    /**
     * Featurizes the hulls and returns the feature matrix X_hat
     * @param hulls: The convex hulls to be featurized
     * @returns: The feature matrix X_hat
     */
    Mat featurize_for_classification(vector<vector<Point>> hulls)
    {
        Mat X_hat;
        for (vector<Point> hull : hulls)
        {
            X_hat.push_back(Mat(form_feature_vector(hull)).reshape(0,1)); // so each feature vector is a row
        }
        X_hat.convertTo(X_hat, CV_32F); // to make sure the type is correct
        return X_hat;
    }

    /**
     * Forms the feature vector from a convex hull
     * @param hulls: The convex hulls to be featurized
     * @returns: The feature vector of the hull
     */
    vector<float> form_feature_vector(vector<Point> hull)
    {
        vector<float> features;

        //cout << "before ellipse" << endl ;
        vector<float> ellipse_feat = cnt_features.ellispe_features(hull);
        float MA = ellipse_feat.at(0);
        float ma = ellipse_feat.at(1);
        float angle = ellipse_feat.at(2);

        //cout << "before area" << endl ;
        vector<float> area_feat = cnt_features.area_features(hull);
        float hull_area = area_feat.at(0);
        float rect_area = area_feat.at(1);
        float aspect_ratio = area_feat.at(2);

        //cout << "before min area" << endl ;
        vector<vector<Point>> min_area_feat = cnt_features.min_area_features(hull);
        vector<Point> min_rect = min_area_feat.at(0);
        vector<Point> min_tri = min_area_feat.at(1);
        float min_circ_rad = cnt_features.min_area_feature_circ(hull);

        //cout << "before hu moment" << endl ;
        vector<float> hu_moments = cnt_features.hu_moments_featurize(hull);

        float axis_ratio = MA/ma;
        angle = abs(sin(angle * M_PI/180));
        float extent = contourArea(min_rect)/hull_area;

        // Even though we calculate more features, from testing we find these work the best
        features.push_back(axis_ratio);
        features.push_back(aspect_ratio);
        features.push_back(extent);
        features.insert(features.end(), hu_moments.begin(), hu_moments.end());

        return features;
    }

    vector<vector<float>> scale_data(vector<vector<float>> X, bool scale=false)
    {
        if (scale)
        {
            //do scaling
        }
        return X;
    }
};
