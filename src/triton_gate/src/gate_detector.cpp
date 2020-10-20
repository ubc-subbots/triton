#include <iostream>
#include <unistd.h>
//#include <filesystem> //for filepath, can be used in C++17
#include <opencv2/opencv.hpp>
#include "triton_vision_utils/vision_utils.hpp"
using namespace std;
//namespace fs = std::filesystem;
using namespace cv;
using namespace vision_utils;

/**
 * Gate detector using iamge segmentation, shape classification, and robust pose estimation.
 */

class GateDetector : public ObjectDetector
{
private:
    array<vector<int>,4> gate_cntr;
    Size gate_dims; // in m
    vector<tuple<float, float, float, float, float, float>> estimated_poses;
    int frame_count;
    tuple<float, float, float, float, float, float> gate_pose; // x, y, z, phi, theta, psi
    //featurizer 
    char directorybuf[64];
    Mat pre;
    Mat enh;
    Mat seg;
    vector<int>* hulls;
    Mat bound;
    Mat bound_and_pose; 


public:
    GateDetector(float im_resize=1.0, bool debug=false, float focal=400)
    {
        ObjectDetector(im_resize, debug, focal);
        gate_cntr = {};
        gate_dims = Size(1.2192, 3.2004);
        estimated_poses = {};
        frame_count = 0;
        gate_pose = tuple<float,float,float,float,float,float>(0,0,0,0,0,0);
    //featurizer = PoleFeaturizer();
        getcwd(directorybuf, 64);
        string directory(directorybuf);
        // load data/model.pkl

    }

    /**
     * Detects the gate in a raw image and returns the images associated to the stages
     * of the algorithm.
     * @param src: Raw underwater image containing the gate
     * @returns: Image associated to bounding and posing.
     */
    Mat detect(Mat src)
    {
        pre = preprocess(src);
        enh = enhance(pre, 0, 0, 0, 1);
        seg = morphological(segment(enh), Size(1,1), Size(1,1));
        hulls = convex_hulls(seg, 1.0/4, 1.0/800);
        bound = bound_gate_using_poles(hulls, src);
        bound_and_pose = estimate_gate_pose(bound);
        return bound_and_pose;
    }

    /**
     * 
     */
    Mat segment(Mat src)
    {
      return src;
    }

    /**
     * 
     */
    Mat bound_gate_using_poles(vector<int>* hulls, Mat src)
    {
      return src;
    }

    /**
     * 
     */
    array<vector<int>,4> create_gate_contour(int hull_points[], Mat src)
    {
      return array<vector<int>,4>{};
    }

    /**
     * 
     */
    Mat estimate_gate_pose(Mat src, int k=5)
    {
      return src;
    }

    /**
     * 
     */
    tuple<float, float, float, float, float, float> calculate_gate_pose(Mat src)
    {
      return tuple<float, float, float, float, float, float>();
    }

   };
/**
 * Main for demo and testing purposes
 */
int main()
{
    cv::Mat src;
    ObjectDetector objdtr = ObjectDetector(0.5, true, 400);

    src = cv::imread("/home/jared/19.jpg");
    cv::imshow("Src", src);
    //cv::imwrite("Src.jpg", src);

    cv::Mat gradiented = objdtr.gradient(src);
    cv::imshow("Grad", gradiented);
    //cv::imwrite("Grad.jpg", gradiented);

    cv::Mat morphed = objdtr.morphological(src);
    cv::imshow("Mor", morphed);
    cv::imwrite("Mor.jpg", morphed);
    cv::waitKey(0);

    return 0;
}
