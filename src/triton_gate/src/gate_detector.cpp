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
    array<vector<Point>,4> gate_cntr;
    Size gate_dims; // in m
    vector<tuple<float, float, float, float, float, float>> estimated_poses;
    int frame_count;
    tuple<float, float, float, float, float, float> gate_pose; // x, y, z, phi, theta, psi
    //featurizer 
    char directorybuf[64];
    Mat pre;
    Mat enh;
    Mat seg;
    vector<Point> *hulls;
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
        // Convert to HSV color space
        Mat hsv;
        cvtColor(src, hsv, COLOR_BGR2HSV);

        // Compute gradient threshold on saturation channel of HSV image
        // (seems to have best response)
        ObjectDetector objdtr = ObjectDetector(0.5, true, 400);
        Mat hsvChannels[3];
        split(hsv, hsvChannels);
        Mat grad = objdtr.gradient(hsvChannels[1]);
        Scalar grad_mean, grad_std;
        meanStdDev(grad, grad_mean, grad_std);
        Mat grad_thres;
        // Since grad should be single-channel, grad_mean[0] and grad_std[0] should contain the values we need
        threshold(grad, grad_thres, (double)(grad_mean[0])+4*grad_std[0], 255, THRESH_BINARY);

        // Orange/red hue mask
        Mat upper_hue_mask, lower_hue_mask, color_mask;
        inRange(hsvChannels[0], 175, 180, upper_hue_mask);
        inRange(hsvChannels[0], 0, 30, lower_hue_mask);
        bitwise_or(upper_hue_mask, lower_hue_mask, color_mask);

        // Combine the two binary image, note that the gradient threshold has the best response from farther away
        // and the color mask works best at close distances, so by combining them, we have an image that produces
        // a great response to the poles at all distances to them
        Mat segmented;
        bitwise_or(grad_thres, color_mask, segmented);
        return segmented;
    }

    /**
     * 
     */
    Mat bound_gate_using_poles(vector<Point>* hulls, Mat src)
    {
      return src;
    }

    /**
     * 
     */
    array<vector<Point>,4> create_gate_contour(Point hull_points[], Mat src)
    {
      return array<vector<Point>,4>{};
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
    //cv::imshow("Src", src);
    //cv::imwrite("Src.jpg", src);
    //cv::waitKey(0);

    cv::Mat enh = objdtr.enhance(src, 1,1,1,1);
    //cv::imshow("Enh", enh);
    //cv::waitKey(0);

    cv::Mat gradiented = objdtr.gradient(src);
    //cv::imshow("Grad", gradiented);
    //cv::imwrite("Grad.jpg", gradiented);
    //cv::waitKey(0);

    cv::Mat morphed = objdtr.morphological(src, Size(50, 50));
    //cv::imshow("Mor", morphed);
    //cv::imwrite("Mor.jpg", morphed);
    //cv::waitKey(0);

    Mat gray;
    cvtColor(gradiented, gray,COLOR_BGR2GRAY);
    vector<Point> *hulls;
    hulls = objdtr.convex_hulls(gray);
    for (int i = 0; i < 5; i++)
    {
      cout << hulls[0].at(i) << endl;
    }

    return 0;
}
