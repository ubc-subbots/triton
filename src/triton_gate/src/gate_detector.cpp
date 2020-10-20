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

    cv::Mat morphed = objdtr.morphological(gradiented);
    cv::imshow("Mor", morphed);
    //cv::imwrite("Mor.jpg", morphed);
    cv::waitKey(0);

     if (__cplusplus == 201703L) std::cout << "C++17\n";
        else if (__cplusplus == 201402L) std::cout << "C++14\n";
        else if (__cplusplus == 201103L) std::cout << "C++11\n";
        else if (__cplusplus == 199711L) std::cout << "C++98\n";
        else std::cout << "pre-standard C++\n";

    return 0;
}
