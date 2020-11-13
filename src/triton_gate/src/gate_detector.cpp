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

/**
 * Gate detector using iamge segmentation, shape classification, and robust pose estimation.
 */

class GateDetector : public ObjectDetector
{
private:
    vector<Point> gate_cntr;
    Size gate_dims; // in m
    vector<tuple<float, float, float, float, float, float>> estimated_poses;
    int frame_count;
    tuple<float, float, float, float, float, float> gate_pose; // x, y, z, phi, theta, psi
    //featurizer 
    char directorybuf[64];
    Mat pre;
    Mat enh;
    Mat seg;
    //vector<Point> *hulls;
    vector<vector<Point>> hulls;
    Mat bound;
    Mat bound_and_pose; 
    bool debug;


public:
    GateDetector(float im_resize=1.0, bool _debug=false, float focal=400)
    {
        ObjectDetector(im_resize, _debug, focal);
        debug = _debug;
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
     * Segment the image using thresholded saturation gradient and orange/red color mask
     * @param src: A preprocessed image
     * @returns: A segmented grayscale image
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
    Mat bound_gate_using_poles(vector<vector<Point>> hulls, Mat src)
    {
        //ignore featurize first, let all hulls be pole hulls
        // Resize src to match the image the hulls were found on
        //resize(src, src, seg.size(), 0,0,INTER_CUBIC);

        // We can't do anything if we aren't given any hulls
        if (hulls.size() == 0)
        {
            return src;
        }

        // Featurize hulls, predict using model and get classified pole hulls
        vector<vector<Point>> pole_hulls;
        Ptr<SVM> svm = SVM::create();
        svm->setType(SVM::Types::C_SVC);
        svm->setKernel(SVM::KernelTypes::LINEAR);
        svm->load("/home/jared/subbot/triton/accuracy_opencv_model.xml");
        vector<float> y_hat;
        cout << "helo\n";
        /*
        sort(hulls.begin(), hulls.end(), 
            [] (vector<Point> a, vector<Point>b)
            {return a.size()>b.size();});
            */
        svm->predict(hulls, y_hat);
        cout << "hi\n";
        for (int i = 0; i < hulls.size(); i++)
        {
            if (y_hat.at(i) == 1)
            {
                pole_hulls.push_back(hulls.at(i));
            }
            else 
            {
                cout << "Not a hull!\n";
            }
          
      }

      // Get 2D array of all the points of the pole hulls (to determine extrema)
      vector<Point> hull_points;
      for (vector<Point> h : pole_hulls)
      {
          for (Point p : h)
          {
          hull_points.push_back(p);
          }
      }

      // If we have detected a hull associated to a pole
      if (hull_points.size() > 0)
      {
          vector<Point>_gate_cntr = create_gate_contour(hull_points, src);

          // Get bounding box of contour to get it's approximate width/height
          Rect box = boundingRect(_gate_cntr);
          //Rect box = boundingRect(hull_points);
          int w = box.width;
          int h = box.height;

          // If the bounding rectangle is more wide than high, most likely we have detected both poles
          if ((float)w/h >= 1)
          {
              if (gate_cntr.size() == 0)
              {
                  // We make the strong assumption that the first time we detect the gate, it is accurate
                  gate_cntr = _gate_cntr;
              }
              else
              {
                  // we make sure the area hasn't changed too much (50%) between frames to account for outliers
                  vector<Point> prev_cntr = gate_cntr;
                  double prev_area = contourArea(prev_cntr);
                  double curr_area = contourArea(_gate_cntr);
                  if (curr_area <= 1.5*prev_area && curr_area >= 0.5*prev_area)
                  {
                      gate_cntr = _gate_cntr;
                  }
              }
              
          }
      }
      // Draw the gate if we have detected it
      if (gate_cntr.size() != 0)
      {
          polylines(src, gate_cntr, true, Scalar(0,0,255),2);
      }

      // Draw all non pole hulls and pole hulls on src for debug purposes
      if (debug)
      {
          polylines(src, hulls, true, Scalar(0,0,255), 2);
          polylines(src, pole_hulls, true, Scalar(255,255,255), 2);
      }
      return src;
    }

    /**
     * 
     */
    vector<Point> create_gate_contour(vector<Point> hull_points, Mat src)
    {
        //int width = src.cols;

        // Get extrema points of hulls (i.e the points closest/furthest from the top left (0,0) and top right (width, 0) of the image)
        Point top_left = *min_element(hull_points.begin(), hull_points.end(),
            [] (Point a, Point b)
            {
                return (a.x < b.x || a.y < b.y);
            });
        Point top_right = *min_element(hull_points.begin(), hull_points.end(),
            [] (Point a, Point b)
            {
                return (a.x > b.x || a.y < b.y);
            });
        Point bot_left = *min_element(hull_points.begin(), hull_points.end(),
            [] (Point a, Point b)
            {
                return (a.x < b.x || a.y > b.y);
            });
        Point bot_right = *min_element(hull_points.begin(), hull_points.end(),
            [] (Point a, Point b)
            {
                return (a.x > b.x || a.y > b.y);
            });

        if (debug)
        {
            circle(src, top_left, 8, Scalar(0,128,0), 4);
            circle(src, top_right, 8, Scalar(0,128,0), 4);
            circle(src, bot_left, 8, Scalar(0,128,0), 4);
            circle(src, bot_right, 8, Scalar(0,128,0), 4);
        }
        gate_cntr.push_back(top_left);
        gate_cntr.push_back(top_right);
        gate_cntr.push_back(bot_left);
        gate_cntr.push_back(bot_right);
        return gate_cntr;
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
    ObjectDetector objdtr = ObjectDetector(1, true, 400);
    GateDetector gatedtr = GateDetector(1, true, 400);

    src = cv::imread("/home/jared/subbot/triton/19.jpg");
    /*
    cv::imwrite("Src.jpg", src);
    //cv::waitKey(0);

    cv::Mat enh = objdtr.enhance(src, 1,1,1,1);
    cv::imwrite("Enh.jpg", enh);
    //cv::waitKey(0);

    cv::Mat gradiented = objdtr.gradient(src);
    cv::imwrite("Grad.jpg", gradiented);
    //cv::waitKey(0);

    cv::Mat morphed = objdtr.morphological(src, Size(50, 50));
    cv::imwrite("Mor.jpg", morphed);

    */
    cv::Mat segmented = gatedtr.segment(src);
    cv::imwrite("Seg.jpg", segmented);

/*
    cv::imshow("Src", src);
    cv::imshow("Enh", enh);
    cv::imshow("Grad", gradiented);
    cv::imshow("Mor", morphed);
    cv::imshow("Seg", segmented);
    cv::waitKey(0);
    */
    vector<vector<Point>> hulls;
    hulls = objdtr.convex_hulls(segmented);
    /*
    Mat drawing = Mat::zeros( segmented.size(), CV_8UC3 );
    for( size_t i = 0; i<hulls.size(); i++ )
    {
        Scalar color = Scalar(255,255,255);
        drawContours( drawing, hulls, (int)i, color );
    }
        imshow("drawing", drawing);
        imwrite("draw.jpg", drawing);
        imshow("seg", segmented);
        cv::waitKey(0);
        */
    gatedtr.bound_gate_using_poles(hulls,src);
    imshow("bounded using poles", src);
    imwrite("bounded_using_poles.jpg", src);
    waitKey(0);


    return 0;
}
