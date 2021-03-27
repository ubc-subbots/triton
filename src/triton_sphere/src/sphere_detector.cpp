#include "triton_sphere/sphere_detector.hpp"
//#include <opencv2/core/mat.hpp>

#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <cmath> // for circle_position
using namespace std;
using namespace cv;
using namespace vision_utils;

SphereDetector::SphereDetector(float _im_resize, bool _debug, float _focal)
{
    ObjectDetector(_im_resize, _debug, _focal);
    focal = _focal;
}

vector<Vec3f> SphereDetector::find_circles(Mat src, double minDist, int method, double dp, double cannyThreshold, double accumulatorThreshold, int minRadius, int maxRadius)
{
    vector<Vec3f> circles;

    GaussianBlur(src, src, Size(9,9), 2, 2);

    double mD;
    if (minDist == 0)
    {
        mD = src.rows/10;
    }
    else {
        mD = minDist;
    }
    HoughCircles(src, circles, method, dp, mD, cannyThreshold, accumulatorThreshold, minRadius, maxRadius);
    return circles;
}

vector<Vec3f> SphereDetector::auto_find_circles(Mat &src, int expected, int hue)
{
    double minDist = (double)src.rows/10;
    double minCircleArea = minDist * minDist / 4 * 3.14159 * 0.9;
    // ===== Some preprocessing should be done outside of this function ====
    src = this->util_segment(src, hue);
    src = this->filter_small_contours(src, minCircleArea);
    src = this->morphological(src, Size(3,3), Size(25,25));

    vector<Vec3f> circles;
    // Dynamically adjust accum_thres only every certain number of frames where
    // not enough circles are found, to improve performance
    if (acc_thres_hist.second > 9) {
      int accum_thres = 30;
      circles = this->find_circles(src, (int)src.rows/10, 3, 1, 100, accum_thres, 0, 0);
      while ((int)circles.size() < expected && accum_thres > 20) { // if there are circles, ~20 should be the lowest we need
          circles = this->find_circles(src, (int)src.rows/10, 3, 1, 100, accum_thres, 0, 0);
          accum_thres--;
      }
      // Only updates acc_thres_hist when all circles are found, to avoid situations
      // where some circles are out of frame and accum_thres is really low
      if ((int)circles.size() == expected) {
        //if (accum_thres < acc_thres_hist.first)
          acc_thres_hist.first = accum_thres;
      }
      acc_thres_hist.second = 0;
    }
    else {
      int accum_thres = acc_thres_hist.first;
      circles = this->find_circles(src, (int)src.rows/10, 3, 1, 100, accum_thres, 0, 0);
      // acc_thres_hist.second counts the number of frames where we have not found
      // enough circles
      if ((int)circles.size() != expected) {
        acc_thres_hist.second++;
      }
    }
    return circles;
}

vector<vector<float>> SphereDetector::circles_positions(Mat &src, float radius, int number, int hue) {

    vector<Vec3f> circles = this->auto_find_circles(src, number, hue);
    vector<vector<float>> positions;

    // If no circles are found
    if (circles.size() < 1)
        return positions;

    // Calculate the distane based on perceived size, real size, focal length, (and tilted angles?)
    for (size_t i = 0; i < circles.size(); i++) {
      vector<float> position;
      int circle_x = circles[i][0]; // Coordinates of the circle
      int circle_y = circles[i][1];
      int radius_pixel = circles[i][2]; // Radius of the circle in pixels

      // Distance = Real Radius * Focal length in pixels / Radius on image in pixels
      float distance = radius * this->focal / radius_pixel;

      // Calculate the angles based on given x,y coordinates from auto_find_circles

      // Obtain the coordinates of the circle in the image if the origin is at the center of the image.
      int center_x = src.cols / 2;
      int center_y = src.rows / 2;
      int circle_x_from_center = circle_x - center_x;
      int circle_y_from_center = circle_y - center_y;
      // Calculate the horizontal and vertical distances from the x-axis (of the 3D spherical coordiante system)
      // in the same unit as circle radius (assumed meters)
      float mp_ratio = radius / radius_pixel;
      float circle_x_distance = circle_x_from_center * mp_ratio;
      float circle_y_distance = circle_y_from_center * mp_ratio;
      // Calcaulate phi
      float phi = atan(circle_x_distance/circle_y_distance);
      // Calculate theta
      float opp_side = sqrt(circle_y_distance*circle_y_distance + circle_x_distance*circle_x_distance);
      float theta = atan(opp_side / circle_y_distance);


      position.push_back(distance);
      position.push_back(theta);
      position.push_back(phi);
      position.push_back(circle_x);
      position.push_back(circle_y);
      position.push_back(radius_pixel);

      if (circle_pos.empty()) {
        // We make the strong assumption that the first circles we find are accurate.
        positions.push_back(position);
      }
      else {
        // We make sure the circles have not changed too much in position or distance to account for false circles
        int ng = 0;
        for (vector<float> v : circle_pos) {
          if (abs(distance - v[0]) > 0.25 // assuming 20 fps, sphere speed should be < 5 m/s (20?)
            || abs((center_x - v[3]) * mp_ratio) > 0.25
            || abs((center_y - v[4]) * mp_ratio) > 0.25 ) {
            //|| radius_pixel < v[5] * 0.5 || radius_pixel > v[5] * 1.5 ) {
              ng++;
            }
          else {
            break;
          }
        }
        if (ng < number) {
          positions.push_back(position);
        }
      }
    }

    circle_pos = positions;
    return positions;
}

int main(void)
{
    const String window_capture_name = "Video Capture";
    const String window_capture_name_filtered = "Video Capture: filtered";
    const String window_detection_name = "Object Detection";

    //! [cap]
    //VideoCapture cap(argc > 1 ? atoi(argv[1]) : 0);
    //VideoCapture cap(0); // Web cam
    //VideoCapture cap("/home/jared/projects/triton_sphere_tests/assets/test1.MOV");
    VideoCapture cap("/home/jared/projects/triton_sphere_tests/assets/underwater_juggling.mp4");
    //VideoCapture cap("/home/jared/projects/triton_sphere_tests/assets/bounce.mp4");
    //! [cap]

    int hue = 10;
    int number = 1;

    //! [window]
    namedWindow(window_capture_name, WINDOW_KEEPRATIO);
    //namedWindow(window_capture_name_filtered, WINDOW_KEEPRATIO);
    namedWindow(window_detection_name, WINDOW_KEEPRATIO);
    //! [window]

    // laptop
    int focal = 700;
    // iphone 8 video
    //int focal = 2100;
    SphereDetector objdtr = SphereDetector(1, true, focal);

    //int tennis_ball_radius = 35; // mm
    //int larger_ball_radius = 100;
    int iphone_image = 29;
    //int test_mov = 45;
    int ball_radius = iphone_image;

    createTrackbar( "Hue", window_capture_name, &hue, 180, 0 );
    createTrackbar( "Number of spheres", window_capture_name, &number, 10, 0 );
    createTrackbar( "Ball radius (mm)", window_capture_name, &ball_radius, 1000, 0 );
    createTrackbar( "Focal length (pixels)", window_capture_name, &focal, 5000, 0 );

    Mat frame, frame_detection;
    while (true) {
        //! [while]
        cap >> frame;
        if(frame.empty())
        {
            break;
        }
        Mat original = frame;
        vector<vector<float>> circle_pos = objdtr.circles_positions(frame, (float)ball_radius / 1000, number, hue);
        if (!circle_pos.empty()) {
            //cvtColor(frame, frame_detection, COLOR_BGR2GRAY);
            cvtColor(frame, frame_detection, COLOR_GRAY2BGR);
          for (size_t i = 0; i < circle_pos.size(); i++) {
            Point center(cvRound(circle_pos[i][3]), cvRound(circle_pos[i][4]));
            int radius = cvRound(circle_pos[i][5]);

            // draw the circle center
            circle( frame_detection, center, 3, Scalar(0,255,0), -1, 8, 0 );
            circle( original, center, 3, Scalar(0,255,0), -1, 8, 0 );
            // draw the circle outline
            circle( frame_detection, center, radius, Scalar(0,0,255), 3, 8, 0 );
            circle( original, center, radius, Scalar(0,0,255), 3, 8, 0 );
            ostringstream str;
            str << "( " << circle_pos[i][0] << "m, " << circle_pos[i][1] << ", " << circle_pos[i][2] << ")";
            cout << i << ": " << circle_pos[i][0] << ", " << circle_pos[i][1] << ", " << circle_pos[i][2] << "\n";
            string circle_info = str.str();
            ostringstream str2;
            str2 << "Focal: " << focal << " pixels  " << "Ball radius: " << ball_radius / 1000.0 << "m";
            string more_info = str2.str();
            putText(frame_detection, circle_info, center, FONT_HERSHEY_SIMPLEX, 1, Scalar(255,200,200), 2);
            putText(frame_detection, more_info, Point(20,20), FONT_HERSHEY_PLAIN, 1, Scalar(255,200,200), 1);
          }
        }
        else cout << "no circles\n";

          //! [show]
          // Show the frames
          imshow(window_capture_name, original);
          //imshow(window_capture_name_filtered, frame);
          imshow(window_detection_name, frame_detection);
          /*
          int h = original.rows / 2;
          int w = original.cols/2;
          resizeWindow(window_capture_name, h, w);
          resizeWindow(window_capture_name_filtered, h, w);
          resizeWindow(window_detection_name, h, w);
          */
          //! [show]


        char key = (char) waitKey(30);
        if (key == 'q' || key == 27)
        {
            break;
        }
    }
    return 0;
}
