#include <cstdio>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/ml.hpp>
#include <vector>

#include <iostream>
using namespace std;

int main(int argc, char ** argv)
{
  cv::Ptr<cv::ml::SVM> svm = cv::ml::SVM::load("/home/logan/Projects/underwater-object-detection/src/accuracyopencv_model.xml");

  float X_hat_vec[7][10] = {
    {  0.09385864 ,  0.11278195,   1.06720508  , 0.08767594  , 0.19329583,
      3.49600725,   3.54310926,   7.06266775,   3.63978549, -10.04392885},
    { 0.06184626,   0.07086614,   1.08295195,  -0.15385408,  -0.30197186,
      2.27922654,   2.29384885,   4.58038654,   2.14286352,  -8.671244},
    {  0.12124721,   0.17322835,   1.06714845,   0.11280684,   0.2457326,
      3.02761138,   3.07569424,   6.12734981,   3.19918599,  -8.575469},
    {  0.06135734,   0.1322314,    1.09632751,  -0.12821825,  -0.25001849,
      4.47631857,   4.48453041,   8.96495493,   4.35959737, -12.46409888},
    {  0.63128464,   0.68421053,   1.32153392,   0.74862317,   2.22327781,
      4.18081392,   5.71194012, -11.73740486,  -7.3368992,  -10.65983102},
    {  0.82512111,   0.68,         1.13330963,   0.74374756,   2.22661604,
      4.15227431,   5.65816302,  10.66307356,   6.8151742,  -10.78037121},
    {  0.3964516,   1.84,         1.26688742,   0.67980776,   1.77769975,
      3.19467863,   4.00361988,   7.61504598,   4.91673724,   8.23271258}};
  cv::Mat X_hat= cv::Mat(7,10, CV_32F, &X_hat_vec);
  int y_hat[7] = {1, 1, 1, 1, 0, 0, 0};
  cv::Mat y_pred;
  svm->predict(X_hat, y_pred);

  cout << "Weights " << endl << " "  << svm->getClassWeights() << endl << endl;
  cout << "Predictions " << endl << " "  << y_pred << endl << endl;
  for (int i=0; i < 7; i++)
  {
    cout << "Is correct prediction for y_hat[" << i << "]: " << endl;
    printf(y_pred.at<cv::Scalar>(i) == cv::Scalar(y_hat[i]) ? "true" : "false");
  }
  return 0;
}
