#include <opencv2/opencv.hpp>
#include <string>
#include <utility> // For finding circles
#include "triton_vision_utils/vision_utils.hpp"
using namespace std;
using namespace cv;
using namespace vision_utils;

class SphereDetector : public ObjectDetector
{
private:
	float focal;
	vector<vector<float>> circle_pos;
	pair<int, int> acc_thres_hist = pair<int, int>(999, 10);

public:
	/**
	 *  Initializes an sphere detector
	 * @param input_im_resize: The scale to resize the image to, default: 1.0
	 * @param input_debug: If True, adds debug information to output, default: false
	 * @param input_focol: default: 400.0
	 */
	SphereDetector(float _im_resize = 1, bool _debug = false, float _focal = 400.0);

	/**
	 * Finds circles in an image using Hough Circle Transform.
	 * @param src: 		 A grayscale image
	 * @param method: 	 Detection method. Currently the only implemented method is HOUGH_GRADIENT
	 * @param dp: 		 Inverse ratio of the accumulator resolution to the image resolution. Default is 1,
	 * 			    	 the accumulator has the same resolution as the input image.
	 * @param minDist: 	 Minimum distance between centers of the detected circles. If = 0, minDist = src.rows/10
	 * @param cannyThreshold: 	 First method-spicific parameter. In case of HOUGH_GRADIENT, it is the first
	 * 				  	 		 threshold for the hysteresis procedure used in the Canny algorithm. Default is 100
	 * @param accumulatorThreshold: 	 Second method-spicific parameter. In case of HOUGH_GRADIENT, it is the second
	 * 				  	 				 threshold for the hysteresis procedure used in the Canny algorithm. Default is 33
	 * @param minRadius: Minimum circle radius
	 * @param maxRadius: Maximum circle radius. If <= 0, uses the maximum image dimension. If <0,
	 * 					 returns center without finding radius.
	 * @return: 	 	 A vector of circles in the form of (x, y, radius), it would be modified
	 */
	vector<Vec3f> find_circles(Mat src, double minDist=0, int method = HOUGH_GRADIENT, double dp = 1, double cannyThreshold = 100, double accumulatorThreshold = 33, int minRadius = 0, int maxRadius = 0);
	/**
	 * Finds circles but with default paramenters and an option to specify how many circles are expected to be found.
	 * Also performs preprocesses and filtering to a copy of the source image before finding circles.
	 * @param src: source image
	 * @param expected: The number of circles expected to be found
	 * @param hue: The hue of circles in interest, default is 10
	 */
	vector<Vec3f> auto_find_circles(Mat &src, int expected, int hue = 10);

	/**
	 * Caculate the position and distance of a circle given its hue and diameter.
	 * They are given in terms of spherical coordinates (r, theta, phi), with the camera as origin.
	 * Positive x is the forwards direction, positive z is the upwards direction.
	 * It also gives the position and radius of circle on the image, for better visualization and testing.
	 * Uses auto_find_circles
	 * @param src: The source image
	 * @param hue: Hue of circles
	 * @param radius: Radius of circles
	 * @param number: Number of circles
	 * @return: A vector of vectors with six elements representing (r, theta, phi, x, y, r_pixel) in this order.
	 * 			Returns an empty vector if no circles are found in the image.
	 */
	vector<vector<float>> circles_positions(Mat &src, float radius, int number, int hue=10);
};

