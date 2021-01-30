#ifndef TRITON_VISION_UTILS__VISION_UTILS
#define TRITON_VISION_UTILS__VISION_UTILS

#include <opencv2/opencv.hpp>
#include <string>
using namespace std;
using namespace cv;

namespace vision_utils
{
	class ObjectDetector
	{
	private:
		float im_resize = 1.0;
		bool debug = false;
		float focal = 400.0;
		Size im_dims;
		Mat curr_image;

	public:
		/**
		 *  Initializes an object detector
		 * @param input_im_resize: The scale to resize the image to, default: 1.0
		 * @param input_debug: If True, adds debug information to output, default: false
		 * @param input_focol: default: 400.0
		 */
		ObjectDetector(float _im_resize = 1, bool _debug = false, float _focal = 400.0);

		/**
		 * Detects an object from a raw image
		 * @param src: Raw image
		 * @return Three images representing the algorithms at various stages. The last
		 *          image must always be the final output fo the algorithm.
		 */
		//detect(src);

		/**
		 * Preprocess the source image by blurring and resizing
		 * @param src: A raw unscaled image
		 * @return: The preprocessed and scaled image
		 */
		Mat preprocess(Mat src);

		/**
         * Enhances a raw image to account for underwater artifacts affecting contrast, hue and saturation.
         * Performs CLAHE ton the given input color spaces then blends the equally weighted result across
         * all color spaces used.
         * @param src: A preprocessed image
         * @param clahe_clr_space_bgr: 1 means to use this CLAHE color space
         * @param clahe_clr_space_hsv: 1 means to use this CLAHE color space
         * @param clahe_clr_space_lab: 1 means to use this CLAHE color space
         * @param clahe_clip_limt: The limit at which CLAHE clips the contrast to prevent over-contrasting
         * @return: An enhanced image
         */
		Mat enhance(Mat src, int clahe_clr_space_bgr = 1, int clahe_clr_space_hsv = 1, int clahe_clr_space_lab = 1, int clahe_clip_limit = 1);

		/**
         * Computes the sobel gradient fo a source image
         * @param src: A grayscale image
         * @return The sobel gradient response of the image
         */
		Mat gradient(Mat src);

		/**
		 * Smooths a binary image with morphological operations
		 * @param src: A binary image
		 * @param open_kernel: Opening kernel dimensions
		 * @param close_kernel: Closing kernel dimensions
		 */
		Mat morphological(Mat src, Size open_kernel = Size(1, 1), Size close_kernel = Size(1, 1));

		/**
		 * Creates a set of convex hulls from the binary segmented image and which are of 
		 * an appropriate size based on upper and lower area thresholds
		 * @param src: A binary segmented grayscale image
		 * @param upper_area: Upper threshold of area filter
		 * @param lower_area: Lower threshold of area filter
		 */
		vector<vector<Point>> convex_hulls(Mat src, float upper_area = 1.0 / 2, float lower_area = 1.0 / 1000);
		
		/**
		 * Segment image using specified hue.
		 * @param src: An image
		 * @param hue: Desired hue
		 * @return: A segmented grayscale image
		 */
		Mat util_segment(Mat src, int hue);
		/**
		 * Finds circles in an image using Hough Circle Transform.
		 * @param src: 		 A grayscale image
		 * @param method: 	 Detection method. Currently the only implemented method is HOUGH_GRADIENT
		 * @param dp: 		 Inverse ratio of the accumulator resolution to the image resolution. Default is 1, 
		 * 			    	 the accumulator has the same resolution as the input image.
		 * @param minDist: 	 Minimum distance between centers of the detected circles. If = 0, minDist = src.rows/10
		 * @param cannyThreshold: 	 First method-spicific parameter. In case of HOUGH_GRADIENT, it is the first 
		 * 				  	 		 threshold for the hysteresis procedure used in the Canny algorithm
		 * @param accumulatorThreshold: 	 Second method-spicific parameter. In case of HOUGH_GRADIENT, it is the second 
		 * 				  	 				 threshold for the hysteresis procedure used in the Canny algorithm
		 * @param minRadius: Minimum circle radius
		 * @param maxRadius: Maximum circle radius. If <= 0, uses the maximum image dimension. If <0, 
		 * 					 returns center without finding radius.
		 * @return: 	 	 A vector of circles in the form of (x, y, radius), it would be modified
		 */
		vector<Vec3f> find_circles(Mat src, double minDist=0, int method = HOUGH_GRADIENT, double dp = 1, double cannyThreshold = 100, double accumulatorThreshold = 33, int minRadius = 0, int maxRadius = 0);
		vector<Vec3f> auto_find_circles(Mat src);
	};

} // namespace vision_utils

#endif // TRITON_VISION_UTILS__VISION_UTILS