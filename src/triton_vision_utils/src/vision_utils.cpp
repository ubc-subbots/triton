#include "triton_vision_utils/vision_utils.hpp"
//#include <opencv2/core/mat.hpp>

#define HULL_LIST_SIZE 50

#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
using namespace std;
using namespace cv;

namespace vision_utils
{

	class ObjectDetector
	{
		float im_resize=1.0;
		bool debug=false;
		float focal=400.0;
		Size im_dims;
		Mat curr_image;
        //static const string default_clahe_clr_spaces[3] = {"bgr","hsv","lab"};

		public:
		/**
		 *  Initializes an object detector
		 * @param input_im_resize: The scale to resize the image to, default: 1.0
		 * @param input_debug: If True, adds debug information to output, default: false
		 * @param input_focol: default: 400.0
		 */
		ObjectDetector(float _im_resize, bool _debug, float _focal)
		{
			im_resize = _im_resize;
			debug = _debug;
			focal = _focal;
		}
		/**
		 * Initializes an object detector using default values for 
		 * im_resize, debug and focal
		 */
		ObjectDetector();

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
		Mat preprocess(Mat src)
		{
			im_dims = Size(src.size[0]*im_resize, src.size[1]*im_resize);
			if (im_resize != 1.0)
			{
				Mat srcSplit[3];
				split(src, srcSplit);
				Size kernel = Size(3,3);
				int sig = 1;
				GaussianBlur(srcSplit[0], srcSplit[0], kernel, sig);
				GaussianBlur(srcSplit[1], srcSplit[1], kernel, sig);
				GaussianBlur(srcSplit[2], srcSplit[2], kernel, sig);
				merge(srcSplit, 3, src);
				resize(src, src, im_dims);
				curr_image = src;
			}
			return src;
		}

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
		Mat enhance(Mat src, int clahe_clr_space_bgr=1, int clahe_clr_space_hsv=1,int clahe_clr_space_lab=1, int clahe_clip_limit=1)
		{
            /*
			for (int i = 0; i < sizeof(clahe_clr_spaces); i++)
			{
                String s = clahe_clr_spaces[i];
				if (s.compare("bgr") != 0 && s.compare("bgr") != 0 && s.compare("bgr") != 0)
				{
					cout << "Please only use any of [\"bgr\", \"hsv\", \"lab\"] as CLAHE color spaces.\n";
					return src;
				}
			}
            */

			Ptr<CLAHE> clahe = createCLAHE();
			clahe->setTilesGridSize(Size(11,11));
			clahe->setClipLimit(clahe_clip_limit);
			Mat parts[3];
			int partsLen = 0;

            if (clahe_clr_space_bgr==1)
            {
                Mat bgr[3];
                split(src,bgr);
                clahe->apply(bgr[0], bgr[0]);
                clahe->apply(bgr[1], bgr[1]);
                clahe->apply(bgr[2], bgr[2]);
                Mat bgr_clahe;
                merge(bgr, 3, bgr_clahe);
                parts[partsLen] = bgr_clahe;
                partsLen++;
            }
            if (clahe_clr_space_lab==1)
            {
                Mat lab[3];
                split(src,lab);
                clahe->apply(lab[0], lab[0]);
                clahe->apply(lab[1], lab[1]);
                clahe->apply(lab[2], lab[2]);
                Mat lab_clahe;
                merge(lab, 3, lab_clahe);
                parts[partsLen] = lab_clahe;
                partsLen++;
            }
            if (clahe_clr_space_hsv==1)
            {
                Mat hsv[3];
                split(src,hsv);
                clahe->apply(hsv[0], hsv[0]);
                clahe->apply(hsv[1], hsv[1]);
                clahe->apply(hsv[2], hsv[2]);
                Mat hsv_clahe;
                merge(hsv, 3, hsv_clahe);
                parts[partsLen] = hsv_clahe;
                partsLen++;
            }

			if (partsLen > 0)
			{
				float weight = 1.0/partsLen;
				// Create Mat of unsigned 8-bit int with zeros
                Mat blended = Mat::zeros(im_dims.height, im_dims.width, CV_8U);
				for (int i = 0; i < partsLen; i++)
				{
					blended += weight*parts[i];
				}
				src = blended;
			}
			return src;
		}

        /**
         * Computes the sobel gradient fo a source image
         * @param src: A grayscale image
         * @return The sobel gradient response of the image
         */
        Mat gradient(Mat src)
        {
            int scale = 1;
            int delta = 0;
            int ddepth = CV_16S;
            Mat grad_x, abs_grad_x, grad_y, abs_grad_y;
            Sobel(src, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT);
            Sobel(src, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT);
            convertScaleAbs(grad_x, abs_grad_x, scale, 0);
            convertScaleAbs(grad_y, abs_grad_y, scale, 0);
            Mat grad;
            addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);
            return grad;
            // Note: original python version uses np expand_dims(..., axis=2)

        }

		/**
		 * Smooths a binary image with morphological operations
		 * @param src: A binary image
		 * @param open_kernel: Opening kernel dimensions
		 * @param close_kernel: Closing kernel dimensions
		 */
		Mat morphological(Mat src, Size open_kernel = Size(1,1), Size close_kernel = Size(1,1))
		{
			Mat open_k = getStructuringElement(MORPH_RECT, open_kernel);
			Mat close_k = getStructuringElement(MORPH_RECT, close_kernel);
			Mat opening, closing;
			morphologyEx(src, opening, MORPH_OPEN, open_k);
			morphologyEx(opening, closing, MORPH_CLOSE, close_k);
			return closing;
			
		}

		/**
		 * Creates a set of convex hulls from the binary segmented image and which are of 
		 * an appropriate size based on upper and lower area thresholds
		 * @param src: A binary segmented grayscale image
		 * @param upper_area: Upper threshold of area filter
		 * @param lower_area: Lower threshold of area filter
		 */
		vector<int>* convex_hulls(Mat src, float upper_area=1.0/2, float lower_area=1.0/1000)
		{
			vector<int> hulls[HULL_LIST_SIZE];
            int hullIndex = 0;
			vector<int> right_size_hulls[HULL_LIST_SIZE];
            int right_s_h_index = 0;

			// Find contours in the image
			vector<vector<Point>> contours;
			findContours(src, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

			// Create a convex hull around each connected contour
			for (vector<Point> j : contours)
			{
				convexHull(j, hulls[hullIndex++],false);
			}

			// Get the hulls whose area is within some threshold range
			for (vector<int> hull : hulls)
			{
				int hull_area = contourArea(hull);
				auto im_size = im_dims.height*im_dims.width;
				if (hull_area > im_size*lower_area && hull_area < im_size*upper_area)
				{
                    right_size_hulls[right_s_h_index++] = hull;
				}
			}
            vector<int> right_size_hulls_[right_s_h_index];
            for (int i = 0; i < right_s_h_index; i++)
            {
                 right_size_hulls_[i] = right_size_hulls[i];
            }
			return right_size_hulls_;
		}




	};
    
} // namespace vision_utils
