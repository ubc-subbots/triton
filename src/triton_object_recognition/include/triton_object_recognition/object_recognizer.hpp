#ifndef TRITON_OBJECT_RECOGNITION__OBJECT_RECOGNIZER
#define TRITON_OBJECT_RECOGNITION__OBJECT_RECOGNIZER

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.hpp"
#include "triton_interfaces/msg/detection_box_array.hpp"
#include <opencv2/opencv.hpp>

namespace object_recognition
{      

    class ObjectRecognizer : public rclcpp::Node
    {

    public:

        /** Brief description of function.
         * 
         * Longer description in which you describe more in depth about
         * the way the function performs the task mentioned in the brief
         * description above.
         * 
         * @pre precondition of this method, if any
         * 
         * @param options ros2 node options.
         * @param other_param another param
         * 
         * @returns what this function returns, if anything
         * 
         * @post postcondition of this method, if any
         * 
         * @note something of particular note about this function
         * 
         */
        explicit ObjectRecognizer(const rclcpp::NodeOptions & options);

    private:

        /** Brief description of function.
         * 
         * Longer description in which you describe more in depth about
         * the way the function performs the task mentioned in the brief
         * description above.
         * 
         * @pre precondition of this method, if any
         * 
         * @param options ros2 node options.
         * @param other_param another param
         * 
         * @returns what this function returns, if anything
         * 
         * @post postcondition of this method, if any
         * 
         * @note something of particular note about this function
         * 
         */
        void callback(const sensor_msgs::msg::Image::SharedPtr image) const;

        rclcpp::Publisher<triton_interfaces::msg::DetectionBoxArray>::SharedPtr publisher_;  
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_; 

        //Neural Net settings
        std::shared_ptr<cv::dnn::Net> net_;
        cv::String model_ = "src/triton_object_recognition/models/yolov3.weights";
        cv::String config_ = "src/triton_object_recognition/models/yolov3.cfg";
        cv::dnn::Backend backend_ = cv::dnn::DNN_BACKEND_OPENCV;
        cv::dnn::Target target_ = cv::dnn::DNN_TARGET_CPU;
        std::vector<std::string> classes_;
        
        //Parameters
        float conf_threshold_ = 0.2;
        float nms_threshold_ = 0.2;
        float scale_ = 0.00392;
        int inp_width_ = 416;
        int inp_height_ = 416;
        bool swap_rb_ = true;
        float mean_ = 0.5;

        //Processing code adapted from https://github.com/opencv/opencv/blob/master/samples/dnn/object_detection.cpp
        void preprocess(const cv::Mat& frame) const;
        void postprocess(std::vector<int> & classIds, std::vector<float> & confidences, std::vector<cv::Rect> & boxes, 
                cv::Mat & frame, const std::vector<cv::Mat>& outs) const;
    };
    
} // namespace example

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(object_recognition::ObjectRecognizer)

#endif  //TRITON_OBJECT_RECOGNITION__OBJECT_RECOGNIZER
