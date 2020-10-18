#ifndef TRITON_OBJECT_RECOGNITION__OBJECT_RECOGNIZER
#define TRITON_OBJECT_RECOGNITION__OBJECT_RECOGNIZER

#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "std_msgs/msg/string.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.hpp"
#include "triton_interfaces/msg/detection_box_array.hpp"
#include "triton_interfaces/srv/object_detection.hpp"
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

        /** Reads image and performs object recognition.
         * 
         * Uses the loaded network to perform object recognition on the image.
         * 
         * @param msg message containing image data
         * 
         * @returns DetectionBoxArray message containing the bounding boxes of detected objects
         * 
         * @note something of particular note about this function
         * 
         */
        triton_interfaces::msg::DetectionBoxArray process(const sensor_msgs::msg::Image & msg) const;

        /** Callback used by subscriber to process and publish results
         */
        void subscriberCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg) const;
        /** Callback used by service to process and respond to client with results
         */
        void serviceCallback(const triton_interfaces::srv::ObjectDetection::Request::SharedPtr request, 
                const triton_interfaces::srv::ObjectDetection::Response::SharedPtr response) const;

        rclcpp::Publisher<triton_interfaces::msg::DetectionBoxArray>::SharedPtr publisher_;  
        image_transport::Subscriber subscription_; 
        rclcpp::Service<triton_interfaces::srv::ObjectDetection>::SharedPtr service_;

        std::shared_ptr<cv::dnn::Net> net_;

        //Default Neural Net Parameters (overriden by parameters)
        std::string model_folder_ = "src/triton_object_recognition/models/";
        std::string weights_url_ = "https://pjreddie.com/media/files/yolov3.weights";
        std::string weights_filename_ = "yolov3.weights";
        std::string cfg_url_ = "https://raw.githubusercontent.com/pjreddie/darknet/master/cfg/yolov3.cfg";
        std::string cfg_filename_ = "yolov3.cfg";
        cv::dnn::Backend backend_ = cv::dnn::DNN_BACKEND_OPENCV;
        cv::dnn::Target target_ = cv::dnn::DNN_TARGET_CPU;
        std::vector<std::string> classes_;
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
