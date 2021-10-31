#ifndef TRITON_OBJECT_RECOGNITION__BOUNDING_BOX_POSE_ESTIMATION
#define TRITON_OBJECT_RECOGNITION__BOUNDING_BOX_POSE_ESTIMATION

#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "std_msgs/msg/string.hpp"
#include "cv_bridge/cv_bridge.h"
#include "triton_interfaces/msg/detection_box_array.hpp"
#include "triton_interfaces/msg/point_array.hpp"
#include <opencv2/opencv.hpp>
#include <map>

#ifndef DEBUG_VISUALIZE
    #define DEBUG_VISUALIZE 1
#endif

namespace triton_object_recognition
{

    class BoundingBoxPoseEstimation : public rclcpp::Node
    {

    public:
        /** ROS2 node to estimate 3D position from bounding boxes
         * 
         */
        explicit BoundingBoxPoseEstimation(const rclcpp::NodeOptions &options);

    private:
        /** Reads image and performs object recognition.
         * 
         * Uses the loaded network to perform object recognition on the image.
         * 
         * @param msg message containing image data
         * 
         * @returns DetectionBoxArray message containing the bounding boxes of detected objects
         * 
         */
        triton_interfaces::msg::PointArray process(const triton_interfaces::msg::DetectionBoxArray &msg) const;

        /** Callback used by subscriber to process and publish results
         */
        void subscriberCallback(const triton_interfaces::msg::DetectionBoxArray::SharedPtr msg) const;

        rclcpp::Publisher<triton_interfaces::msg::PointArray>::SharedPtr publisher_;
        rclcpp::Subscription<triton_interfaces::msg::DetectionBoxArray>::SharedPtr subscription_;

        std::vector<double> intr_;
        std::vector<double> distort_;
        std::vector<double> class_sizes_;
        //std::map<int, double> class_sizes_;
        int camera_height_;
        int camera_width_;
    };

} // namespace triton_object_recognition

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(triton_object_recognition::BoundingBoxPoseEstimation)

#endif //TRITON_OBJECT_RECOGNITION__BOUNDING_BOX_POSE_ESTIMATION
