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

    };
    
} // namespace example

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(object_recognition::ObjectRecognizer)

#endif  //TRITON_OBJECT_RECOGNITION__OBJECT_RECOGNIZER
