#include "triton_object_recognition/object_recognizer.hpp"

using std::placeholders::_1;
using namespace cv;

namespace object_recognition
{

ObjectRecognizer::ObjectRecognizer(const rclcpp::NodeOptions & options)
: Node("object_recognizer", options) 
{
    publisher_ = this->create_publisher<triton_interfaces::msg::DetectionBoxArray>("object_recognizer/out", 10);

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "object_recognizer/in", 10, std::bind(&ObjectRecognizer::callback, this, _1));
}


void ObjectRecognizer::callback(const sensor_msgs::msg::Image::SharedPtr msg) const
{
    RCLCPP_INFO(this->get_logger(), "In object_recognizer");
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(get_logger(),"cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat frame = cv_ptr->image;
    imshow("test",frame);
    waitKey(0);


    auto message = triton_interfaces::msg::DetectionBoxArray();




    msg->data;
    //message.data = msg->data + " from object_recognizer";
    publisher_->publish(message);
}

} // namespace object_recognition