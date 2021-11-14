#include "triton_object_recognition/bounding_box_pose_estimation.hpp"
#include <rcl_yaml_param_parser/parser.h>
#include "ament_index_cpp/get_package_share_directory.hpp"
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using namespace std;
using namespace cv;

namespace triton_object_recognition
{
    BoundingBoxPoseEstimation::BoundingBoxPoseEstimation(const rclcpp::NodeOptions &options)
        : Node("bbox_pose", options)
    {
        //Create publisher, subscriber, and service
        publisher_ = this->create_publisher<triton_interfaces::msg::PointArray>(
            "bbox_pose/out",
            10);

        subscription_ = this->create_subscription<triton_interfaces::msg::DetectionBoxArray>("bbox_pose/in", 10,
                                                                                             bind(&BoundingBoxPoseEstimation::subscriberCallback, this, _1));

        //Populate parameters (values are not modified if parameters have not been declared)
        this->declare_parameter("intrinsics", intr_); //fx,fy,cx,cy
        this->declare_parameter("distortion_coefficients", distort_);
        this->declare_parameter("classes", class_sizes_);
        this->declare_parameter("camera_height", camera_height_);
        this->declare_parameter("camera_width", camera_width_);

        this->get_parameter("intrinsics", intr_);
        this->get_parameter("distortion_coefficients", distort_);
        this->get_parameter("classes", class_sizes_);
        this->get_parameter("camera_height", camera_height_);
        this->get_parameter("camera_width", camera_width_);

        RCLCPP_INFO(get_logger(), "Bounding Box Pose Estimation successfully started!");
    }

    void BoundingBoxPoseEstimation::subscriberCallback(const triton_interfaces::msg::DetectionBoxArray::SharedPtr msg) const
    {
        publisher_->publish(process(*msg));
    }

    triton_interfaces::msg::PointArray BoundingBoxPoseEstimation::process(const triton_interfaces::msg::DetectionBoxArray &msg) const
    {
        triton_interfaces::msg::PointArray out;
        out.header = msg.header;

        RCLCPP_INFO(get_logger(), "Finding pose");

#if DEBUG_VISUALIZE
        cv::Mat frame = cv::Mat::zeros(cv::Size2i(camera_width_, camera_height_), CV_8UC1);
#endif

        for (auto &box : msg.boxes)
        {
            triton_interfaces::msg::Point point;
            //Each class of object is associated with a "size" parameter corresponding to the dimensions along two axes (most of our objects are flat so this is not completely unreasonable)
            auto class_size = class_sizes_[box.class_id]; //single dimension for each object for now
            if (class_size == 0)
            {
                continue;
            }

            //Inaccurate pose estimation:
            //We want to use the 2D bounding box to estimate a 3D position for the object

            //Key equation: box_width = x1 - x2 = (i1-cx)/fx*z - (i2-cx)/fx*z =  ~= size_x
            //We can then recover z = size_x*fx/(i1-i2)

            //This equation is only correct if the object is perfectly parallel to the image plane and axis-aligned and if the object lies completely within the image and distortion is ignored
            //TL;DR don't expect this to be very accurate

            double z_x = class_size * intr_[0] / (box.width);
            double z_y = class_size * intr_[1] / (box.height);
            double z_avg = (z_x + z_y) / 2.;

            //x,y are the centre of the bounding box scaled based on our estimated z
            auto centre_x = (box.x + (box.width / 2.));
            auto centre_y = (box.y + (box.height / 2.));
            point.class_id = box.class_id;
            point.x = z_avg * centre_x / intr_[0];
            point.y = z_avg * centre_y / intr_[1];
            point.z = z_avg;
            out.points.push_back(point);

#if DEBUG_VISUALIZE
            //visualize by projecting estimated object square into image
            int w2 = class_sizes_[box.class_id] / 2. * intr_[0] / z_avg;
            int h2 = class_sizes_[box.class_id] / 2. * intr_[1] / z_avg;
            auto c1 = cv::Point2i(centre_x - w2, centre_y - h2);
            auto c2 = cv::Point2i(centre_x - w2, centre_y + h2);
            auto c3 = cv::Point2i(centre_x + w2, centre_y + h2);
            auto c4 = cv::Point2i(centre_x + w2, centre_y - h2);

            cv::line(frame, c1, c2, cv::Scalar(0xff, 0, 0), 2);
            cv::line(frame, c2, c3, cv::Scalar(0xff, 0, 0), 2);
            cv::line(frame, c3, c4, cv::Scalar(0xff, 0, 0), 2);
            cv::line(frame, c4, c1, cv::Scalar(0xff, 0, 0), 2);
#endif
        }

#if DEBUG_VISUALIZE
        cv::imshow("pose", frame);
        cv::waitKey(1);
#endif

        return out;
    }
} // namespace triton_object_recognition