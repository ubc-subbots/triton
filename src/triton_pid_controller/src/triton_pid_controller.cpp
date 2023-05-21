#include <chrono>
#include <cmath>
#include <memory>
#include <triton_pid_controller/triton_pid_controller.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace triton_pid_controller
{

  PidController::PidController(const rclcpp::NodeOptions & options) 
  : Node("pid_controller", options)
  {
  //  pid_pitch_.load("pid_controller/pid_pitch.yaml");
  //  pid_pos_.load("pid_controller/pid_pos.yaml");

    last_time_ = std::chrono::high_resolution_clock::now();
    auto control_loop_time = 5ms;
    control_loop_timer_ = create_wall_timer(control_loop_time, 
      std::bind(&PidController::control_loop, this));

    // ROS2 setup
    sub_ = create_subscription<geometry_msgs::msg::Pose>(
        "/triton/controls/input_pose",
        10,
        std::bind(&PidController::pose_update, this, _1));

    pub_ = create_publisher<geometry_msgs::msg::Wrench>(
        "/triton/controls/input_forces",
        10);

    RCLCPP_INFO(this->get_logger(), "PID Controller starting!");

    float x_p, x_i, x_d;
    float y_p, y_i, y_d;
    float z_p, z_i, z_d;
    float yaw_p, yaw_i, yaw_d;
    this->declare_parameter("force_x_p", x_p);
    this->declare_parameter("force_x_i", x_i);
    this->declare_parameter("force_x_d", x_d);
    this->declare_parameter("force_y_p", y_p);
    this->declare_parameter("force_y_i", y_i);
    this->declare_parameter("force_y_d", y_d);
    this->declare_parameter("force_z_p", z_p);
    this->declare_parameter("force_z_i", z_i);
    this->declare_parameter("force_z_d", z_d);
    this->declare_parameter("force_yaw_p", yaw_p);
    this->declare_parameter("force_yaw_i", yaw_i);
    this->declare_parameter("force_yaw_d", yaw_d);

    this->get_parameter("force_x_p", x_p);
    this->get_parameter("force_x_i", x_i);
    this->get_parameter("force_x_d", x_d);
    this->get_parameter("force_y_p", y_p);
    this->get_parameter("force_y_i", y_i);
    this->get_parameter("force_y_d", y_d);
    this->get_parameter("force_z_p", z_p);
    this->get_parameter("force_z_i", z_i);
    this->get_parameter("force_z_d", z_d);
    this->get_parameter("force_yaw_p", yaw_p);
    this->get_parameter("force_yaw_i", yaw_i);
    this->get_parameter("force_yaw_d", yaw_d);

    pid_force_x.load(x_p, x_i, x_d);
    pid_force_y.load(y_p, y_i, y_d);
    pid_force_z.load(z_p, z_i, z_d);
    pid_yaw.load(yaw_p, yaw_i, yaw_d);

    RCLCPP_INFO(this->get_logger(), "PID Controller successfully started!");
  }

  PidController::~PidController()
  {
  }

  void PidController::pose_update(const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    cur_pose = msg;
  }

  void PidController::control_loop()
  {
    if (!cur_pose)
    {
      return;
    }

    auto now = std::chrono::high_resolution_clock::now();
    float dt = 
      std::chrono::duration_cast<std::chrono::microseconds>(now - last_time_).count() / 1e6;

    auto & x = cur_pose->orientation.x;
    auto & y = cur_pose->orientation.y;
    auto & z = cur_pose->orientation.z;
    auto & w = cur_pose->orientation.w;

    float cur_yaw = -std::atan2(2*y*w + 2*x*z, 1 - 2*y*y - 2*z*z);

    float yaw_error = cur_yaw;
    float pos_x_error = cur_pose->position.x;
    float pos_y_error = cur_pose->position.y;
    float pos_z_error = cur_pose->position.z;

    float forceX = pid_force_x.update(pos_x_error, dt);
    float forceY = pid_force_y.update(pos_y_error, dt);
    float forceZ = pid_force_z.update(pos_z_error, dt);
    float torqueZ = pid_yaw.update(yaw_error, dt);

    last_time_ = std::chrono::high_resolution_clock::now();

    geometry_msgs::msg::Wrench wrenchOut;
    wrenchOut.force.x = forceX;
    wrenchOut.force.y = forceY;
    wrenchOut.force.z = forceZ;
    wrenchOut.torque.x = 0;
    wrenchOut.torque.y = 0;
    wrenchOut.torque.z = torqueZ;
    pub_->publish(wrenchOut);
  }
}  // namespace triton_pid_controller

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions();
  rclcpp::spin(std::make_shared<triton_pid_controller::PidController>(options));
  rclcpp::shutdown();
  return 0;
}
