#include <triton_pid_controller/triton_pid_controller.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <chrono>
#include <cmath>

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace triton_pid_controller
{

  PidController::PidController(const rclcpp::NodeOptions & options) : Node("pid_controller", options)
  {
  //  pid_pitch_.load("pid_controller/pid_pitch.yaml");
  //  pid_pos_.load("pid_controller/pid_pos.yaml");

    last_time_ = std::chrono::high_resolution_clock::now();
    auto control_loop_time = 5ms;
    control_loop_timer_ = create_wall_timer(control_loop_time, std::bind(&PidController::control_loop, this));

    // ROS2 setup
    sub_ = create_subscription<geometry_msgs::msg::Pose>(
        "/triton/controls/input_pose",
        10,
        std::bind(&PidController::pose_update, this, _1));

    pub_ = create_publisher<geometry_msgs::msg::Wrench>(
        "/triton/controls/input_forces",
        10);

    std::string pid_force_x_file;
    std::string pid_force_y_file;
    std::string pid_force_z_file;
    std::string pid_yaw_file;

    RCLCPP_INFO(this->get_logger(), "PID Controller starting!");

    this->declare_parameter("pid_force_x_file", pid_force_x_file);
    this->declare_parameter("pid_force_y_file", pid_force_y_file);
    this->declare_parameter("pid_force_z_file", pid_force_z_file);
    this->declare_parameter("pid_yaw_file", pid_yaw_file);
    this->get_parameter("pid_force_x_file", pid_force_x_file);
    this->get_parameter("pid_force_y_file", pid_force_y_file);
    this->get_parameter("pid_force_z_file", pid_force_z_file);
    this->get_parameter("pid_yaw_file", pid_yaw_file);
            
    pid_force_x_file = ament_index_cpp::get_package_share_directory("triton_pid_controller") + "/config/" + pid_force_x_file;
    pid_force_y_file = ament_index_cpp::get_package_share_directory("triton_pid_controller") + "/config/" + pid_force_y_file;
    pid_force_z_file = ament_index_cpp::get_package_share_directory("triton_pid_controller") + "/config/" + pid_force_z_file;
    pid_yaw_file = ament_index_cpp::get_package_share_directory("triton_pid_controller") + "/config/" + pid_yaw_file;

    pid_force_x.load(pid_force_x_file);
    pid_force_y.load(pid_force_y_file);
    pid_force_z.load(pid_force_z_file);
    pid_yaw.load(pid_yaw_file);

    RCLCPP_INFO(this->get_logger(), "PID Controller successfully started!");
  }

  PidController::~PidController()
  {

  }
  void PidController::pose_update(const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    cur_pose =msg;
  }

  void PidController::control_loop()
  {
    if (!cur_pose) 
    {
      return;
    }

    auto now = std::chrono::high_resolution_clock::now();
    float dt = std::chrono::duration_cast<std::chrono::microseconds>(now - last_time_).count() / 1e6;

    auto & x = cur_pose->orientation.x;
    auto & y = cur_pose->orientation.y;
    auto & z = cur_pose->orientation.z;
    auto & w = cur_pose->orientation.w;
    
    // float cur_yaw = -std::atan2(2*y*w + 2*x*z, 1 - 2*y*y - 2*z*z);
    float cur_yaw = -std::atan2(2.0*(y*z + w*x), w*w - x*x - y*y + z*z);
    
    float yaw_error = cur_yaw;
    float pos_x_error = cur_pose->position.x;
    float pos_y_error = cur_pose->position.y;
    float pos_z_error = cur_pose->position.z;

    float forceX = pid_force_x.update(pos_x_error,dt);
    float forceY = pid_force_y.update(pos_y_error,dt);
    float forceZ = pid_force_z.update(pos_z_error,dt);
    float torqueZ = pid_yaw.update(yaw_error,dt);

    last_time_ = std::chrono::high_resolution_clock::now();

    //std::cout << "Pos: " << cur_pos << std::endl;
    //std::cout << "PID out: " << pid_out << std::endl;
    geometry_msgs::msg::Wrench wrenchOut; 
    wrenchOut.force.x = forceX;
    wrenchOut.force.y = forceY;
    wrenchOut.force.z = forceZ;
    wrenchOut.torque.x = 0;
    wrenchOut.torque.y = 0;
    wrenchOut.torque.z = torqueZ;
    //command_.header.stamp = get_clock()->now();
    //command_.velocity = {pid_out, pid_out};
    pub_->publish(wrenchOut);
  }
} // namespace triton_pid_controller

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions();
  rclcpp::spin(std::make_shared<triton_pid_controller::PidController>(options));
  rclcpp::shutdown();
  return 0;
}
