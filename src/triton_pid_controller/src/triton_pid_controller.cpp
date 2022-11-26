#include <triton_pid_controller/triton_pid_controller.h>
#include <chrono>
#include <cmath>
#include "triton_interfaces/msg/float.hpp" 

using std::placeholders::_1;
using namespace std::chrono_literals;

PidController::PidController() : Node("pid_controller")
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

  auto now = std::chrono::high_resolution_clock::now();
  float dt = std::chrono::duration_cast<std::chrono::microseconds>(now - last_time_).count() / 1e6;

  auto & x = cur_pose->orientation.x;
  auto & y = cur_pose->orientation.y;
  auto & z = cur_pose->orientation.z;
  auto & w = cur_pose->orientation.w;
  
  float cur_yaw = -std::atan2(2*y*w + 2*x*z, 1 - 2*y*y - 2*z*z);
  
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

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PidController>());
  rclcpp::shutdown();
  return 0;
}
