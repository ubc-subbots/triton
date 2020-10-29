#include "triton_controls/thrust_allocator.hpp"
using std::placeholders::_1;

namespace triton_controls
{


  ThrustAllocator::ThrustAllocator(const rclcpp::NodeOptions & options)
  : Node("thrust_allocator", options) 
  {

      double alpha = 45.0;
      std::pair<double, double> l1(10.833*cosd(52.704), 10.833*sind(52.704));
      std::pair<double, double> l2(11.238*cosd(54.261), 11.238*sind(54.261));
      std::pair<double, double> l3(11.466*cosd(48.73), 11.466*sind(48.73));
      std::pair<double, double> l4(11.85*cosd(50.34), 11.85*sind(50.34));
      double l5 = 8.744;
      double l6 = 8.239;

      double alloc_arr[10][10] = {
          {-1, 0, -1, 0, 1, 0, 1, 0, 0, 0},
          {0, -1, 0, 1, 0, -1, 0 ,1, 0, 0},
          {0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
          {0, 0, 0, 0, 0, 0, 0, 0, -l5, l6},
          {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
          {-l1.first,-l1.second,l2.first,l2.second,
            l3.first,l3.second,-l4.first,-l4.second,0,0},
          {1/cosd(alpha), -1/sind(alpha), 0, 0, 0, 0, 0, 0, 0, 0},
          {0, 0, 1/cosd(alpha), -1/sind(alpha), 0, 0, 0, 0, 0, 0},
          {0, 0, 0, 0, 1/cosd(alpha), -1/sind(alpha), 0, 0, 0, 0},
          {0, 0, 0, 0, 0, 0, 1/cosd(alpha), -1/sind(alpha), 0, 0}
      };

      cv::Mat alloc_mat (10,10, CV_64F);
      for (int i = 0; i < 10; i++)
        for (int j = 0; j < 10; j++)
          alloc_mat.at<double>(i,j) = alloc_arr[i][j];

      cv::invert(alloc_mat, alloc, cv::DECOMP_SVD);

      forces_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "output_forces", 10);

      signals_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "signals", 10);

      forces_sub_ = this->create_subscription<geometry_msgs::msg::Wrench>(
        "/triton/controls/input_forces", 10, std::bind(&ThrustAllocator::wrenchCallback, this, _1));

      RCLCPP_INFO(this->get_logger(), "Thrust Allocator succesfully started!");
  }


  void ThrustAllocator::wrenchCallback(const geometry_msgs::msg::Wrench::SharedPtr msg) const
  {
      double tau_arr[10] = {
        msg->force.x, msg->force.y, msg->force.z,
        msg->torque.x, msg->torque.y, msg->torque.z,
        0,0,0,0
      };

      cv::Mat tau_mat (10, 1, CV_64F);
      for (int i = 0; i < 10; i++)
        tau_mat.at<double>(i, 0) = tau_arr[i];

      cv::Mat u_mat =  alloc*tau_mat;

      std::vector<double> u;
      for (int i = 0; i < 10; i++)
        u.push_back(u_mat.at<double>(i,0));

      double thrust_arr[6] = {
        getThrust(u[0], u[1]), getThrust(u[2], u[3]),
        getThrust(u[4], u[5]), getThrust(u[6], u[7]),
        u[8], u[9]
      };
      std::vector<double> thrust (
        thrust_arr, thrust_arr + sizeof(thrust_arr)/sizeof(thrust_arr[0]));

      auto forces_msg = std_msgs::msg::Float64MultiArray();
      forces_msg.data = thrust;
      forces_pub_->publish(forces_msg);
  }
    
} // namespace triton_controls


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions();
  rclcpp::spin(std::make_shared<triton_controls::ThrustAllocator>(options));
  rclcpp::shutdown();
  return 0;
}
