#include "triton_controls/thrust_allocator.hpp"
using std::placeholders::_1;

namespace triton_controls
{

  ThrustAllocator::ThrustAllocator(const rclcpp::NodeOptions & options)
  : Node("thrust_allocator", options),
    x_lens_ (MAX_THRUSTERS, 0),
    y_lens_ (MAX_THRUSTERS, 0),
    z_lens_ (MAX_THRUSTERS, 0),
    contribs_ (3, std::vector<double>(MAX_THRUSTERS))
  {   
      this->declare_parameter<int>("num_thrusters", num_thrusters_);

      this->get_parameter("num_thrusters", num_thrusters_);
      if (num_thrusters_ > MAX_THRUSTERS){
        RCLCPP_ERROR(this->get_logger(), "Attempted to configure too many thrusters, results will not be desirable");
      }

      std::string names[MAX_THRUSTERS] = {"t1", "t2", "t3", "t4", "t5", "t6"};
    
      for (int i = 0; i < MAX_THRUSTERS; i++)
      {
        this->declare_parameter<float>(names[i]+".contrib.x", contribs_[0][i]);
        this->declare_parameter<float>(names[i]+".contrib.y", contribs_[1][i]);
        this->declare_parameter<float>(names[i]+".contrib.z", contribs_[2][i]);
        this->declare_parameter<float>(names[i]+".lx", x_lens_[i]);
        this->declare_parameter<float>(names[i]+".ly", y_lens_[i]);
        this->declare_parameter<float>(names[i]+".lz", z_lens_[i]);

        this->get_parameter(names[i]+".contrib.x", contribs_[0][i]);
        this->get_parameter(names[i]+".contrib.y", contribs_[1][i]);
        this->get_parameter(names[i]+".contrib.z", contribs_[2][i]);
        this->get_parameter(names[i]+".lx", x_lens_[i]);
        this->get_parameter(names[i]+".ly", y_lens_[i]);
        this->get_parameter(names[i]+".lz", z_lens_[i]);
      } 

      std::vector<std::vector<double>> alloc_vec;
      alloc_vec.push_back(contribs_[0]);
      alloc_vec.push_back(contribs_[1]);
      alloc_vec.push_back(contribs_[2]);
      alloc_vec.push_back(addVecs(mulVecs(contribs_[2], y_lens_),
                                  mulVecs(contribs_[1], z_lens_)));
      alloc_vec.push_back(addVecs(mulVecs(contribs_[2], x_lens_),
                                  mulVecs(contribs_[0], z_lens_)));
      alloc_vec.push_back(addVecs(mulVecs(contribs_[1], x_lens_),
                                  mulVecs(contribs_[0], y_lens_)));

      cv::Mat alloc_mat (alloc_vec.size(), alloc_vec[0].size(), CV_64FC1);
      for (int i = 0; i < alloc_mat.rows; i++)
        for (int j = 0; j < alloc_mat.cols; j++)
          alloc_mat.at<double>(i,j) = alloc_vec[i][j];
      
      RCLCPP_INFO(this->get_logger(), "Allocation Matrix:");
      std::cout << alloc_mat << std::endl;

      cv::invert(alloc_mat, pinv_alloc_, cv::DECOMP_SVD);

      forces_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "controls/output_forces", 10);

      signals_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "controls/signals", 10);

      forces_sub_ = this->create_subscription<geometry_msgs::msg::Wrench>(
        "controls/input_forces", 10, std::bind(&ThrustAllocator::wrenchCallback, this, _1));

      RCLCPP_INFO(this->get_logger(), "Thrust Allocator succesfully started!");
  }


  void ThrustAllocator::wrenchCallback(const geometry_msgs::msg::Wrench::SharedPtr msg) const
  {
      double tau_arr[6] = {
        msg->force.x, msg->force.y, msg->force.z,
        msg->torque.x, msg->torque.y, msg->torque.z,
      };

      cv::Mat tau_mat (6, 1, CV_64F);
      for (int i = 0; i < 6; i++)
        tau_mat.at<double>(i, 0) = tau_arr[i];

      cv::Mat thrust_mat =  pinv_alloc_*tau_mat;

      std::vector<double> thrust;
      for (int i = 0; i < 6 ; i++)
        thrust.push_back(thrust_mat.at<double>(i,0));

      auto forces_msg = std_msgs::msg::Float64MultiArray();
      forces_msg.data = thrust;
      forces_pub_->publish(forces_msg);
  }


  std::vector<double> ThrustAllocator::mulVecs(std::vector<double> u, std::vector<double> v)
  {   
      std::vector<double> w;
      for (size_t i = 0; i < u.size(); i++)
      {
        w.push_back(u[i]*v[i]);
      }
      return w;
  }


  std::vector<double> ThrustAllocator::addVecs(std::vector<double> u, std::vector<double> v)
  {   
      std::vector<double> w;
      for (size_t i = 0; i < u.size(); i++)
      {
        w.push_back(u[i]+v[i]);
      }
      return w;
  }
    
} // namespace triton_controls


int main(int argc, char * argv[])
{
  try {
    rclcpp::init(argc, argv);
    auto options = rclcpp::NodeOptions();
    rclcpp::spin(std::make_shared<triton_controls::ThrustAllocator>(options));
    rclcpp::shutdown();
  } catch (rclcpp::exceptions::RCLError const&){} // during testing sometimes throws error
  return 0;
}
