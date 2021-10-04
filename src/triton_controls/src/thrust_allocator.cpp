#include "triton_controls/thrust_allocator.hpp"
using std::placeholders::_1;

namespace triton_controls
{

  ThrustAllocator::ThrustAllocator(const rclcpp::NodeOptions & options)
  : Node("thrust_allocator", options),
    x_lens_ (MAX_THRUSTERS, 0),
    y_lens_ (MAX_THRUSTERS, 0),
    z_lens_ (MAX_THRUSTERS, 0),
    x_contribs_ (MAX_THRUSTERS, 0),
    y_contribs_ (MAX_THRUSTERS, 0),
    z_contribs_ (MAX_THRUSTERS, 0)
  {
      this->declare_parameter<int>("num_thrusters", num_thrusters_);

      this->get_parameter("num_thrusters", num_thrusters_);
      if (num_thrusters_ > MAX_THRUSTERS){
        RCLCPP_ERROR(this->get_logger(), "Attempted to configure too many thrusters, results will not be desirable");
      }

      std::string names[MAX_THRUSTERS] = {"t1", "t2", "t3", "t4", "t5", "t6"};
    
      for (int i = 0; i < MAX_THRUSTERS; i++)
      {
        this->declare_parameter<double>(names[i]+".contrib.x", x_contribs_[i]);
        this->declare_parameter<double>(names[i]+".contrib.y", y_contribs_[i]);
        this->declare_parameter<double>(names[i]+".contrib.z", z_contribs_[i]);
        this->declare_parameter<double>(names[i]+".lx", x_lens_[i]);
        this->declare_parameter<double>(names[i]+".ly", y_lens_[i]);
        this->declare_parameter<double>(names[i]+".lz", z_lens_[i]);

        this->get_parameter<double>(names[i]+".contrib.x", x_contribs_[i]);
        this->get_parameter<double>(names[i]+".contrib.y", y_contribs_[i]);
        this->get_parameter<double>(names[i]+".contrib.z", z_contribs_[i]);
        this->get_parameter<double>(names[i]+".lx", x_lens_[i]);
        this->get_parameter<double>(names[i]+".ly", y_lens_[i]);
        this->get_parameter<double>(names[i]+".lz", z_lens_[i]);
      } 

      std::vector<std::vector<double>> alloc_vec =  createAllocMat();

      cv::Mat alloc_mat (alloc_vec.size(), alloc_vec[0].size(), CV_64FC1);
      for (int i = 0; i < alloc_mat.rows; i++)
        for (int j = 0; j < alloc_mat.cols; j++)
          alloc_mat.at<double>(i,j) = alloc_vec[i][j];
      
      RCLCPP_INFO(this->get_logger(), "Allocation Matrix:");
      std::cout << alloc_mat << std::endl;

      cv::invert(alloc_mat, pinv_alloc_, cv::DECOMP_SVD);

      forces_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "output_forces", 10);

      signals_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "signals", 10);

      forces_sub_ = this->create_subscription<geometry_msgs::msg::Wrench>(
        "input_forces", 10, std::bind(&ThrustAllocator::wrenchCallback, this, _1));

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

  std::vector<double> ThrustAllocator::cross(std::vector<double> r,std::vector<double> F){
      std::vector<double> tau;
      tau.push_back(r[1]*F[2] - r[2]*F[1]);
      tau.push_back(r[2]*F[0] - r[0]*F[2]);
      tau.push_back(r[0]*F[1] - r[1]*F[0]);
      return tau;
  }

  std::vector<std::vector<double>> ThrustAllocator::createAllocMat(){
      std::vector<std::vector<double>> alloc_mat;
      for (int i = 0; i < num_thrusters_; i++){
        std::vector<double> F = {x_contribs_[i], y_contribs_[i], z_contribs_[i]};
        std::vector<double> r = {x_lens_[i], y_lens_[i], z_lens_[i]};
        std::vector<double> tau = cross(r, F);
        F.insert(F.end(), tau.begin(), tau.end());
        alloc_mat.push_back(F);
      }
      std::vector<std::vector<double>> alloc_mat_trans(6, std::vector<double>(num_thrusters_));
      for(int i = 0; i < num_thrusters_; ++i){
        for(int j = 0; j < 6; ++j){
            alloc_mat_trans[j][i]=alloc_mat[i][j];
        }
      }
      return alloc_mat_trans;
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
