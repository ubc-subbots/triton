#include "triton_gazebo/underwater_camera.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

namespace triton_gazebo
{

    UnderwaterCamera::UnderwaterCamera(const rclcpp::NodeOptions & options)
    : Node("underwater_camera", options) 
    {

        rmw_qos_profile_t subscriber_qos_profile = rmw_qos_profile_sensor_data;
        rmw_qos_profile_t publisher_qos_profile = rmw_qos_profile_default;

        underwater_image_pub_ = image_transport::create_publisher(this, 
            "front_camera/underwater/image_raw", 
            publisher_qos_profile);

        image_pub_ = image_transport::create_publisher(this, 
            "repub/image_raw", 
            publisher_qos_profile);

        depth_pub_ = image_transport::create_publisher(this, 
            "repub/depth/image_raw", 
            publisher_qos_profile);

        image_sub_.subscribe(this,
            "front_camera/image_raw", 
            "raw",
            subscriber_qos_profile);

        depth_sub_.subscribe(this,
             "front_camera/depth/image_raw", 
             "raw",
             subscriber_qos_profile);

        approx_sync_ = std::make_shared<ApproxSync>(
            ApproxPolicy(5),
            image_sub_, 
            depth_sub_);

        approx_sync_->registerCallback(
            std::bind(&UnderwaterCamera::syncCallback, this, _1, _2));

        this->declare_parameter("rho");
        this->declare_parameter("irradiance_transmission");
        this->declare_parameter("spectral_sensitivity_blue");
        this->declare_parameter("spectral_sensitivity_red");
        this->declare_parameter("spectral_sensitivity_green");
        this->declare_parameter("illumination_irradiance");

        std::vector<double> rho_vals;
        std::vector<double> Beta_vals;
        std::vector<double> S_b_vals;
        std::vector<double> S_g_vals;
        std::vector<double> S_r_vals;
        std::vector<double> E_0_vals;

        this->get_parameter("rho", rho_vals);
        this->get_parameter("irradiance_transmission", Beta_vals);
        this->get_parameter("spectral_sensitivity_blue", S_b_vals);
        this->get_parameter("spectral_sensitivity_red", S_g_vals);
        this->get_parameter("spectral_sensitivity_green", S_r_vals);
        this->get_parameter("illumination_irradiance", E_0_vals);

        if (rho_vals.size() != 13||
            Beta_vals.size() != 13||
            S_b_vals.size() != 13||
            S_g_vals.size() != 13||
            S_r_vals.size() != 13||
            E_0_vals.size() != 13)
        {
            RCLCPP_ERROR(this->get_logger(), "Parameters are incorrect.");
        }
        rho_ = Array13f(13);
        Beta_ = Array13f(13);
        S_b_ = Array13f(13);
        S_g_ = Array13f(13);
        S_r_ = Array13f(13);
        E_0_ = Array13f(13);
        for (int i = 0; i < 13; i++){
            rho_(i) = rho_vals[i];
            Beta_(i) = Beta_vals[i];
            S_b_(i) = S_b_vals[i];
            S_g_(i) = S_g_vals[i];
            S_r_(i) = S_r_vals[i];
            E_0_(i) = E_0_vals[i];
        }
        Beta_/=100.0;
        Beta_ = Array13f::Ones()-Beta_;//We input the irradiance transmission %, but we want the attenuation factor

        //Randomize parameters
        srand(time(NULL));//set seed
        float d_min = 0.5;
        float d_max = 1.0;
        d_ = ((float) rand())/RAND_MAX *(d_max-d_min) + d_min;

        auto preCalculate = [](float B_c_max, float B_c_min){
            float B_c = ((float) rand())/RAND_MAX *(B_c_max-B_c_min) + B_c_min;
            return B_c;
        };
        
        B_b_ = preCalculate(0.7,0.8);
        B_g_ = preCalculate(0.7,0.8);
        B_r_ = preCalculate(0.4,0.5);

        //Precalculate integrals
        auto preCalculate2 = [&](Array13f& S_c, float& log_trapz_num_cz, float& T_cd){
            Array13f num_cd = S_c * rho_ * E_0_;
            Array13f den_cd = S_c * rho_ * E_0_ * (Beta_ * -d_).exp();
            float Beta_cd = log(simps(num_cd)/simps(den_cd))/d_;
            T_cd = exp(Beta_cd*-d_);

            Array13f num_cz = S_c* rho_ * E_0_ * (Beta_ * -d_).exp();
            log_trapz_num_cz = log(simps(num_cz));
        };

        preCalculate2(S_b_, log_trapz_num_bz_, T_bd_);
        preCalculate2(S_g_, log_trapz_num_gz_, T_gd_);
        preCalculate2(S_r_, log_trapz_num_rz_, T_rd_);

        RCLCPP_INFO(this->get_logger(), "Underwater Camera succesfully started!");
    }


    // void UnderwaterCamera::syncCallback(const ImageMsg & image_msg, const ImageMsg & depth_msg)
    // {   
    //     float image_stamp = image_msg->header.stamp.sec+image_msg->header.stamp.nanosec*1e-9;
    //     float depth_stamp = depth_msg->header.stamp.sec+depth_msg->header.stamp.nanosec*1e-9;
    //     float time_diff = std::abs(image_stamp-depth_stamp);
    //     if (time_diff > 0)
    //     {
    //         // Show non zero time difference between image pairs, never seems to be above 0.07s
    //         RCLCPP_INFO(this->get_logger(), "Non-Zero Time Difference: [%.5f]", time_diff);
    //     }
    //     image_pub_.publish(image_msg);
    //     depth_pub_.publish(depth_msg);
    //     underwaterImageSynthesis(image_msg, depth_msg);
    // }

    void UnderwaterCamera::syncCallback(const ImageMsg & image_msg, const ImageMsg & depth_msg)
    {   
        float image_stamp = image_msg->header.stamp.sec+image_msg->header.stamp.nanosec*1e-9;
        float depth_stamp = depth_msg->header.stamp.sec+depth_msg->header.stamp.nanosec*1e-9;
        float time_diff = std::abs(image_stamp-depth_stamp);
        if (time_diff > 0)
        {
            // Show non zero time difference between image pairs, never seems to be above 0.07s
            RCLCPP_INFO(this->get_logger(), "Non-Zero Time Difference: [%.5f]", time_diff);
        }

        underwaterImageSynthesis(image_msg, depth_msg);
        image_pub_.publish(image_msg);
        depth_pub_.publish(depth_msg);
    }


    void UnderwaterCamera::underwaterImageSynthesis(const ImageMsg & image_msg, const ImageMsg & depth_msg)
    {
        auto timer = this->get_clock()->now();

        auto message = sensor_msgs::msg::Image();

        cv_bridge::CvImagePtr rgb_ptr;
        try {
            rgb_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(),"cv_bridge exception: %s", e.what());
            return;
        }
        
        cv_bridge::CvImagePtr depth_ptr;
        try {
            depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(),"cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat rgb = rgb_ptr->image;
        cv::Mat depth = depth_ptr->image;

        if (rgb.rows!=depth.rows||rgb.cols!=depth.cols){
            RCLCPP_ERROR(get_logger(),"RGB and Depth dimensions do not match");
            return;
        }

        //normalize range
        cv::normalize(depth,depth,0.5,14,cv::NORM_MINMAX);

        //split bgr channels
        std::array<cv::Mat,3> channels;
        cv::split(rgb,channels);
    
        float * depth_data = (float *) depth.data;
        auto synthesizeChannel = [&](float log_trapz_num_cz, float T_cd, float B_c, Array13f& S_c, cv::Mat& channel){
            uint8_t * synth_data = (uint8_t*) channel.data;
            Array13f S_c_rho_E_0 = S_c * rho_ * E_0_;
            for (int i = 0; i<depth.rows; i++){
                for (int j = 0; j<depth.cols; j++){
                    float z = depth_data[i*depth.cols+j];
                    Array13f den_cz = S_c_rho_E_0 * (Beta_ * -(d_+z)).exp();
                    float Beta_cz = (log_trapz_num_cz-log(simps(den_cz)))/z;
                    float T_cz = exp(Beta_cz*-z);
                    
                    float pix_normalized = (float) synth_data[i*depth.cols+j]/255.0f;
                    synth_data[i*depth.cols+j] = 255.0f*(pix_normalized*T_cd*T_cz+B_c*T_cd*(1-T_cz));
                }
            }
        };

        std::array<cv::Mat,3> synth_channels;
        synthesizeChannel(log_trapz_num_bz_,T_bd_,B_b_,S_b_,channels[0]);
        synthesizeChannel(log_trapz_num_gz_,T_gd_,B_g_,S_g_,channels[1]);
        synthesizeChannel(log_trapz_num_rz_,T_rd_,B_r_,S_r_,channels[2]);
        
        cv_bridge::CvImage pub_image;
        pub_image.image = cv::Mat(rgb.rows,rgb.cols,CV_8UC3);
        cv::merge(channels,pub_image.image);
        pub_image.encoding = sensor_msgs::image_encodings::BGR8;
        pub_image.toImageMsg(message);
        message.header = image_msg->header;
        underwater_image_pub_.publish(message);

        auto runtime = this->get_clock()->now() - timer;
        RCLCPP_INFO(this->get_logger(), "Processing took: %lfs", runtime.seconds());
    }

} // namespace triton_gazebo

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions();
  rclcpp::spin(std::make_shared<triton_gazebo::UnderwaterCamera>(options));
  rclcpp::shutdown();
  return 0;
}