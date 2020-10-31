#include "triton_gazebo/underwater_camera.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Core>

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
            "image_raw", 
            publisher_qos_profile);

        image_pub_ = image_transport::create_publisher(this, 
            "gazebo/image_raw", 
            publisher_qos_profile);

        depth_pub_ = image_transport::create_publisher(this, 
            "gazebo/depth/image_raw", 
            publisher_qos_profile);

        image_sub_.subscribe(this,
            "/triton/gazebo_drivers/front_camera/image_raw", 
            "raw",
            subscriber_qos_profile);

        depth_sub_.subscribe(this,
             "/triton/gazebo_drivers/front_camera/depth/image_raw", 
             "raw",
             subscriber_qos_profile);

        approx_sync_ = std::make_shared<ApproxSync>(
            ApproxPolicy(5),
            image_sub_, 
            depth_sub_);

        approx_sync_->registerCallback(
            std::bind(&UnderwaterCamera::syncCallback, this, _1, _2));

        RCLCPP_INFO(this->get_logger(), "Underwater Camera succesfully started!");
    }


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
        image_pub_.publish(image_msg);
        depth_pub_.publish(depth_msg);
        underwaterImageSynthesis(image_msg, depth_msg);
    }


    void UnderwaterCamera::underwaterImageSynthesis(const ImageMsg & image_msg, const ImageMsg & depth_msg)
    {
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

        //from 400 to 700 nm in intervals of 25 nm
        typedef Eigen::Array<float,13,1> Array13f;

        //trapezoidal integration of evenly spaced vector
        auto trapz = [](float dx, Array13f & funk){
            Eigen::ArrayXf traps = (funk.tail(funk.size()-1) + funk.head(funk.size()-1))/2;
            return traps.sum()*dx;
        };
    
        Array13f rho = Array13f::Ones();
        Array13f Beta(13);
        Beta<<97.2,97.8,98.1,98.2,97.2,96.1,94.2,92,85,74,70,66,59;
        Beta/=100.0;
        Array13f S_b(13);
        S_b<<0.35,0.55,0.675,0.65,0.45,0.2,0.1,0.05,0.03,0.03,0.05,0.075,0.09;
        Array13f S_g(13);
        S_g<<0.05,0.04,0.06,0.45,0.85,0.93,0.9,0.79,0.5,0.25,0.18,0.2,0.31;
        Array13f S_r(13);
        S_r<<0.075,0.045,0.025,0.04,0.05,0.09,0.075,0.5,1,0.975,0.925,0.8,0.725;
        Array13f E_0(13);
        E_0<<0.83989,0.99312,1.2881,1.3755,1.3391,1.3859,1.3648,1.3225,1.3278,1.2667,1.2299,1.2639,1.1636;

        float d_min = 0.5;
        float d_max = 1.0;
        float d = ((float) rand())/RAND_MAX *(d_max-d_min) + d_min;

        float * depth_data = (float *) depth.data;
        auto synthesizeChannel = [&](float B_c_min, float B_c_max, Array13f& S_c, cv::Mat& channel){
            cv::Mat synth = channel.clone();
            uint8_t * synth_data = (uint8_t*) synth.data;
            float B_c = ((float) rand())/RAND_MAX *(B_c_max-B_c_min) + B_c_min;
            Array13f num_cd = S_c * rho * E_0;
            Array13f den_cd = S_c * rho * E_0 * (Beta * -d).exp();
            float Beta_cd = log(trapz(25,num_cd)/trapz(25,den_cd))/d;
            float T_cd = exp(Beta_cd*-d);
            for (int i = 0; i<depth.rows; i++){
                for (int j = 0; j<depth.rows; j++){
                    float z = depth_data[i*depth.cols+j];
                    Array13f num_cz = S_c* rho * E_0 * (Beta * -d).exp();
                    Array13f den_cz = S_c * rho * E_0 * (Beta * -(d+z)).exp();
                    float Beta_cz = log(trapz(25,num_cz)/trapz(25,den_cz))/z;
                    float T_cz = exp(Beta_cz*-z);
                    
                    synth_data[i*depth.cols+j] = ((float) synth_data[i*depth.cols+j])*T_cd*T_cz+B_c*T_cd*(1-T_cz);
                }
            }
            return synth;
        };

        std::array<cv::Mat,3> synth_channels;
        synth_channels[0] = synthesizeChannel(0.7,0.8,S_b,channels[0]);
        synth_channels[1] = synthesizeChannel(0.7,0.8,S_g,channels[1]);
        synth_channels[2] = synthesizeChannel(0.4,0.5,S_r,channels[2]);

        cv_bridge::CvImage pub_image;
        pub_image.image = cv::Mat(rgb.rows,rgb.cols,CV_8UC3);
        cv::merge(synth_channels,pub_image.image);
        pub_image.encoding = sensor_msgs::image_encodings::BGR8;
        pub_image.toImageMsg(message);
        underwater_image_pub_.publish(message);
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