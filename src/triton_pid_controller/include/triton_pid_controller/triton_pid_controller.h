#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <yaml-cpp/yaml.h>
#include "triton_interfaces/msg/float.hpp" 

class PidController : public rclcpp::Node
{
public:
    PidController();

    ~PidController();

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void motor_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void cur_pitch_update_callback(const triton_interfaces::msg::Float::SharedPtr msg);
    void target_update_callback(const triton_interfaces::msg::Float::SharedPtr msg);
    void control_loop();
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_motor_state_;
    rclcpp::Subscription<triton_interfaces::msg::Float>::SharedPtr sub_target_;
    rclcpp::Subscription<triton_interfaces::msg::Float>::SharedPtr sub_cur_pitch_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;

    sensor_msgs::msg::Imu::SharedPtr imu_state_;
    sensor_msgs::msg::JointState::SharedPtr motor_state_;
    sensor_msgs::msg::JointState command_;

    std::chrono::time_point<std::chrono::high_resolution_clock> last_time_;
    rclcpp::TimerBase::SharedPtr control_loop_timer_;

    struct PID
    {
        void load(const std::string &path)
        {
            YAML::Node config = YAML::LoadFile(path);
            Kp = config["Kp"].as<float>();
            Ki = config["Ki"].as<float>();
            Kd = config["Kd"].as<float>();
        }

        float Kp = 0;
        float Ki = 0;
        float Kd = 0;
        float sum_error = 0;
        float last_error = 0;
        float update(float error, float dt)
        {
            float diff_error = error - last_error;
            sum_error += error * dt;
            float ret = Kp * error + Ki * sum_error + Kd * diff_error / dt;
            last_error = error;
            return ret;
        }
    };

    PID pid_pitch_;
    PID pid_pos_;
    float pitch_target_;
    float cur_pitch_;
};
