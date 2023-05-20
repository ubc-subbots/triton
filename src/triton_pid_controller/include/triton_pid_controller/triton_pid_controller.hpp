#ifndef TRITON_PID_CONTROLLER__
#define TRITON_PID_CONTROLLER__
#pragma once

#include <string>
#include <yaml-cpp/yaml.h>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace triton_pid_controller
{

class PidController : public rclcpp::Node
{
public:
    explicit PidController(const rclcpp::NodeOptions & options);

    ~PidController();

private:

    void control_loop();
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_;

    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr pub_;
    geometry_msgs::msg::Pose::SharedPtr cur_pose ;

    std::chrono::time_point<std::chrono::high_resolution_clock> last_time_;
    rclcpp::TimerBase::SharedPtr control_loop_timer_;
    void pose_update(const geometry_msgs::msg::Pose::SharedPtr msg);
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

    PID pid_force_x;
    PID pid_force_y;
    PID pid_force_z;
    PID pid_yaw;
};
}  // namespace triton_pid_controller

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(triton_pid_controller::PidController)

#endif
