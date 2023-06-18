#ifndef TRITON_CONTROL__THRUST_ALLOCATOR
#define TRITON_CONTROL__THRUST_ALLOCATOR

#include <math.h>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/u_int32.hpp"

#define MAX_THRUSTERS 6

namespace triton_controls
{      

    class ThrustAllocator : public rclcpp::Node
    {

    public:

        /** Constructor
         * 
         * Creates the allocation matrix from the given parameters, then stores the 
         * pseudoinverse of that matrix and sets up pubs/subs
         * 
         * @param options ros2 node options.
         */
        explicit ThrustAllocator(const rclcpp::NodeOptions & options);

    private:

        /** Wrench message callback
         * 
         * Calculates the thrust by multiplying the pseudo inverse of the allocation
         * matrix by the vector of forces/torques given by wrench and publishing the
         * result to the output forces topic
         * 
         * thrust = pinv(A)*tau
         * 
         * @param msg wrench message with 3D force/torque to allocates
         */
        void wrenchCallback(const geometry_msgs::msg::Wrench::SharedPtr msg) const;

        /** Helper for cross product between 3D distance and force vectors
         * 
         * @param r distance vector, 3D
         * @param F force vector, 3D
         * @return tau, where tau = r X F
         */ 
        std::vector<double> cross(std::vector<double> r,std::vector<double> F);


        /** Helper for constructing the allocation matrix
         * 
         * @returns a 6 X num_thrusters allocation matrix
         */
        std::vector<std::vector<double>> createAllocMat();

        /**
         * @brief Helper for converting a force (in N) to a power level (0-31) for 
         * a thruster. 
         * 
         * @param force 
         * @return uint32_t 
         */
        uint32_t forceToLevel(double force) const;

        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr forces_pub_;  
        rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr signals_pub_;  

        rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr forces_sub_; 

        cv::Mat pinv_alloc_;

        int num_thrusters_ = 0;
        int bits_per_thruster_ = 0;
        int encode_levels_ = 0;
        double max_fwd_ = 0;
        double max_rev_ = 0;
        std::vector<double> x_lens_;
        std::vector<double> y_lens_;
        std::vector<double> z_lens_;
        std::vector<double> x_contribs_;
        std::vector<double> y_contribs_;
        std::vector<double> z_contribs_;
    };
    
} // namespace triton_controls

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(triton_controls::ThrustAllocator)

#endif  //TRITON_CONTROL__THRUST_ALLOCATOR
