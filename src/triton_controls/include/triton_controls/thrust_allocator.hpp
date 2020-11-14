#ifndef TRITON_CONTROL__THRUST_ALLOCATOR
#define TRITON_CONTROL__THRUST_ALLOCATOR

#include <math.h>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

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

        /** Cosine and sine in degrees
         * 
         * @param deg angle in degrees
         * @return cose/sine of the angle
         */
        inline float cosd(float deg) { return cos( deg*M_PI/180.0); }
        inline float sind(float deg) { return cos( deg*M_PI/180.0); }

        /** Helper for elementwise multiplication of same size vectors
         * 
         * @param u first vector
         * @param v second vector
         * @return w, where w is the elementwise multiplication of u and v
         */
        std::vector<double> mulVecs(std::vector<double> u,std::vector<double> v);

        /** Helper for elementwise addition of same size vectors
         * 
         * @param u first vector
         * @param v second vector
         * @return w, where w is the elementwise addition of u and v
         */
        std::vector<double> addVecs(std::vector<double> u,std::vector<double> v);

        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr forces_pub_;  
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr signals_pub_;  

        rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr forces_sub_; 

        cv::Mat pinv_alloc_;

        int num_thrusters_ = 0;
        std::vector<double> x_lens_;
        std::vector<double> y_lens_;
        std::vector<double> z_lens_;
        std::vector<std::vector<double>> contribs_; 
    };
    
} // namespace triton_controls

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(triton_controls::ThrustAllocator)

#endif  //TRITON_CONTROL__THRUST_ALLOCATOR
