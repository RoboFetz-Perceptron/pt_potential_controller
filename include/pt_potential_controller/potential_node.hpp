#ifndef PT_POTENTIAL_PLANNER__POTENTIAL_NODE_HPP
#define PT_POTENTIAL_PLANNER__POTENTIAL_NODE_HPP

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include <force_sim.hpp>


namespace pt_potential_controller {

    class PotentialControllerNode : public rclcpp::Node {
        public:
            PotentialControllerNode(rclcpp::NodeOptions options);
    };
}

#endif