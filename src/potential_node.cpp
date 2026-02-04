#include "potential_node.hpp"


using namespace pt_potential_controller;

PotentialControllerNode::PotentialControllerNode(rclcpp::NodeOptions options) : Node("potential_controller_node", options) {
    RCLCPP_INFO(this->get_logger(), "constructor");
}
