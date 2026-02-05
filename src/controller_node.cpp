#include "controller_node.hpp"


using namespace pt_potential_controller;

PotentialControllerNode::PotentialControllerNode(rclcpp::NodeOptions options) : Node("potential_controller_node", options) {
    RCLCPP_INFO(this->get_logger(), "constructor");

    PointAnchor point_anchor;
    Anchor* anchor = (Anchor*)&point_anchor;

    RCLCPP_INFO(this->get_logger(), "before: x=%lf y=%lf", point_anchor.p_.x(), point_anchor.p_.y());
    anchor->update_point(tuw::Point2D(1.0, 2.0));
    RCLCPP_INFO(this->get_logger(), "after: x=%lf y=%lf", point_anchor.p_.x(), point_anchor.p_.y());
}
