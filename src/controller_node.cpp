#include "controller_node.hpp"


using namespace pt_potential_controller;

PotentialControllerNode::PotentialControllerNode(rclcpp::NodeOptions options) : Node("potential_controller_node", options) {

    // goal
    ConstantPotential p0_0(-0.25);
    LinearPotential p0_1(-1.0);
    std::vector<PotentialPtr> p0 = {std::make_shared<ConstantPotential>(p0_0), std::make_shared<LinearPotential>(p0_1)};
    PointAnchor a0(1.0, -1.0, p0);

    // obstacle
    LinearPotential p1_0(0.5);
    std::vector<PotentialPtr> p1 = {std::make_shared<LinearPotential>(p1_0)};
    PointAnchor a1(0.0, 0.0, p1);


    std::vector<AnchorPtr> a = {std::make_shared<PointAnchor>(a0), std::make_shared<PointAnchor>(a1)};
    Scenario s(a);

    s.draw();
    rclcpp::shutdown();
}
