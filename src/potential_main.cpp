#include <chrono>
#include <thread>
#include <math.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/duration.hpp>

#include "potential_node.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pt_potential_controller::PotentialControllerNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  //std::cout << "aaa" << std::endl;
  return 0;
}