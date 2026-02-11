#ifndef PT_POTENTIAL_PLANNER__POTENTIAL_NODE_HPP
#define PT_POTENTIAL_PLANNER__POTENTIAL_NODE_HPP

#include <rclcpp/rclcpp.hpp>

#include <exception>
#include "yaml-cpp/yaml.h"

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include <anchor.hpp>
#include <scenario.hpp>


namespace pt_potential_controller {

    using Binding = std::tuple<AnchorPtr, std::string, std::string>; // anchor, type, topic
    using BindingVec = std::vector<Binding>;
    using BindingVecPtr = std::shared_ptr<BindingVec>;

    class PotentialControllerNode : public rclcpp::Node {
        public:
            PotentialControllerNode(rclcpp::NodeOptions options);

            ScenarioPtr scenario_;
            BindingVecPtr in_bindings_;
            BindingVecPtr out_bindings_;
        
        
        private:
            bool load_scenario(std::string path);
            void load_anchors(YAML::Node anchors_map, Scenario &scenario,
                              BindingVec &in_bindings, BindingVec &out_bindings);
            void load_potentials(YAML::Node potentials_map, AnchorPtr &anchor);
    };
}

#endif