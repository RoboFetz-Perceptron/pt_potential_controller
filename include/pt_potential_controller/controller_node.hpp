#ifndef PT_POTENTIAL_PLANNER__POTENTIAL_NODE_HPP
#define PT_POTENTIAL_PLANNER__POTENTIAL_NODE_HPP

#include <exception>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include "yaml-cpp/yaml.h"

#include <pt_msgs/srv/load_scenario.hpp>

#include <anchor.hpp>
#include <scenario.hpp>


namespace pt_potential_controller {

    using Binding = std::tuple<AnchorPtr, std::string, std::string>; // anchor, type, topic
    using BindingVec = std::vector<Binding>;
    using BindingVecPtr = std::shared_ptr<BindingVec>;

    using TwistMsg = geometry_msgs::msg::Twist;
    using PointMsg = geometry_msgs::msg::Point;
    using PoseMsg = geometry_msgs::msg::Pose;
    using LoadScenarioSrv = pt_msgs::srv::LoadScenario;

    class PotentialControllerNode : public rclcpp::Node {
        public:
            PotentialControllerNode(rclcpp::NodeOptions options);

            ScenarioPtr scenario_;
            BindingVecPtr in_bindings_;
            BindingVecPtr out_bindings_;

            //std::map<AnchorPtr, rclcpp::SubscriberBase::SharedPtr>
            //std::map<AnchorPtr, rclcpp::PublisherBase::SharedPtr> publishers_;

            

            // ROS parameters
            std::string scenario_path_;
            std::string control_anchor_;
            std::string rotation_target_;
            double freq_ = 10.0;
            //double max_vel_x_ = 1.0
            //double max_vel_y_ = 1.0
            //double max_vel_w_ = 3.14
            double w_pid_p_ = 1.0;
            double w_pid_i_ = 0.1;
            double w_pid_d_ = 0.1;


            // ROS interfaces
            std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
            rclcpp::Publisher<TwistMsg>::SharedPtr twist_publisher_;
            rclcpp::TimerBase::SharedPtr twist_timer_;
            rclcpp::Service<LoadScenarioSrv>::SharedPtr load_scenario_server_;

            std::unique_ptr<std::thread> vis_thread_;

        
        private:
            void control_loop();

            bool load_scenario(std::string path);
            void load_anchors(YAML::Node anchors_map, Scenario &scenario);//,
                              //BindingVec &in_bindings, BindingVec &out_bindings);
            void load_potentials(YAML::Node potentials_map, AnchorPtr &anchor);
            void load_vis(YAML::Node vis_map, Scenario &scenario);
    };
}

#endif