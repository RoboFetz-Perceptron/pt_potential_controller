#ifndef PT_POTENTIAL_PLANNER__POTENTIAL_NODE_HPP
#define PT_POTENTIAL_PLANNER__POTENTIAL_NODE_HPP

#include <exception>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include "yaml-cpp/yaml.h"

#include <pt_msgs/msg/line.hpp>
#include <pt_msgs/srv/load_scenario.hpp>
#include <pt_msgs/srv/set_bool.hpp>

#include <anchor.hpp>
#include <scenario.hpp>


namespace pt_potential_controller {

    using TwistMsg = geometry_msgs::msg::Twist;
    using PointMsg = geometry_msgs::msg::Point;
    using PoseMsg = geometry_msgs::msg::Pose;
    using LineMsg = pt_msgs::msg::Line;
    using BoolMsg = std_msgs::msg::Bool;
    //using StringMsg = std_msgs::msg::Bool;
    using LoadScenarioSrv = pt_msgs::srv::LoadScenario;
    using SetBoolSrv = pt_msgs::srv::SetBool;

    class PotentialControllerNode : public rclcpp::Node {
        public:
            PotentialControllerNode(rclcpp::NodeOptions options);

            // ROS parameters
            std::string scenario_path_;
            std::string control_anchor_;
            std::string rotation_target_;
            double freq_ = 10.0;
            double w_pid_p_ = 1.0;
            double w_pid_i_ = 0.1;
            double w_pid_d_ = 0.1;
            double w_pid_i_clamp_ = 1.0;
            bool vis_enabled_ = true;
            bool ctrl_enabled_ = false;

            // ROS interfaces
            std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
            rclcpp::Subscription<PoseMsg>::SharedPtr sub_control_pose_;
            rclcpp::Subscription<PointMsg>::SharedPtr sub_rotation_target_point_;
            //rclcpp::Subscription<BoolMsg>::SharedPtr sub_nav_enabled_;
            rclcpp::Publisher<TwistMsg>::SharedPtr twist_publisher_;
            // rclcpp::Publisher<BoolMsg>::SharedPtr ctrl_enabled_publisher_;
            // rclcpp::Publisher<BoolMsg>::SharedPtr scenario_path_publisher_;
            rclcpp::TimerBase::SharedPtr twist_timer_;
            rclcpp::Service<LoadScenarioSrv>::SharedPtr load_scenario_server_;
            rclcpp::Service<SetBoolSrv>::SharedPtr ctrl_enable_server_;
            OnSetParametersCallbackHandle::SharedPtr param_change_callback_;

            // other fields
            ScenarioPtr scenario_;
            tuw::Pose2D control_pose_;
            tuw::Point2D rotation_target_point_;
            bool scenario_loaded_ = false;
            double w_pid_i_state_ = 0.0;
            double w_pid_d_state_ = 0.0;

        private:
            bool anchors_updated_ = true;
            void control_loop();

            rcl_interfaces::msg::SetParametersResult reload_params(const std::vector<rclcpp::Parameter> &parameters);
            bool load_scenario(std::string path, bool init_poses);
            void load_anchors(YAML::Node anchors_map, Scenario &scenario, bool init_poses);
            void load_potentials(YAML::Node potentials_map, AnchorPtr &anchor);
            void load_vis(YAML::Node vis_map, Scenario &scenario);
    };
}

#endif