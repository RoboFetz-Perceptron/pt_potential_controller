#include "controller_node.hpp"


using namespace pt_potential_controller;

PotentialControllerNode::PotentialControllerNode(rclcpp::NodeOptions options) : Node("potential_controller_node", options) {
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
        descriptor = rcl_interfaces::msg::ParameterDescriptor{};
        descriptor.description = "...";
        scenario_path_ = this->declare_parameter<std::string>("scenario_path", "/home/arc/projects/robofetz/ws02/src/pt_potential_controller/config/example_scenario.yaml", descriptor);

        descriptor = rcl_interfaces::msg::ParameterDescriptor{};
        descriptor.description = "...";
        control_anchor_ = this->declare_parameter<std::string>("controlled_anchor", "perceptron", descriptor);

        descriptor = rcl_interfaces::msg::ParameterDescriptor{};
        descriptor.description = "...";
        rotation_target_ = this->declare_parameter<std::string>("rotation_target_anchor", "enemy", descriptor);

        descriptor = rcl_interfaces::msg::ParameterDescriptor{};
        descriptor.description = "...";
        freq_ = this->declare_parameter<double>("frequency", 10.0, descriptor);

        descriptor = rcl_interfaces::msg::ParameterDescriptor{};
        descriptor.description = "...";
        w_pid_p_ = this->declare_parameter<double>("w_pid_p", 1.0, descriptor);

        descriptor = rcl_interfaces::msg::ParameterDescriptor{};
        descriptor.description = "...";
        w_pid_i_ = this->declare_parameter<double>("w_pid_i", 0.1, descriptor);

        descriptor = rcl_interfaces::msg::ParameterDescriptor{};
        descriptor.description = "...";
        w_pid_d_ = this->declare_parameter<double>("w_pid_d", 0.1, descriptor);
    }


    twist_publisher_ = this->create_publisher<TwistMsg>("cmd_vel", 10);
    twist_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0/freq_), std::bind(&PotentialControllerNode::control_loop, this));

    load_scenario_server_ = create_service<LoadScenarioSrv>("potential_controller_node/load_scenario",
        [this](const std::shared_ptr<LoadScenarioSrv::Request> request, std::shared_ptr<LoadScenarioSrv::Response> response) {
            response->success = load_scenario(request->scenario_path);
        }
        //std::bind(&PoseTransformerNode::callback_lock, this, _1, _2)
    );

    load_scenario(scenario_path_);

    //std::thread t(control_loop);
    //vis_thread_ = std::make_unique<std::thread>(t);
    
    //scenario_->draw();
    //rclcpp::shutdown();
}


void PotentialControllerNode::control_loop() {
    std::pair<double, double> xy = scenario_->compute_feedback(control_anchor_);
    //RCLCPP_INFO(this->get_logger(), "LOOP, x: %s y: %s w: %s", );
    TwistMsg msg;
    msg.linear.x = xy.first;
    msg.linear.y = xy.second;
    // TODO: rotation command/PID
    twist_publisher_->publish(msg);
}


/*void PoseTransformerNode::callback_lock(const std::shared_ptr<LoadScenarioSrv::Request> request,
                                              std::shared_ptr<LoadScenarioSrv::Response> response) {
    response->success = load_scenario(request->scenario_path);
}*/



bool PotentialControllerNode::load_scenario(std::string path) {
    try {
        YAML::Node file_root;
        file_root = YAML::LoadFile(path);

        if(!file_root.IsMap())
            throw std::runtime_error("Root node is not a mapping containing either \"anchors\" or \"visualization\"");
        Scenario loaded_scenario = Scenario();
        BindingVec loaded_in_bindings;
        BindingVec loaded_out_bindings;
        for(std::pair<YAML::Node, YAML::Node> file_child : file_root) {
            if(file_child.first.as<std::string>() == "anchors")
                load_anchors(file_child.second, loaded_scenario);//, loaded_in_bindings, loaded_out_bindings);
            else if(file_child.first.as<std::string>() == "visualization")
                load_vis(file_child.second, loaded_scenario);
            else
                throw std::runtime_error("Root node is not a mapping containing either \"anchors\" or \"visualization\"");
        }
        RCLCPP_INFO(this->get_logger(), "Loaded scenario!");
        scenario_ = std::make_shared<Scenario>(loaded_scenario);
        in_bindings_ = std::make_shared<BindingVec>(loaded_in_bindings);
        out_bindings_ = std::make_shared<BindingVec>(loaded_out_bindings);
        return true;
    } catch(std::exception &ex) {
        RCLCPP_ERROR(this->get_logger(), "Exception while parsing YAML: %s", ex.what());
        return false;
    }
}

void PotentialControllerNode::load_anchors(YAML::Node anchors_map, Scenario &scenario) { //, BindingVec &in_bindings, BindingVec &out_bindings) {
    if(!anchors_map.IsMap())
        throw std::runtime_error("Anchor node is not a mapping");
    //std::cout << anchors_map << std::endl;

    for(std::pair<YAML::Node, YAML::Node> anchor_node : anchors_map) {
        std::string anchor_type = anchor_node.first.as<std::string>();
        YAML::Node anchor_children = anchor_node.second;
        
        // type-specific initialization
        AnchorPtr loaded;
        if(anchor_type == "point") {
            loaded = std::make_shared<PointAnchor>(PointAnchor());
            if(anchor_children["pos_x"] && anchor_children["pos_x"]) {
                tuw::Point2D initial(tuw::Point2D(anchor_children["pos_x"].as<double>(), anchor_children["pos_y"].as<double>()));
                loaded->update_point(initial);
            }
            else {
                throw std::runtime_error("Parameters missing!"); // TODO: initialize anchor as disabled instead
            }
            if(anchor_children["pos_topic"]) {
                auto sub = this->create_subscription<PointMsg>(anchor_children["pos_topic"].as<std::string>(), 10,
                    [this, loaded](const PointMsg::SharedPtr msg) {
                        loaded->update_point(tuw::Point2D(msg->x, msg->y));
                    }
                );
                //::SharedPtr
                subs_.push_back(sub);
            }
        } else if(anchor_type == "pose") {
            loaded = std::make_shared<PoseAnchor>(PoseAnchor());
            if(anchor_children["pos_x"] && anchor_children["pos_x"] && anchor_children["pos_t"]) {
                tuw::Pose2D initial(anchor_children["pos_x"].as<double>(), anchor_children["pos_y"].as<double>(), anchor_children["pos_t"].as<double>());
                loaded->update_pose(initial);  
            }
            else {
                throw std::runtime_error("Parameters missing!"); // TODO: initialize anchor as disabled instead
            }
            if(anchor_children["pos_topic"]) {
                auto sub = this->create_subscription<PoseMsg>(anchor_children["pos_topic"].as<std::string>(), 10,
                    [this, loaded](const PoseMsg::SharedPtr msg) {
                        loaded->update_pose(tuw::Pose2D(msg->position.x, msg->position.y, tuw::QuaternionToYaw(msg->orientation)));
                    }
                );
                subs_.push_back(sub);
            }
        } else if(anchor_type == "line") {
            throw std::runtime_error("Line anchors not implemented");
        } else if(anchor_type == "image") {
            throw std::runtime_error("Image anchors not implemented");
        } else {
            throw std::runtime_error("Unkown anchor type");
        }

        // add optional traits
        if(anchor_children["potentials"])
            load_potentials(anchor_children["potentials"], loaded);
        
        std::string anchor_id;
        if(anchor_children["id"])
            anchor_id = anchor_children["id"].as<std::string>();
        else
            anchor_id = std::string("auto_id_") + std::to_string((unsigned long long)loaded.get());

        scenario.add_anchor(anchor_id, loaded);
        RCLCPP_INFO(this->get_logger(), "loaded %s-anchor \"%s\"", anchor_type.c_str(), anchor_id.c_str());
    }
    
}

void PotentialControllerNode::load_potentials(YAML::Node potentials_map, AnchorPtr &anchor) {
    for(std::pair<YAML::Node, YAML::Node> potential_node :potentials_map) {
        std::string potential_type = potential_node.first.as<std::string>();
        YAML::Node potential_children = potential_node.second;
        PotentialPtr loaded;
        if(potential_type == "constant") {
            if(!potential_children["const"])
                throw std::runtime_error("Constant potential missing parameter \"const\"");
            loaded = std::make_shared<ConstantPotential>(ConstantPotential(potential_children["const"].as<double>()));
        } else if(potential_type == "linear") {
            if(!potential_children["lin"])
                throw std::runtime_error("Linear potential missing parameter \"lin\"");
            loaded = std::make_shared<LinearPotential>(LinearPotential(potential_children["lin"].as<double>()));
        } else if(potential_type == "exponential") {
            if(!(potential_children["base"] && potential_children["base"]))
                throw std::runtime_error("Exponential potential missing parameter(s) \"base\"/\"exp\"");
            loaded = std::make_shared<ExponentialPotential>(ExponentialPotential(potential_children["base"].as<double>(),
                                                                                 potential_children["exp"].as<double>()));
        } else {
            throw std::runtime_error("Unkown potential type: " + potential_type);
        }

        if(potential_children["d_invert"])
            loaded->d_invert_ = potential_children["d_invert"].as<bool>();
            
        if(potential_children["d_offset"])
            loaded->d_offset_ = potential_children["d_offset"].as<double>();

        if(potential_children["f_sign_override"])
            loaded->f_sign_override_ = potential_children["f_sign_override"].as<int>();

        if(potential_children["f_max"])
            loaded->f_max_ = potential_children["f_max"].as<double>();

        if(potential_children["rho_twist"])
            loaded->rho_twist_ = potential_children["rho_twist"].as<double>();

        anchor->add_potential(loaded);
    }
}

void PotentialControllerNode::load_vis(YAML::Node vis_map, Scenario &scenario) {
    if(vis_map["win_name"])
        scenario.set_vis_win_name(vis_map["win_name"].as<std::string>());
    if(vis_map["scale"])
        scenario.set_vis_scale(vis_map["scale"].as<double>());
    if(vis_map["width"] && vis_map["height"])
        scenario.set_vis_win_size(vis_map["width"].as<size_t>(), vis_map["height"].as<size_t>());
    if(vis_map["center_x"] && vis_map["center_y"])
        scenario.set_vis_center(vis_map["center_x"].as<double>(), vis_map["center_y"].as<double>());
    if(vis_map["saturation"])
        scenario.set_vis_f_max(vis_map["f_max"].as<double>());
}