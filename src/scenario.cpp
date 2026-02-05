#include <scenario.hpp>

using namespace pt_potential_controller;

Scenario::Scenario() {
    dt_ = 1.0;
    std::map<std::string, AnchorPtr> anchors;
    anchors_ = anchors;
}

Scenario::Scenario(std::vector<AnchorPtr> anchors) {
    dt_ = 1.0;
    for(std::size_t i; i < anchors.size(); i++) {
        anchors_[std::to_string(i)] = anchors[i];
    }
}

Scenario::Scenario(double dt, std::map<std::string, AnchorPtr> anchors) {
    dt_ = dt;
    anchors_ = anchors;
}


std::pair<double, double> Scenario::total_force(tuw::Point2D target_point) {
    double fx = 0;
    double fy = 0;
    for(auto [_, ptr] : anchors_) {
        std::pair<double, double> f = ptr->force_exerted(target_point);
        fx += f.first;
        fy += f.second;
    }
    return std::pair<double, double>(fx, fy);
}

std::pair<double, double> Scenario::compute_feedback(std::string anchor_id) {
    AnchorPtr target = anchors_[anchor_id];
    if(target->type_ != "PointAnchor" || target->type_ != "PoseAnchor") {
        std::cout << "anchor feedback is only supported for PointAnchors and PoseAnchors!" << std::endl;
    }
    
    std::vector<AnchorPtr> others;
    for(auto [id, ptr] : anchors_) {
        if(id != anchor_id)
            others.push_back(ptr);
    }

    return anchors_[anchor_id]->force_affected(others);
}