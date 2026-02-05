#include <anchor.hpp>

using namespace pt_potential_controller;


Anchor::Anchor(std::string type) : type_(type) {}
Anchor::~Anchor() = default;


//// PointAnchor ////

PointAnchor::PointAnchor() : Anchor("point") {
    p_ = tuw::Point2D();
}

PointAnchor::PointAnchor(double x, double y) : Anchor("point") {
    p_ = tuw::Point2D(x, y);
}

PointAnchor::PointAnchor(tuw::Point2D p) : Anchor("point") {
    p_ = p;
}

PointAnchor::~PointAnchor() = default;

void PointAnchor::update_point(tuw::Point2D p) {
    p_ = p;
}

std::pair<double, double> PointAnchor::force_exerted(tuw::Point2D target) {
    double dx = target.x() - p_.x();
    double dy = target.y() - p_.y();
    double fx = 0;
    double fy = 0;
    for(auto pot : potentials_) {
        std::pair<double, double> f = pot->force(dx, dy);
        fx += f.first;
        fy += f.second;
    }
    return std::pair<double, double>(fx, fy);
}

std::pair<double, double> PointAnchor::force_affected(std::vector<AnchorPtr> &anchors) {
    double fx = 0;
    double fy = 0;
    for(auto anchor : anchors) {
        std::pair<double, double> f = anchor->force_exerted(p_);
        fx += f.first;
        fy += f.second;
    }
}


//// PoseAnchor ////

PoseAnchor::PoseAnchor() : Anchor("pose") {
    p_ = tuw::Pose2D(0.0, 0.0, 0.0);
}

PoseAnchor::PoseAnchor(double x, double y, double t) : Anchor("pose") {
    p_ = tuw::Pose2D(x, y, t);
}

PoseAnchor::PoseAnchor(tuw::Pose2D p) : Anchor("pose") {
    p_ = p;
}

PoseAnchor::~PoseAnchor() = default;

void PoseAnchor::update_pose(tuw::Pose2D p) {
    p_ = p;
}

/*void PoseAnchor::update_pose(double x, double y, double t) {
    p_.set(x, y, t);
}*/

std::pair<double, double> PoseAnchor::force_exerted(tuw::Point2D target) {
    double dx = target.x() - p_.x();
    double dy = target.y() - p_.y();
    double fx = 0;
    double fy = 0;
    for(auto pot : potentials_) {
        std::pair<double, double> f = pot->force(dx, dy);
        fx += f.first;
        fy += f.second;
    }
    return std::pair<double, double>(fx, fy);
}


// TODO: output fx, fy in relation to pose orientation instead of world coordinates
std::pair<double, double> PoseAnchor::force_affected(std::vector<AnchorPtr> &anchors) {
    double fx = 0;
    double fy = 0;
    for(auto anchor : anchors) {
        std::pair<double, double> f = anchor->force_exerted(tuw::Point2D(p_.position()));
        fx += f.first;
        fy += f.second;
    }
}