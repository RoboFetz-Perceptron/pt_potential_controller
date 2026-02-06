#include <anchor.hpp>

using namespace pt_potential_controller;


Anchor::Anchor(std::string type) : type_(type) {}
Anchor::~Anchor() = default;


//// PointAnchor ////

PointAnchor::PointAnchor() : Anchor("point") {
    p_ = tuw::Point2D();
    potentials_ = {};
}

PointAnchor::PointAnchor(double x, double y, std::vector<PotentialPtr> potentials = {}) : Anchor("point") {
    p_ = tuw::Point2D(x, y);
    potentials_ = potentials;
}

PointAnchor::PointAnchor(tuw::Point2D p, std::vector<PotentialPtr> potentials = {}) : Anchor("point") {
    p_ = p;
    potentials_ = potentials;
}

PointAnchor::~PointAnchor() = default;

void PointAnchor::update_point(tuw::Point2D p) {
    p_ = p;
}

std::pair<double, double> PointAnchor::force_exerted(tuw::Point2D target) {
    //std::cout << "point force_exerted" << std::endl;
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
    return std::pair<double, double>(fx, fy);
}

void PointAnchor::draw_anchor(cv::Mat &img, double scale, double cx, double cy) {
    int img_x = (p_.x() + cx)*scale + img.cols/2;
    int img_y = (p_.y() + cy)*scale + img.rows/2;
    if(img_x < 0 || img_x > img.cols || img_y < 0 || img_y > img.rows)
        return; // point is out of bounds
    cv::Point2l p_cv(img_x, img_y);
    cv::circle(img, p_cv, 6, cv::Vec3b(255,0,0), -1, 0, 0);
    cv::circle(img, p_cv, 6, cv::Vec3b(0,0,0), 1, 0, 0); // outline
}


//// PoseAnchor ////

PoseAnchor::PoseAnchor() : Anchor("pose") {
    p_ = tuw::Pose2D(0.0, 0.0, 0.0);
    potentials_ = {};
}

PoseAnchor::PoseAnchor(double x, double y, double t, std::vector<PotentialPtr> potentials = {}) : Anchor("pose") {
    p_ = tuw::Pose2D(x, y, t);
    potentials_ = potentials;
}

PoseAnchor::PoseAnchor(tuw::Pose2D p, std::vector<PotentialPtr> potentials = {}) : Anchor("pose") {
    p_ = p;
    potentials_ = potentials;
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
    return std::pair<double, double>(fx, fy);
}

void PoseAnchor::draw_anchor(cv::Mat &img, double scale, double cx, double cy) {
    int img_x = (p_.position().x() + cx)*scale + img.cols/2;
    int img_y = (p_.position().y() + cy)*scale + img.rows/2;
    if(img_x < 0 || img_x > img.cols || img_y < 0 || img_y > img.rows)
        return; // point is out of bounds
    cv::Point2l p_cv(img_x, img_y);
    cv::circle(img, p_cv, 6, cv::Vec3b(0,255,0), -1, 0, 0);
    cv::circle(img, p_cv, 6, cv::Vec3b(0,0,0), 1, 0, 0); // outline

    cv::Point2l p1(img_x, img_y);
    int line_end_x = img_x + 6*std::cos(p_.theta());
    int line_end_y = img_y + 6*std::sin(p_.theta());
    line_end_x = std::max(0, std::min(line_end_x, img.cols));
    line_end_y = std::max(0, std::min(line_end_y, img.rows));
    cv::Point2l p2(line_end_x, line_end_y);
    cv::line(img, p1, p2, cv::Vec3b(0,0,0));
}