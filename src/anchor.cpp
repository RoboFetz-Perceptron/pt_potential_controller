#include <anchor.hpp>

using namespace pt_potential_controller;


Anchor::Anchor(std::string type) : type_(type) {}
Anchor::~Anchor() = default;

void Anchor::add_potential(PotentialPtr potential) {
    potentials_.push_back(potential);
}

Force Anchor::force_exerted(tuw::Point2D target) {
    if(!enabled_) return Force(0.0, 0.0);
    tuw::Point2D diff = target - acting_point(target);
    Force f_total = Force(0.0, 0.0);
    for(auto pot : potentials_) {
        Force f = pot->resultant(diff);
        f_total = f_total + f;
    }
    return f_total;
}

//// PointAnchor ////

PointAnchor::PointAnchor() : Anchor("point") {
    p_ = tuw::Point2D();
    potentials_ = {};
}

PointAnchor::PointAnchor(tuw::Point2D p) : Anchor("point") {
    p_ = p;
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

tuw::Point2D PointAnchor::acting_point(tuw::Point2D p) {
    (void) p;
    return p_;
}

Force PointAnchor::force_affected(std::vector<AnchorPtr> &anchors) {
    Force f_total = Force(0.0, 0.0);
    for(auto anchor : anchors) {
        Force f = anchor->force_exerted(p_);
        f_total = f_total + f;
    }
    return f_total;
}

void PointAnchor::draw_anchor(cv::Mat &img, double scale, double cx, double cy) {
    if(!enabled_) return;
    int img_x = (cx + p_.x())/scale + img.cols/2;
    int img_y = (cy - p_.y())/scale + img.rows/2;
    cv::Point2l p_cv(img_x, img_y);
    cv::circle(img, p_cv, 6, cv::Vec3b(255,0,0), -1, 0, 0);
    cv::circle(img, p_cv, 6, cv::Vec3b(0,0,0), 1, 0, 0); // outline
}


//// PoseAnchor ////

PoseAnchor::PoseAnchor() : Anchor("pose") {
    p_ = tuw::Pose2D(0.0, 0.0, 0.0);
    potentials_ = {};
}

PoseAnchor::PoseAnchor(tuw::Pose2D p) : Anchor("pose") {
    p_ = p;
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

tuw::Point2D PoseAnchor::acting_point(tuw::Point2D p) {
    (void) p;
    return p_.position();
}

Force PoseAnchor::force_affected(std::vector<AnchorPtr> &anchors) {
    Force f_total = Force(0.0, 0.0);
    for(auto anchor : anchors) {
        Force f = anchor->force_exerted(p_.position());
        f_total = f_total + f;
    }
    return Force(f_total.abs(), f_total.angle()-p_.get_theta());
}

void PoseAnchor::draw_anchor(cv::Mat &img, double scale, double cx, double cy) {
    if(!enabled_) return;
    int img_x = (cx + p_.position().x())/scale + img.cols/2;
    int img_y = (cy - p_.position().y())/scale + img.rows/2;
    cv::Point2l p_cv(img_x, img_y);
    cv::circle(img, p_cv, 6, cv::Vec3b(0,255,0), -1, 0, 0);
    cv::circle(img, p_cv, 6, cv::Vec3b(0,0,0), 1, 0, 0); // outline

    cv::Point2l p1(img_x, img_y);
    int line_end_x = img_x + 6*std::cos(p_.theta());
    int line_end_y = img_y - 6*std::sin(p_.theta());
    cv::Point2l p2(line_end_x, line_end_y);
    cv::line(img, p1, p2, cv::Vec3b(0,0,0));
}


//// LineAnchor ////

LineAnchor::LineAnchor() : Anchor("line") {
    l_ = tuw::LineSegment2D(0.0, 0.0, 0.0, 0.0);
    potentials_ = {};
}

LineAnchor::LineAnchor(tuw::LineSegment2D l) : Anchor("line") {
    l_ = l;
    potentials_ = {};
}

LineAnchor::LineAnchor(tuw::Point2D p0, tuw::Point2D p1) : Anchor("line") {
    l_ = tuw::LineSegment2D(p0, p1);
    potentials_ = {};
}

LineAnchor::~LineAnchor() = default;

void LineAnchor::update_line(tuw::LineSegment2D l) {
    l_ = l;
}

tuw::Point2D LineAnchor::acting_point(tuw::Point2D p) {
    return l_.closestPointTo(p);
}

Force LineAnchor::force_affected(std::vector<AnchorPtr> &anchors) {
    (void) anchors;
    throw std::invalid_argument("Force acting on LineAnchor is not implemented!");
}

void LineAnchor::draw_anchor(cv::Mat &img, double scale, double cx, double cy) {
    if(!enabled_) return;
    int img_x0 = (cx + l_.x0())/scale + img.cols/2;
    int img_y0 = (cy - l_.y0())/scale + img.rows/2;
    int img_x1 = (cx + l_.x1())/scale + img.cols/2;
    int img_y1 = (cy - l_.y1())/scale + img.rows/2;
    if(img_x0 < 0 && img_y0 < 0 && img_x1 < 0 && img_y1 < 0) return;
    if(img_x0 >= img.cols && img_y0 >= img.rows && img_x1 >= img.cols && img_y1 >= img.rows) return;
    cv::Point2l p0_cv(img_x0, img_y0);
    cv::Point2l p1_cv(img_x1, img_y1);
    cv::line(img, p0_cv, p1_cv, cv::Vec3b(0,0,0), 2);
}


//// ImageAnchor ////

Force ImageAnchor::force_affected(std::vector<AnchorPtr> &anchors) {
    (void) anchors;
    throw std::invalid_argument("Force acting on ImageAnchor is not implemented!");
}