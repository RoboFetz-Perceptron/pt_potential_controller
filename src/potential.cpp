#include <potential.hpp>
#include <iostream>

using namespace pt_potential_controller;


Potential::Potential(std::string type) : type_(type) {}
Potential::~Potential() = default;

Force Potential::resultant(tuw::Point2D diff) {
    return resultant(diff.radius(), diff.angle());
}

Force Potential::resultant(double dist, double angle) {
    double d_prime = preprocess_dist(dist);
    double abs = postprocess_force(force(d_prime));
    angle += rho_twist_;
    return Force(abs, angle);
}

double Potential::preprocess_dist(double d) {
    double d_prime = d - d_offset_;
    if(d_invert_)
        d_prime = 1/d_prime;
    return d_prime;
}

double Potential::postprocess_force(double f) {   
    if(f_sign_override_ < 0 && f > 0)
        f = -f;
    if(f_sign_override_ > 0 && f < 0)
        f = -f;
    return std::clamp(f, -f_max_, f_max_);
}

//// ConstantPotential ////

ConstantPotential::ConstantPotential(double con) : Potential("constant") {
    con_ = con;
}

double ConstantPotential::force(double d) {
    (void) d;
    return con_;
}

//// LinearPotential ////

LinearPotential::LinearPotential(double lin) : Potential("linear") {
    lin_ = lin;
}

double LinearPotential::force(double d) {
    return lin_ * d;
}

//// ExponentialPotential ////

ExponentialPotential::ExponentialPotential(double base, double exp) : Potential("exponential") {
    base_ = base;
    exp_ = exp;
}

double ExponentialPotential::force(double d) {
    return pow(base_, (exp_ * d));
}