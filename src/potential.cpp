#include <potential.hpp>

using namespace pt_potential_controller;


Potential::Potential(std::string type) : type_(type) {}
Potential::~Potential() = default;


//// LinearPotential ////

LinearPotential::LinearPotential(double lin, double con) : Potential("linear") {
    lin_ = lin;
    con_ = con;
}

std::pair<double, double> LinearPotential::force(double dx, double dy) {
    double fx = lin_ * dx + con_;
    double fy = lin_ * dy + con_;
    return std::pair<double, double>(fx, fy);
}

//// ExponentialPotential ////

ExponentialPotential::ExponentialPotential(double base, double exp) : Potential("exponential") {
    base_ = base;
    exp_ = exp;
}

std::pair<double, double> ExponentialPotential::force(double dx, double dy) {
    double fx = pow(base_, (exp_ * dx));
    double fy = pow(base_, (exp_ * dy));
    return std::pair<double, double>(fx, fy);
}