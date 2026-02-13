#ifndef PT_POTENTIAL_PLANNER__POTENTIAL_HPP
#define PT_POTENTIAL_PLANNER__POTENTIAL_HPP

#include <string>
#include <vector>
#include <cmath>

#include <tuw_geometry/tuw_geometry.hpp>
#include <opencv2/core/core.hpp>

namespace pt_potential_controller {

    class Potential;
    using PotentialPtr = std::shared_ptr<Potential>;

    // rename functions of Polar2D and hide old functions for more intuitive access
    class Force : private tuw::Polar2D {
        public:
            Force(double abs, double angle) : tuw::Polar2D(angle, abs){};
            double abs() {return rho();}
            double angle() {return alpha();}
            double x() {return cos(alpha()) * rho();}
            double y() {return sin(alpha()) * rho();}
            Force operator+(Force &f) {
                double dx = x() - f.x();
                double dy = y() - f.y();
                return Force(sqrt(dx*dx + dy*dy), atan2(dy, dx));
            }
    };


    class Potential { // this is an abstract class!
        public:
            Potential(std::string type);
            virtual ~Potential();

            const std::string type_;
            // TODO: add range-parameter to apply 0 force when distance too large?
            bool d_invert_ = true;
            double d_offset_ = 0.0;
            int f_sign_override_ = 0;
            double f_max_ = 1e9;
            double angle_twist_ = 0.0;

            Force resultant(tuw::Point2D diff);
            Force resultant(double dist, double angle);
        
        private:
            double preprocess_dist(double d);
            double postprocess_force(double d);
            virtual double force(double d) = 0;
    };

    class ConstantPotential : public Potential {
        public:
            ConstantPotential(double con);
            double con_;

            double force(double d) override;
    };

    class LinearPotential : public Potential {
        public:
            LinearPotential(double lin);
            double lin_;

            double force(double d) override;
    };

    class ExponentialPotential : public Potential {
        public:
            ExponentialPotential(double base, double exp);
            double base_;
            double exp_;

            double force(double d) override;
    };

}

#endif