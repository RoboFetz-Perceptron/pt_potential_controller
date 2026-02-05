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

    class Potential { // this is an abstract class!
        public:
            Potential(std::string type);
            virtual ~Potential();

            const std::string type_;
            int sign_override_ = 0;
            double fmax_ = __DBL_MAX__;
            double fmin_ = __DBL_MIN__;

            virtual std::pair<double, double> force(double dx, double dy) = 0;
    };

    class LinearPotential : public Potential {
        public:
            LinearPotential(double lin, double con);
            double lin_;
            double con_;

            std::pair<double, double> force(double dx, double dy) override;
    };

    class ExponentialPotential : public Potential {
        public:
            ExponentialPotential(double base, double exp);
            double base_;
            double exp_;

            std::pair<double, double> force(double dx, double dy) override;
    };

}

#endif