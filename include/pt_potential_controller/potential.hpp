#ifndef PT_POTENTIAL_PLANNER__POTENTIAL_HPP
#define PT_POTENTIAL_PLANNER__POTENTIAL_HPP

#include <string>
#include <vector>
#include <cmath>
//#include <numeric>

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
            // TODO: add range-parameter to apply 0 force when distance too large?
            bool d_invert_ = true;
            double d_offset_ = 0.0;
            int f_sign_override_ = 0;
            double f_min_ = std::numeric_limits<double>::lowest();
            double f_max_ = std::numeric_limits<double>::max();
            double angle_twist_ = 0.0;

            // TODO: create resultant()-function that calls all these and returns Force object?
            //Force resultant(double d, double rho);
            double preprocess_dist(double d);
            double postprocess_force(double d);
            double twist_angle(double t);
            virtual double force(double d) = 0;
    };

    class LinearPotential : public Potential {
        public:
            LinearPotential(double lin, double con);
            double lin_;
            double con_;

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