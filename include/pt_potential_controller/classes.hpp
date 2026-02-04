#ifndef PT_POTENTIAL_PLANNER__CLASSES_HPP
#define PT_POTENTIAL_PLANNER__CLASSES_HPP

#include <string>
#include <vector>

#include <tuw_geometry/tuw_geometry.hpp>
#include <opencv2/core/core.hpp>

namespace pt_potential_controller {

    class Potential {
        public:
            std::string type_;
            int sign_override_ = 0;
            double fmax_ = __DBL_MAX__;
            double fmin_ = __DBL_MIN__;
    };

    class Linear_Potential {
        public:
            double lin_;
            double const_;
    };

    class Exponential_Potential {
        public:
            double base_;
            double exp_;
    };



    class Anchor {
        public:
            std::string id_;
            std::string type_;
            std::vector<Potential> potentials_;
    };

    class Point_Anchor : Anchor {
        public:
            tuw::Point2D p_;
    };

    class Line_Anchor : Anchor {
        public:
            tuw::Line2D l_;
    };

    class Image_Anchor : Anchor {
        public:
            cv::Mat img_;
    };

    class Agent : Point_Anchor {
        public:
            double w_;
    };

}

#endif