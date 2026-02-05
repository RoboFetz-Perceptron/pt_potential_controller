#ifndef PT_POTENTIAL_PLANNER__IN_INTERFACE_HPP
#define PT_POTENTIAL_PLANNER__IN_INTERFACE_HPP

#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

#include <tuw_geometry/tuw_geometry.hpp>

// NOT NEEDED?

namespace pt_potential_controller {

    class InInterface {
        public:
            std::string type_;
    };

    class Point_InInterface : InInterface {
        public:
            tuw::Point2D p_;
    };

    class Line_InInterface : InInterface {
        public:
            tuw::Line2D l_;
    };

    class Image_InInterface : InInterface {
        public:
            cv::Mat img_;
    };


    using Point_InInterface_Ptr = std::shared_ptr<Point_InInterface>;
    using Line_InInterface_Ptr = std::shared_ptr<Line_InInterface>;
    using Image_InInterface_Ptr = std::shared_ptr<Image_InInterface>;
}

#endif