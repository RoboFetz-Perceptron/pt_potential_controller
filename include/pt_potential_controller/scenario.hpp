#ifndef PT_POTENTIAL_PLANNER__SCENARIO_HPP
#define PT_POTENTIAL_PLANNER__SCENARIO_HPP

#include <algorithm>

#include "opencv2/highgui.hpp"

#include <anchor.hpp>
#include <potential.hpp>

namespace pt_potential_controller {

    class Scenario {
        public:
            Scenario();
            Scenario(std::vector<AnchorPtr> anchors);
            Scenario(std::map<std::string, AnchorPtr> anchors);

            std::map<std::string, AnchorPtr> anchors_;
            double vis_scale_ = 0.01;
            double vis_width_ = 640;
            double vis_height_ = 480;
            double vis_cx_ = 0.0;
            double vis_cy_ = 0.0;
            double vis_saturation_ = 5.0;

            // compute total force at target point
            std::pair<double, double> total_force(tuw::Point2D target_point);

            // compute feedback resulting from force acting on anchor
            std::pair<double, double> compute_feedback(std::string anchor_id);

            void draw_forces(cv::Mat &img);
            void draw(std::string win_name);
    };

}

#endif