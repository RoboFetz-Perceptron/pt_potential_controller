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
            

            // compute total force at target point
            std::pair<double, double> total_force(tuw::Point2D target_point);

            // compute sum of forces acting on anchor
            std::pair<double, double> compute_feedback(std::string anchor_id);

            void set_vis_params(std::string win_name,
                                double scale, double width, double height,
                                double cx, double cy, double saturation);
            void draw();

        private:
            std::string vis_win_name_ = "APF Planner";
            double vis_scale_ = 0.01; // coordinate units per pixel
            double vis_width_ = 640;  // pixels
            double vis_height_ = 480;
            double vis_cx_ = 0.0;     // coordinate units
            double vis_cy_ = 0.0;
            double vis_saturation_ = 5.0;
            void draw_forces(cv::Mat &img);
    };

}

#endif