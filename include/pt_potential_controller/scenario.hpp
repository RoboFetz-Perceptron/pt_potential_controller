#ifndef PT_POTENTIAL_PLANNER__SCENARIO_HPP
#define PT_POTENTIAL_PLANNER__SCENARIO_HPP

#include <algorithm>

#include "opencv2/highgui.hpp"

#include <anchor.hpp>
#include <potential.hpp>


namespace pt_potential_controller {

    class Scenario;
    using ScenarioPtr = std::shared_ptr<Scenario>;

    class Scenario {
        public:
            Scenario();
            Scenario(std::vector<AnchorPtr> anchors);
            Scenario(std::map<std::string, AnchorPtr> anchors);

            void add_anchor(std::string anchor_id, AnchorPtr anchor);
            void add_potential(std::string anchor_id, PotentialPtr Potential);

            // compute total force at target point
            Force total_force(tuw::Point2D target_point);

            // compute sum of forces acting on anchor
            Force compute_feedback(std::string anchor_id);

            // drawing functions
            void set_vis_params(std::string win_name,
                                double scale, int width, int height,
                                double cx, double cy, double f_max);
            void set_vis_win_name(std::string win_name);
            void set_vis_win_size(size_t width, size_t height);
            void set_vis_scale(double scale);
            void set_vis_center(double cx, double cy);
            void set_vis_f_max(double f_max);
            void draw();

        private:
            std::map<std::string, AnchorPtr> anchors_;
            std::string vis_win_name_ = "APF Planner";
            double vis_scale_ = 0.01; // coordinate units per pixel
            size_t vis_width_ = 640;  // pixels
            size_t vis_height_ = 480;
            double vis_cx_ = 0.0;     // coordinate units
            double vis_cy_ = 0.0;
            double vis_f_max_ = 5.0;
            void draw_forces(cv::Mat &img);
    };

}

#endif