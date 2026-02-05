#include <anchor.hpp>
#include <potential.hpp>

namespace pt_potential_controller {

    class Scenario {
        public:
            Scenario();
            Scenario(std::vector<AnchorPtr> anchors);
            Scenario(double dt, std::map<std::string, AnchorPtr> anchors);
            double dt_;
            std::map<std::string, AnchorPtr> anchors_;

            // compute total force at target point
            std::pair<double, double> total_force(tuw::Point2D target_point);

            // compute feedback resulting from force acting on anchor
            std::pair<double, double> compute_feedback(std::string anchor_id);

    };

}