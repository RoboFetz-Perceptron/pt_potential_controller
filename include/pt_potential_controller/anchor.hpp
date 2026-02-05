#ifndef PT_POTENTIAL_PLANNER__ANCHOR_HPP
#define PT_POTENTIAL_PLANNER__ANCHOR_HPP

#include <string>
#include <vector>
#include <iostream>

#include <opencv2/core/core.hpp>

#include <tuw_geometry/tuw_geometry.hpp>

#include <potential.hpp>


namespace pt_potential_controller {

    class Anchor;
    using AnchorPtr = std::shared_ptr<Anchor>;

    class Anchor { // this is an abstract class!
        public:
            Anchor(std::string type);
            virtual ~Anchor();

            const std::string type_;
            std::vector<std::shared_ptr<Potential>> potentials_;

            // compute force this anchor exerts onto target point
            virtual std::pair<double, double> force_exerted(tuw::Point2D p) = 0;

            // compute force exerted onto this anchor by a of other anchors
            virtual std::pair<double, double> force_affected(std::vector<AnchorPtr> &anchors) = 0;

            // this would be ideal, but templated functions cant be declared virtual/be overloaded
            //template<typename ... Ts>
            //virtual void update_position(Ts ... args);

            // workaround: user has to check type and call correct update function
            virtual void update_point(tuw::Point2D p) {(void) p;};
            virtual void update_pose(tuw::Pose2D p) {(void) p;};
            virtual void update_line(tuw::Line2D l) {(void) l;};
            virtual void update_image(cv::Mat img) {(void) img;};
    };

    class PointAnchor : public Anchor {
        public:
            PointAnchor();
            PointAnchor(tuw::Point2D p);
            PointAnchor(double x, double y);
            ~PointAnchor();
            tuw::Point2D p_ = tuw::Point2D();

            std::pair<double, double> force_exerted(tuw::Point2D p) override;
            std::pair<double, double> force_affected(std::vector<AnchorPtr> &anchors) override;
            void update_point(tuw::Point2D p) override;
    };

    class PoseAnchor : public Anchor {
        public:
            PoseAnchor();
            PoseAnchor(tuw::Pose2D p);
            PoseAnchor(double x, double y, double t);
            ~PoseAnchor();
            tuw::Pose2D p_ = tuw::Pose2D();

            std::pair<double, double> force_exerted(tuw::Point2D p) override;
            std::pair<double, double> force_affected(std::vector<AnchorPtr> &anchors) override;
            void update_pose(tuw::Pose2D p) override;
    };

    class LineAnchor : public Anchor {
        public:
            tuw::Line2D l_ = tuw::Line2D();

            std::pair<double, double> force_exerted(tuw::Point2D p) override;
            void update_line(tuw::Line2D l) override;
    };

    class ImageAnchor : public Anchor {
        public:
            cv::Mat img_ = cv::Mat();

            std::pair<double, double> force_exerted(tuw::Point2D p) override;
            void update_image(cv::Mat img) override;
    };

}

#endif