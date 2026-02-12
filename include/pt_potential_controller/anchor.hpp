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
            Anchor(std::string type); // needed to set value of type
            virtual ~Anchor();

            const std::string type_;
            std::vector<PotentialPtr> potentials_;
            bool enabled_ = true;
            bool updated_ = true; // set if anchor has been updated since last "relevant" computation

            // this would be ideal, but templated functions cant be declared virtual/be overloaded
            //template<typename ... Ts>
            //virtual void update_position(Ts ... args);

            // workaround: user has to check type and call correct update function :/
            virtual void update_point(tuw::Point2D p) {(void) p;};
            virtual void update_pose(tuw::Pose2D p) {(void) p;};
            virtual void update_line(tuw::Line2D l) {(void) l;};
            virtual void update_image(cv::Mat img) {(void) img;};

            // add a new potential to this anchor
            void add_potential(PotentialPtr potential);

            // compute force this anchor exerts onto target point
            virtual Force force_exerted(tuw::Point2D p) = 0;

            // compute force exerted onto this anchor by other anchors
            virtual Force force_affected(std::vector<AnchorPtr> &anchors) = 0;

            // visualize this anchor in the given image
            virtual void draw_anchor(cv::Mat &img, double scale, double cx, double cy) = 0;
    };


    class PointAnchor : public Anchor {
        public:
            PointAnchor();
            PointAnchor(tuw::Point2D p);
            PointAnchor(tuw::Point2D p, std::vector<PotentialPtr> potentials);
            PointAnchor(double x, double y, std::vector<PotentialPtr> potentials);
            ~PointAnchor();

            tuw::Point2D p_ = tuw::Point2D();

            void update_point(tuw::Point2D p) override;
            Force force_exerted(tuw::Point2D p) override;
            Force force_affected(std::vector<AnchorPtr> &anchors) override;
            void draw_anchor(cv::Mat &img, double scale, double cx, double cy) override;
            
    };

    class PoseAnchor : public Anchor {
        public:
            PoseAnchor();
            PoseAnchor(tuw::Pose2D p);
            PoseAnchor(tuw::Pose2D p, std::vector<PotentialPtr> potentials);
            PoseAnchor(double x, double y, double t, std::vector<PotentialPtr> potentials);
            ~PoseAnchor();

            tuw::Pose2D p_ = tuw::Pose2D();

            void update_pose(tuw::Pose2D p) override;
            Force force_exerted(tuw::Point2D p) override;
            Force force_affected(std::vector<AnchorPtr> &anchors) override;
            void draw_anchor(cv::Mat &img, double scale, double cx, double cy) override;
    };

    // not implemented:
    class LineAnchor : public Anchor {
        public:
            tuw::Line2D l_ = tuw::Line2D();

            void update_line(tuw::Line2D l) override;
            Force force_exerted(tuw::Point2D p) override;
    };

    class ImageAnchor : public Anchor {
        public:
            cv::Mat img_ = cv::Mat();

            void update_image(cv::Mat img) override;
            Force force_exerted(tuw::Point2D p) override;
    };

}

#endif