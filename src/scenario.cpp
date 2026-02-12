#include <scenario.hpp>

using namespace pt_potential_controller;

Scenario::Scenario() {
    std::map<std::string, AnchorPtr> anchors;
    anchors_ = anchors;
}

Scenario::Scenario(std::vector<AnchorPtr> anchors) {
    for(std::size_t i = 0; i < anchors.size(); i++) {
        anchors_[std::to_string(i)] = anchors[i];
    }
}

Scenario::Scenario(std::map<std::string, AnchorPtr> anchors) {
    anchors_ = anchors;
}


void Scenario::set_vis_params(std::string win_name, double scale, int width, int height, double cx, double cy, double saturation) {
    vis_win_name_ = win_name;
    vis_scale_ = scale;
    vis_width_ = width;
    vis_height_ = height;
    vis_cx_ = cx;
    vis_cy_ = cy;
    vis_f_max_ = saturation;
}

void Scenario::set_vis_win_name(std::string win_name) {
    vis_win_name_ = win_name;
}

void Scenario::set_vis_win_size(size_t width, size_t height) {
    vis_width_ = width;
    vis_height_ = height;
}

void Scenario::set_vis_scale(double scale) {
    vis_scale_ = scale;
}

void Scenario::set_vis_center(double cx, double cy) {
    vis_cx_ = cx;
    vis_cy_ = cy;
}

void Scenario::set_vis_f_max(double f_max) {
    vis_f_max_ = f_max;
}

/*void Scenario::add_anchor(std::string anchor_id, Anchor anchor) {
    anchors_[anchor_id] = std::make_shared<Anchor>(anchor);
}*/

void Scenario::add_anchor(std::string anchor_id, AnchorPtr anchor) {
    anchors_[anchor_id] = anchor;
}

void Scenario::add_potential(std::string anchor_id, PotentialPtr potential) {
    anchors_[anchor_id]->add_potential(potential);
}


std::pair<double, double> Scenario::total_force(tuw::Point2D target_point) {
    double fx = 0;
    double fy = 0;
    for(auto [_, ptr] : anchors_) {
        std::pair<double, double> f = ptr->force_exerted(target_point);
        fx += f.first;
        fy += f.second;
    }
    return std::pair<double, double>(fx, fy);
}


std::pair<double, double> Scenario::compute_feedback(std::string anchor_id) {
    AnchorPtr target = anchors_[anchor_id];
    if(target->type_ != "point" && target->type_ != "pose") {
        throw std::invalid_argument("anchor feedback is only supported for PointAnchors and PoseAnchors!");
    }
    
    std::vector<AnchorPtr> others;
    for(auto [id, ptr] : anchors_) {
        if(id != anchor_id)
            others.push_back(ptr);
    }

    return anchors_[anchor_id]->force_affected(others);
}


void Scenario::draw_forces(cv::Mat &img) {
    double wy = vis_cy_ + vis_scale_*vis_height_/2 - vis_scale_/2; // world coordinates of pixels (sample at center!)
    for(size_t row = 0; row < vis_height_; row++) {
        double wx = vis_cx_ - vis_scale_*vis_width_/2  + vis_scale_/2;
        for(size_t col = 0; col < vis_width_; col++) {
            std::pair<double, double> f = total_force(tuw::Point2D(wx, wy));
            double f_res = std::sqrt(f.first*f.first + f.second*f.second); // resultant force
            f_res = std::min(f_res, vis_f_max_)/vis_f_max_; // scale to match bounds


            // blend pixel with yellow overlay, intensity depending on force at pixel
            img.at<cv::Vec3b>(row, col) = (1.0-f_res)*img.at<cv::Vec3b>(row, col) + f_res*cv::Vec3b(0, 255, 255);
            wx += vis_scale_;
        }
        wy -= vis_scale_;
    }
}


void mouse_callback(int  event, int  x, int  y, int  flag, void *param) {
    (void) flag;
    int* mouse_coords = (int*)param;
    if(event == cv::EVENT_LBUTTONDOWN) {
        mouse_coords[0] = -mouse_coords[0];
        mouse_coords[1] = -mouse_coords[1];
    }
    else if(event == cv::EVENT_MOUSEMOVE && mouse_coords[0] > 0) {
        mouse_coords[0] = x;
        mouse_coords[1] = y;
    }
}

void Scenario::draw() {
    static int mouse_coords[2] = {-1, -1};
    static cv::Mat prev_base_img(1, 1, CV_8UC1); // default size 1 pixel to recognize first call to draw()

    bool anchors_changed = false;
    for(auto [_, ptr] : anchors_) {
        anchors_changed = ptr->updated_ ? true : anchors_changed;
        ptr->updated_ = false;
    }

    // draw underlay
    cv::Mat img;
    if(anchors_changed || prev_base_img.size[0] == 1) { // need to redraw entire image
        img = cv::Mat(vis_height_, vis_width_, CV_8UC3, cv::Vec3b(255, 255, 255));
        // draw grid lines for scale
        for(size_t i = vis_height_/2; i < vis_height_; i += 1/vis_scale_)
            cv::line(img, cv::Point(0, i), cv::Point(vis_width_, i),  cv::Vec3b(128, 128, 128));
        for(size_t i = vis_height_/2; i > 0; i -= 1/vis_scale_)
            cv::line(img, cv::Point(0, i), cv::Point(vis_width_, i),  cv::Vec3b(128, 128, 128));
        for(size_t i = vis_width_/2; i < vis_width_; i += 1/vis_scale_)
            cv::line(img, cv::Point(i, 0), cv::Point(i, vis_height_), cv::Vec3b(128, 128, 128));
        for(size_t i = vis_width_/2; i > 0; i -= 1/vis_scale_)
            cv::line(img, cv::Point(i, 0), cv::Point(i, vis_height_), cv::Vec3b(128, 128, 128));

        // visualize intensity of forces
        draw_forces(img);

        // draw anchors
        for(auto [_, ptr] : anchors_) {
            ptr->draw_anchor(img, vis_scale_, vis_cx_, vis_cy_);
        }
        prev_base_img = img.clone();
    } else { // re-use force/anchor visualization from last frame
        img = prev_base_img.clone();
    }

    // interactively show force vector at mouse position
    if(mouse_coords[0] > 0) {
        // TODO: make this less messy
        double mouse_x_world = vis_cx_ - vis_scale_*vis_width_/2  + vis_scale_/2 + mouse_coords[0]*vis_scale_;
        double mouse_y_world = vis_cy_ + vis_scale_*vis_height_/2 - vis_scale_/2 - mouse_coords[1]*vis_scale_;
        std::pair<double, double> f_mouse = total_force(tuw::Point2D(mouse_x_world, mouse_y_world));
        int arrow_x = mouse_coords[0] + f_mouse.first/vis_scale_;
        int arrow_y = mouse_coords[1] - f_mouse.second/vis_scale_;
        cv::Point2l p1(mouse_coords[0], mouse_coords[1]);
        cv::Point2l p2(arrow_x, arrow_y);
        cv::arrowedLine(img, p1, p2, cv::Vec3b(0,0,255), 2, 8, 0, 0.05);
        std::cout << "force: " << f_mouse.first << ", " << f_mouse.second << std::endl;
    }

    cv::namedWindow(vis_win_name_);
    cv::setMouseCallback(vis_win_name_, mouse_callback, (void*)mouse_coords);
    cv::imshow(vis_win_name_, img);
    cv::waitKey(1);
}



