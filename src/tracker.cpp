#include "tracker.h"

Tracker::Tracker() : kf_(8, 4) {
    frame_count_ = 0;

    /*** Define constant velocity model ***/
    // state - center_x, center_y, width, height, v_cx, v_cy, v_width, v_height
    kf_.F_ <<
           1, 0, 0, 0, 1, 0, 0, 0,
           0, 1, 0, 0, 0, 1, 0, 0,
           0, 0, 1, 0, 0, 0, 1, 0,
           0, 0, 0, 1, 0, 0, 0, 1,
           0, 0, 0, 0, 1, 0, 0, 0,
           0, 0, 0, 0, 0, 1, 0, 0,
           0, 0, 0, 0, 0, 0, 1, 0,
           0, 0, 0, 0, 0, 0, 0, 1;

    kf_.P_ <<
           10, 0, 0, 0, 0, 0, 0, 0,
           0, 10, 0, 0, 0, 0, 0, 0,
           0, 0, 10, 0, 0, 0, 0, 0,
           0, 0, 0, 10, 0, 0, 0, 0,
           0, 0, 0, 0, 10000, 0, 0, 0,
           0, 0, 0, 0, 0, 10000, 0, 0,
           0, 0, 0, 0, 0, 0, 10000, 0,
           0, 0, 0, 0, 0, 0, 0, 10000;

    kf_.Q_ <<
           1, 0, 0, 0, 0, 0, 0, 0,
           0, 1, 0, 0, 0, 0, 0, 0,
           0, 0, 1, 0, 0, 0, 0, 0,
           0, 0, 0, 1, 0, 0, 0, 0,
           0, 0, 0, 0, 0.01, 0, 0, 0,
           0, 0, 0, 0, 0, 0.01, 0, 0,
           0, 0, 0, 0, 0, 0, 0.01, 0,
           0, 0, 0, 0, 0, 0, 0, 0.01;

    kf_.H_ <<
           1, 0, 0, 0, 0, 0, 0, 0,
           0, 1, 0, 0, 0, 0, 0, 0,
           0, 0, 1, 0, 0, 0, 0, 0,
           0, 0, 0, 1, 0, 0, 0, 0;

    kf_.R_ <<
            1, 0, 0,  0,
            0, 1, 0,  0,
            0, 0, 10, 0,
            0, 0, 0,  10;


}


// Get predicted locations from existing trackers
// dt is time elapsed between the current and previous measurements
void Tracker::Predict(float dt) {
    kf_.F_(0, 4) = dt;
    kf_.F_(1, 5) = dt;
    kf_.F_(2, 6) = dt;
    kf_.F_(3, 7) = dt;

    // TODO: update Q based on deltaT
    kf_.Predict();

    // hit streak count will be reset
    if (coast_cycles_ > 0) {
        hit_streak_ = 0;
    }
    // accumulate coast cycle count
    coast_cycles_ += 1;
}


// Update matched trackers with assigned detections
void Tracker::Update(const cv::Rect& bbox) {
    frame_count_ += 1;

    // get measurement update, reset coast cycle count
    coast_cycles_ = 0;
    // accumulate hit streak count
    hit_streak_ += 1;

    // observation - center_x, center_y, area, ratio
    Eigen::VectorXd observation = ConvertBboxToObservation(bbox);
    kf_.Update(observation);


}


// Create and initialize new trackers for unmatched detections, with initial bounding box
void Tracker::Init(const cv::Rect &bbox) {
    kf_.x_.head(4) << ConvertBboxToObservation(bbox);
}


/**
 * Returns the current bounding box estimate
 * @return
 */
cv::Rect Tracker::GetStateAsBbox() const {
    return ConvertStateToBbox(kf_.x_);
}


double Tracker::GetNIS() const {
    return kf_.NIS_;
}


/**
 * Takes a bounding box in the form [x, y, width, height] and returns z in the form
 * [x, y, s, r] where x,y is the centre of the box and s is the scale/area and r is
 * the aspect ratio
 *
 * @param bbox
 * @return
 */
Eigen::VectorXd Tracker::ConvertBboxToObservation(const cv::Rect& bbox) const{
    Eigen::VectorXd observation = Eigen::VectorXd::Zero(4);
    auto width = static_cast<float>(bbox.width);
    auto height = static_cast<float>(bbox.height);
    float center_x = bbox.x + width / 2;
    float center_y = bbox.y + height / 2;
    observation << center_x, center_y, width, height;
    return observation;
}


/**
 * Takes a bounding box in the centre form [x,y,s,r] and returns it in the form
 * [x1,y1,x2,y2] where x1,y1 is the top left and x2,y2 is the bottom right
 *
 * @param state
 * @return
 */
 // TODO: add score as additional output
cv::Rect Tracker::ConvertStateToBbox(const Eigen::VectorXd &state) const {
    // state - center_x, center_y, width, height, v_cx, v_cy, v_width, v_height
    auto width = static_cast<int>(state[2]);
    auto height = static_cast<int>(state[3]);
    auto tl_x = static_cast<int>(state[0] - width / 2.0);
    auto tl_y = static_cast<int>(state[1] - height / 2.0);
    cv::Rect rect(cv::Point(tl_x, tl_y), cv::Size(width, height));
    return rect;
}
