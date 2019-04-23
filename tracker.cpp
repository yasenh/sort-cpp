//
// Created by yasen on 4/22/19.
//

#include "tracker.h"

Tracker::Tracker(unsigned int max_age, unsigned int min_hits) : max_age_(max_age), min_hits_(min_hits), kf_(7,4) {
    frame_count_ = 0;

    /*** Define constant velocity model ***/
    // state - center_x, center_y, area, ratio, v_cx, v_cy, v_area
    kf_.F_ <<
           1, 0, 0, 0, 1, 0, 0,
           0, 1, 0, 0, 0, 1, 0,
           0, 0, 1, 0, 0, 0, 1,
           0, 0, 0, 1, 0, 0, 0,
           0, 0, 0, 0, 1, 0, 0,
           0, 0, 0, 0, 0, 1, 0,
           0, 0, 0, 0, 0, 0, 1;

    // TODO: tune P
    kf_.P_ <<
           10, 0, 0, 0, 0, 0, 0,
           0, 10, 0, 0, 0, 0, 0,
           0, 0, 10, 0, 0, 0, 0,
           0, 0, 0, 10, 0, 0, 0,
           0, 0, 0, 0, 1000, 0, 0,
           0, 0, 0, 0, 0, 1000, 0,
           0, 0, 0, 0, 0, 0, 10;

    // TODO: tune Q
    kf_.Q_ <<
           1, 0, 0, 0, 1, 0, 0,
           0, 1, 0, 0, 0, 1, 0,
           0, 0, 1, 0, 0, 0, 1,
           0, 0, 0, 1, 0, 0, 0,
           0, 0, 0, 0, 1, 0, 0,
           0, 0, 0, 0, 0, 1, 0,
           0, 0, 0, 0, 0, 0, 1;

    kf_.H_ <<
           1, 0, 0, 0, 0, 0, 0,
           0, 1, 0, 0, 0, 0, 0,
           0, 0, 1, 0, 0, 0, 0,
           0, 0, 0, 1, 0, 0, 0;

    kf_.R_ <<
           1, 0, 0,  0,
           0, 1, 0,  0,
           0, 0, 10, 0,
           0, 0, 0,  10;
}


// time elapsed between the current and previous measurements
void Tracker::Predict(float dt) {
    kf_.F_(0, 4) = dt;
    kf_.F_(1, 5) = dt;
    kf_.F_(2, 6) = dt;

    // TODO: update Q
    kf_.Predict();
}


void Tracker::Update(const cv::Rect& bbox) {
    frame_count_ += 1;

    // observation - center_x, center_y, area, ratio
    Eigen::VectorXd observation = ConvertBboxToObservation(bbox);
    kf_.Update(observation);
}


Eigen::VectorXd Tracker::ConvertBboxToObservation(const cv::Rect& bbox) {
    Eigen::VectorXd observation;
    auto width = static_cast<float>(bbox.width);
    auto height = static_cast<float>(bbox.height);
    float center_x = bbox.x + width / 2;
    float center_y = bbox.y + height / 2;
    float area = width * height;
    float ratio = width / height;
    observation << center_x, center_y, area, ratio;
    return observation;
}