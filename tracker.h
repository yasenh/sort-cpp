//
// Created by yasen on 4/22/19.
//

#pragma once
#include <opencv2/core.hpp>
#include "kalman_filter.h"

class Tracker {
public:
    explicit Tracker(unsigned int max_age = 1, unsigned int min_hits = 3);
    void Predict(float dt);
    void Update(const cv::Rect& bbox);


private:
    Eigen::VectorXd ConvertBboxToObservation(const cv::Rect& bbox);

    unsigned int max_age_;
    unsigned int min_hits_;
    unsigned int frame_count_;
    KalmanFilter kf_;
};


