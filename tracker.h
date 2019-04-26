//
// Created by yasen on 4/22/19.
//

#pragma once
#include <opencv2/core.hpp>
#include "kalman_filter.h"

class Tracker {
public:
    explicit Tracker(unsigned int max_age = 5, unsigned int min_hits = 3);
    void Init(const cv::Rect& bbox);
    void Predict(float dt);
    void Update(const cv::Rect& bbox);
    cv::Rect GetStateBbox() const;

    unsigned int max_age_;
    unsigned int min_hits_;
    unsigned int frame_count_;

    unsigned int age_ = 0, hits_ = 0, hit_streak_ = 0;


private:
    Eigen::VectorXd ConvertBboxToObservation(const cv::Rect& bbox) const;
    cv::Rect ConvertStateToBbox(const Eigen::VectorXd &state) const;

    KalmanFilter kf_;
};


