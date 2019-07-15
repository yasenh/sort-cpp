#pragma once
#include <opencv2/core.hpp>
#include "kalman_filter.h"

class Tracker {
public:
    // Constructor
    Tracker();

    void Init(const cv::Rect& bbox);
    void Predict(float dt);
    void Update(const cv::Rect& bbox);
    cv::Rect GetStateAsBbox() const;
    float GetNIS() const;

    int coast_cycles_ = 0, hit_streak_ = 0;

private:
    Eigen::VectorXd ConvertBboxToObservation(const cv::Rect& bbox) const;
    cv::Rect ConvertStateToBbox(const Eigen::VectorXd &state) const;

    KalmanFilter kf_;
};


