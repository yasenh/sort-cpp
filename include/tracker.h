#pragma once

#include <map>
#include <opencv2/core.hpp>

#include "track.h"
#include "munkres.h"
#include "utils.h"

class Tracker {
public:
    Tracker();
    ~Tracker() = default;

    static float CalculateIou(const cv::Rect& det, const Track& track);

    static void HungarianMatching(const std::vector<std::vector<float>>& iou_matrix,
                           size_t nrows, size_t ncols,
                           std::vector<std::vector<float>>& association);

/**
 * Assigns detections to tracked object (both represented as bounding boxes)
 * Returns 2 lists of matches, unmatched_detections
 * @param detection
 * @param tracks
 * @param matched
 * @param unmatched_det
 * @param iou_threshold
 */
    static void AssociateDetectionsToTrackers(const std::vector<cv::Rect>& detection,
                                       std::map<int, Track>& tracks,
                                       std::map<int, cv::Rect>& matched,
                                       std::vector<cv::Rect>& unmatched_det,
                                       float iou_threshold = 0.3);

    void Run(const std::vector<cv::Rect>& detections);

    std::map<int, Track> GetTracks();

private:
    // Hash-map between ID and corresponding tracker
    std::map<int, Track> tracks_;

    // Assigned ID for each bounding box
    int id_;
};