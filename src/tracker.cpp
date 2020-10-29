#include "tracker.h"


Tracker::Tracker() {
    id_ = 0;
}

float Tracker::CalculateIou(const cv::Rect& det, const Track& track) {
    auto trk = track.GetStateAsBbox();
    // get min/max points
    auto xx1 = std::max(det.tl().x, trk.tl().x);
    auto yy1 = std::max(det.tl().y, trk.tl().y);
    auto xx2 = std::min(det.br().x, trk.br().x);
    auto yy2 = std::min(det.br().y, trk.br().y);
    auto w = std::max(0, xx2 - xx1);
    auto h = std::max(0, yy2 - yy1);

    // calculate area of intersection and union
    float det_area = det.area();
    float trk_area = trk.area();
    auto intersection_area = w * h;
    float union_area = det_area + trk_area - intersection_area;
    auto iou = intersection_area / union_area;
    return iou;
}


void Tracker::HungarianMatching(const std::vector<std::vector<float>>& iou_matrix,
                                size_t nrows, size_t ncols,
                                std::vector<std::vector<float>>& association) {
    Matrix<float> matrix(nrows, ncols);
    // Initialize matrix with IOU values
    for (size_t i = 0 ; i < nrows ; i++) {
        for (size_t j = 0 ; j < ncols ; j++) {
            // Multiply by -1 to find max cost
            if (iou_matrix[i][j] != 0) {
                matrix(i, j) = -iou_matrix[i][j];
            }
            else {
                // TODO: figure out why we have to assign value to get correct result
                matrix(i, j) = 1.0f;
            }
        }
    }

//    // Display begin matrix state.
//    for (size_t row = 0 ; row < nrows ; row++) {
//        for (size_t col = 0 ; col < ncols ; col++) {
//            std::cout.width(10);
//            std::cout << matrix(row,col) << ",";
//        }
//        std::cout << std::endl;
//    }
//    std::cout << std::endl;


    // Apply Kuhn-Munkres algorithm to matrix.
    Munkres<float> m;
    m.solve(matrix);

//    // Display solved matrix.
//    for (size_t row = 0 ; row < nrows ; row++) {
//        for (size_t col = 0 ; col < ncols ; col++) {
//            std::cout.width(2);
//            std::cout << matrix(row,col) << ",";
//        }
//        std::cout << std::endl;
//    }
//    std::cout << std::endl;

    for (size_t i = 0 ; i < nrows ; i++) {
        for (size_t j = 0 ; j < ncols ; j++) {
            association[i][j] = matrix(i, j);
        }
    }
}


void Tracker::AssociateDetectionsToTrackers(const std::vector<cv::Rect>& detection,
                                            std::map<int, Track>& tracks,
                                            std::map<int, cv::Rect>& matched,
                                            std::vector<cv::Rect>& unmatched_det,
                                            float iou_threshold) {

    // Set all detection as unmatched if no tracks existing
    if (tracks.empty()) {
        for (const auto& det : detection) {
            unmatched_det.push_back(det);
        }
        return;
    }

    std::vector<std::vector<float>> iou_matrix;
    // resize IOU matrix based on number of detection and tracks
    iou_matrix.resize(detection.size(), std::vector<float>(tracks.size()));

    std::vector<std::vector<float>> association;
    // resize association matrix based on number of detection and tracks
    association.resize(detection.size(), std::vector<float>(tracks.size()));


    // row - detection, column - tracks
    for (size_t i = 0; i < detection.size(); i++) {
        size_t j = 0;
        for (const auto& trk : tracks) {
            iou_matrix[i][j] = CalculateIou(detection[i], trk.second);
            j++;
        }
    }

    // Find association
    HungarianMatching(iou_matrix, detection.size(), tracks.size(), association);

    for (size_t i = 0; i < detection.size(); i++) {
        bool matched_flag = false;
        size_t j = 0;
        for (const auto& trk : tracks) {
            if (0 == association[i][j]) {
                // Filter out matched with low IOU
                if (iou_matrix[i][j] >= iou_threshold) {
                    matched[trk.first] = detection[i];
                    matched_flag = true;
                }
                // It builds 1 to 1 association, so we can break from here
                break;
            }
            j++;
        }
        // if detection cannot match with any tracks
        if (!matched_flag) {
            unmatched_det.push_back(detection[i]);
        }
    }
}


void Tracker::Run(const std::vector<cv::Rect>& detections) {

    /*** Predict internal tracks from previous frame ***/
    for (auto &track : tracks_) {
        track.second.Predict();
    }

    // Hash-map between track ID and associated detection bounding box
    std::map<int, cv::Rect> matched;
    // vector of unassociated detections
    std::vector<cv::Rect> unmatched_det;

    // return values - matched, unmatched_det
    if (!detections.empty()) {
        AssociateDetectionsToTrackers(detections, tracks_, matched, unmatched_det);
    }

    /*** Update tracks with associated bbox ***/
    for (const auto &match : matched) {
        const auto &ID = match.first;
        tracks_[ID].Update(match.second);
    }

    /*** Create new tracks for unmatched detections ***/
    for (const auto &det : unmatched_det) {
        Track tracker;
        tracker.Init(det);
        // Create new track and generate new ID
        tracks_[id_++] = tracker;
    }

    /*** Delete lose tracked tracks ***/
    for (auto it = tracks_.begin(); it != tracks_.end();) {
        if (it->second.coast_cycles_ > kMaxCoastCycles) {
            it = tracks_.erase(it);
        } else {
            it++;
        }
    }
}


std::map<int, Track> Tracker::GetTracks() {
    return tracks_;
}