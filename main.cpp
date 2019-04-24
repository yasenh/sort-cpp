#include <iostream>
#include <fstream>
#include <map>
#include <opencv2/core.hpp>
#include "opencv2/imgproc.hpp"
#include <opencv2/highgui.hpp>

#include "tracker.h"
#include "munkres.h"
#include "adapters/boostmatrixadapter.h"

void ProcessLabel(std::ifstream& label_file,
        std::vector<std::vector<cv::Rect>>& bbox) {
    // Process labels - group bounding boxes by frame index
    std::string line;
    unsigned int current_frame_index = 1;

    std::vector<cv::Rect> bbox_per_frame;
    while (std::getline(label_file, line)) {
        std::stringstream ss(line);
        // Label format <frame>, <id>, <bb_left>, <bb_top>, <bb_width>, <bb_height>, <conf>, <x>, <y>, <z>
        std::vector<float> label;
        std::string data;
        while (getline(ss , data, ',')) {
            label.push_back(std::stof(data));
        }

        if (label[0] != current_frame_index) {
            current_frame_index = static_cast<unsigned int>(label[0]);
            bbox.push_back(bbox_per_frame);
            bbox_per_frame.clear();
        }
        bbox_per_frame.emplace_back(label[2], label[3], label[4], label[5]);
    }
    // Last frame
    bbox.push_back(bbox_per_frame);
}


float CalculateIou(const cv::Rect& det, Tracker& trk) {
    auto state = trk.GetState();

    //convert_x_to_bbox
    // state - center_x, center_y, area, ratio, v_cx, v_cy, v_area
    auto width = std::sqrt(state[2] * state[3]);
    auto height = state[2] / width;
    auto tl_x = static_cast<int>(state[0] - width / 2);
    auto tl_y = static_cast<int>(state[1] - height / 2);
    auto br_x = static_cast<int>(state[0] + width / 2);
    auto br_y = static_cast<int>(state[1] + height / 2);

    auto xx1 = std::max(det.tl().x, tl_x);
    auto yy1 = std::max(det.tl().y, tl_y);
    auto xx2 = std::min(det.br().x, br_x);
    auto yy2 = std::min(det.br().y, br_y);
    auto w = std::max(0, xx2 - xx1);
    auto h = std::max(0, yy2 - yy1);
    auto intersection_area = w * h;
    float det_area = det.area();
    float trk_area = (br_x - tl_x) * (br_y - tl_y);
    float union_area = det_area + trk_area - intersection_area;
    auto iou = intersection_area / union_area;
    return iou;
}


void HungarianMatching(const std::vector<std::vector<float>>& iou_matrix,
        size_t nrows, size_t ncols,
        std::vector<std::vector<float>>& association) {
    // https://github.com/yasenh/sort-cpp/blob/master/main.cpp
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

    // Display begin matrix state.
    for (size_t row = 0 ; row < nrows ; row++) {
        for (size_t col = 0 ; col < ncols ; col++) {
            std::cout.width(10);
            std::cout << matrix(row,col) << ",";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;


    // Apply Kuhn-Munkres algorithm to matrix.
    Munkres<float> m;
    m.solve(matrix);

    // Display solved matrix.
    for (size_t row = 0 ; row < nrows ; row++) {
        for (size_t col = 0 ; col < ncols ; col++) {
            std::cout.width(2);
            std::cout << matrix(row,col) << ",";
        }
        std::cout << std::endl;
    }

    std::cout << std::endl;


    // Trick: copy assignment result back to IOU matrix
    for (size_t i = 0 ; i < nrows ; i++) {
        for (size_t j = 0 ; j < ncols ; j++) {
            association[i][j] = matrix(i, j);
        }
    }
}

void AssociateDetectionsToTrackers(const std::vector<cv::Rect>& detection,
        std::map<unsigned int, Tracker>& tracks,
        std::map<unsigned int, cv::Rect>& matched,
        std::vector<cv::Rect>& unmatched_det,
        float iou_threshold = 0.3) {

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
        for (auto& trk : tracks) {
            iou_matrix[i][j] = CalculateIou(detection[i], trk.second);
            j++;
        }
    }

    HungarianMatching(iou_matrix, detection.size(), tracks.size(), association);

    for (size_t i = 0; i < detection.size(); i++) {
        bool matched_flag = false;
        size_t j = 0;
        for (auto& trk : tracks) {
            if (0 == association[i][j]) {
                if (iou_matrix[i][j] < iou_threshold) {
                    //filter out matched with low IOU
                    break;
                }
                matched[trk.first] = detection[i];
                matched_flag = true;
                break;
            }
            j++;
        }

        if (!matched_flag) {
            unmatched_det.push_back(detection[i]);
        }
    }
}


int main() {
    // Label
    std::ifstream label_file("../data/ADL-Rundle-6/det.txt");
    if (!label_file.is_open()) {
        std::cerr << "Could not open or find the label!!!" << std::endl;
        return -1;
    }

    std::vector<std::vector<cv::Rect>> bbox;
    ProcessLabel(label_file, bbox);
    label_file.close();

    // Load images
    cv::String path("../mot_benchmark/train/ADL-Rundle-6/img1/*.jpg");
    std::vector<cv::String> images;
    // Non-recursive
    cv::glob(path, images);

    cv::namedWindow("Original", cv::WINDOW_AUTOSIZE); // Create a window for display.
    cv::namedWindow("Tracking", cv::WINDOW_AUTOSIZE); // Create a window for display.

    unsigned int current_frame_index = 0;
    unsigned int current_ID = 0;
    // Hash-map between ID and corresponding tracker
    std::map<unsigned int, Tracker> tracks;

    for(const auto & image : images) {
        cv::Mat img = imread(image); // Read the file
        cv::Mat img_tracking = img.clone();

        // Check for invalid input
        if (img.empty()) {
            std::cerr << "Could not open or find the image!!!" << std::endl;
            return -1;
        }


        // Predict internal tracks from previous frame to current one
        // TODO: user can define dt
        const float dt = 0.1f;
        for (auto& track : tracks) {
            track.second.Predict(dt);
        }

        for (auto it = tracks.begin(); it != tracks.end();) {
            if (it->second.age_ > it->second.max_age_) {
                it = tracks.erase(it);
            }
            else {
                it++;
            }
        }

        /*** Build association ***/
        for (const auto& box : bbox[current_frame_index]) {
            // draw current bounding box in red
            cv::rectangle(img, box, cv::Scalar(0,0,255), 3);
        }



        std::map<unsigned int, cv::Rect> matched;
        std::vector<cv::Rect> unmatched_det;

        // return values - matched, unmatched_det
        AssociateDetectionsToTrackers(bbox[current_frame_index], tracks, matched, unmatched_det);

        /*** Update tracks with associated bbox ***/
        for (const auto& match : matched) {
            const auto& ID = match.first;
            tracks[ID].Update(match.second);
        }

        for (auto& det : unmatched_det) {
            Tracker tracker;
            tracker.Init(det);
            tracks[current_ID++] = tracker;
        }

        for (auto& trk : tracks) {
            auto state = trk.second.GetState();
            //convert_x_to_bbox
            // state - center_x, center_y, area, ratio, v_cx, v_cy, v_area
            auto width = std::sqrt(state[2] * state[3]);
            auto height = state[2] / width;
            auto tl_x = static_cast<int>(state[0] - width / 2);
            auto tl_y = static_cast<int>(state[1] - height / 2);
            auto br_x = static_cast<int>(state[0] + width / 2);
            auto br_y = static_cast<int>(state[1] + height / 2);

            cv::putText(img_tracking, std::to_string(trk.first), cv::Point(tl_x, tl_y - 10), cv::FONT_HERSHEY_DUPLEX, 2, cv::Scalar(255, 255, 255), 2);
            cv::rectangle(img_tracking, cv::Point(tl_x, tl_y), cv::Point(br_x, br_y),cv::Scalar(0,255,0), 3);
        }

        // Show our image inside it
        cv::imshow("Original", img);
        cv::imshow("Tracking", img_tracking);

        auto key = cv::waitKey(100);
        // Exit if ESC pressed
        if (27 == key) {
            break;
        }
        else if (32 == key) {
            // Press Space to pause and press it again to resume
            while(32 != cv::waitKey(0));
        }

        // Accumulate frame index
        current_frame_index++;
    }


    return 0;
}
