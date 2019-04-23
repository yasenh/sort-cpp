#include <iostream>
#include <fstream>
#include <map>
#include <opencv2/core.hpp>
#include "opencv2/imgproc.hpp"
#include <opencv2/highgui.hpp>

#include "tracker.h"

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


void AssociateDetectionsToTrackers(const std::vector<cv::Rect>& detection,
        std::map<unsigned int, Tracker>& tracks,
        std::map<unsigned int, cv::Rect>& matched,
        std::vector<cv::Rect>& unmatched_det,
        float threshold = 0.3) {

    if (tracks.empty()) {
        for (const auto& det : detection) {
            unmatched_det.push_back(det);
        }
        return;
    }

    std::vector<std::vector<float>> iou_matrix;
    // resize IOU matrix based on number of detection and tracks
    iou_matrix.resize(detection.size(), std::vector<float>(tracks.size()));

    for (size_t i = 0; i < detection.size(); i++) {
        size_t j = 0;
        for (auto& trk : tracks) {
            iou_matrix[i][j] = CalculateIou(detection[i], trk.second);
            j++;
        }
    }

    // TODO: association ID and the figure out unmatched
    for (const auto& det : detection) {
        unmatched_det.push_back(det);
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

    cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE); // Create a window for display.

    unsigned int current_frame_index = 0;
    unsigned int current_ID = 0;
    // Hash-map between ID and corresponding tracker
    std::map<unsigned int, Tracker> tracks;

    for(const auto & image : images) {
        cv::Mat img = imread(image); // Read the file
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

        /*** Build association ***/
        for (const auto& box : bbox[current_frame_index]) {
            // draw current bounding box in red
            cv::rectangle(img, box, cv::Scalar(0,0,255), 3);
        }

        // matched, unmatched_dets, unmatched_trks

        std::map<unsigned int, cv::Rect> matched;
        std::vector<cv::Rect> unmatched_det;

        AssociateDetectionsToTrackers(bbox[current_frame_index], tracks, matched, unmatched_det);

        /*** Update tracks with associated bbox ***/
        for (auto& track : tracks) {
            //track.second.Update();
        }

        for (auto& det : unmatched_det) {
            Tracker tracker;
            tracker.Init(det);
            tracks[current_ID++] = tracker;
        }

        // Show our image inside it
        cv::imshow("Display window", img);

        auto key = cv::waitKey(33);
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
