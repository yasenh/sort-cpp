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

void CalculateIou() {

}


void AssociateDetectionsToTrackers(const std::vector<cv::Rect>& detection,
        const std::map<unsigned int, Tracker>& tracks, float threshold = 0.3) {

    if (tracks.empty()) {
        // return np.empty((0,2),dtype=int), np.arange(len(detections)), np.empty((0,5),dtype=int)
        return;
    }

    std::vector<std::vector<float>> iou_matrix;
    // resize IOU matrix based on number of detection and tracks
    iou_matrix.resize(detection.size(), std::vector<float>(tracks.size()));

    for (size_t i = 0; i < detection.size(); i++) {
        for (size_t j = 0; j < tracks.size(); j++) {
            iou_matrix[i,j] = ;
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

    cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE); // Create a window for display.

    unsigned int current_frame_index = 0;
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
        AssociateDetectionsToTrackers(bbox[current_frame_index], tracks);

        /*** Update tracks with associated bbox ***/
        for (auto& track : tracks) {
            //track.second.Update();
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
