#include <iostream>
#include <fstream>
#include <map>
#include <random>
#include <opencv2/core.hpp>
#include "opencv2/imgproc.hpp"
#include <opencv2/highgui.hpp>

#include "tracker.h"
#include "munkres.h"
#include "utils.h"

std::vector<std::vector<cv::Rect>> ProcessLabel(std::ifstream& label_file) {
    // Process labels - group bounding boxes by frame index
    std::vector<std::vector<cv::Rect>> bbox;
    std::vector<cv::Rect> bbox_per_frame;
    // Label index starts from 1
    int current_frame_index = 1;
    std::string line;

    while (std::getline(label_file, line)) {
        std::stringstream ss(line);
        // Label format <frame>, <id>, <bb_left>, <bb_top>, <bb_width>, <bb_height>, <conf>, <x>, <y>, <z>
        std::vector<float> label;
        std::string data;
        while (getline(ss , data, ',')) {
            label.push_back(std::stof(data));
        }

        if (label[0] != current_frame_index) {
            current_frame_index = static_cast<int>(label[0]);
            bbox.push_back(bbox_per_frame);
            bbox_per_frame.clear();
        }
        bbox_per_frame.emplace_back(label[2], label[3], label[4], label[5]);
    }
    // Add bounding boxes from last frame
    bbox.push_back(bbox_per_frame);
    return bbox;
}


float CalculateIou(const cv::Rect& det, const Tracker& track) {
    auto trk = track.GetStateAsBbox();
    // calculate area of intersection and union
    auto xx1 = std::max(det.tl().x, trk.tl().x);
    auto yy1 = std::max(det.tl().y, trk.tl().y);
    auto xx2 = std::min(det.br().x, trk.br().x);
    auto yy2 = std::min(det.br().y, trk.br().y);
    auto w = std::max(0, xx2 - xx1);
    auto h = std::max(0, yy2 - yy1);
    auto intersection_area = w * h;
    float det_area = det.area();
    float trk_area = trk.area();
    float union_area = det_area + trk_area - intersection_area;
    auto iou = intersection_area / union_area;
    return iou;
}


void HungarianMatching(const std::vector<std::vector<float>>& iou_matrix,
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

    for (size_t i = 0 ; i < nrows ; i++) {
        for (size_t j = 0 ; j < ncols ; j++) {
            association[i][j] = matrix(i, j);
        }
    }
}


void AssociateDetectionsToTrackers(const std::vector<cv::Rect>& detection,
        std::map<int, Tracker>& tracks,
        std::map<int, cv::Rect>& matched,
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
                //filter out matched with low IOU
                if (iou_matrix[i][j] < iou_threshold) {
                    break;
                }
                matched[trk.first] = detection[i];
                matched_flag = true;
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

// TODO: add arguments to enable or disable display
// TODO: add output format to MOT

int main() {
    // Label file
    std::ifstream label_file("../data/ADL-Rundle-6/det.txt");
    if (!label_file.is_open()) {
        std::cerr << "Could not open or find the label!!!" << std::endl;
        return -1;
    }

    std::vector<std::vector<cv::Rect>> all_detections = ProcessLabel(label_file);
    label_file.close();

    // Load images
    cv::String path("../mot_benchmark/train/ADL-Rundle-6/img1/*.jpg");
    std::vector<cv::String> images;
    // Non-recursive
    cv::glob(path, images);

    // Create a window to display original image
    cv::namedWindow("Original", cv::WINDOW_AUTOSIZE);
    // Create a window to display tracking result
    cv::namedWindow("Tracking", cv::WINDOW_AUTOSIZE);

    // Generate random colors to visualize different bbox
    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
    int max_random_value = 20;
    std::uniform_int_distribution<> dis(0, max_random_value);
    int factor = 255 / max_random_value;
    constexpr int num_of_colors = 32;

    std::vector<cv::Scalar> colors;
    for (int n = 0; n < num_of_colors; ++n) {
        //Use dis to transform the random unsigned int generated by gen into an int in [0, 7]
        colors.emplace_back(cv::Scalar(dis(gen) * factor, dis(gen) * factor, dis(gen) * factor));
    }

    int current_frame_index = 0;
    // Assigned ID for each bounding box
    int current_ID = 0;
    // Hash-map between ID and corresponding tracker
    std::map<int, Tracker> tracks;

    for(const auto & image : images) {
        // Read image file
        cv::Mat img = imread(image);
        // Make a copy for display
        cv::Mat img_tracking = img.clone();

        // Check for invalid input
        if (img.empty()) {
            std::cerr << "Could not open or find the image!!!" << std::endl;
            return -1;
        }

        /*** Predict internal tracks from previous frame ***/
        const float dt = 0.1f;
        for (auto& track : tracks) {
            track.second.Predict(dt);
        }

        /*** Build association ***/
        const auto& detections = all_detections[current_frame_index];
        for (const auto& det : detections) {
            // Draw detected bounding boxes in red
            cv::rectangle(img, det, cv::Scalar(0,0,255), 3);
        }

        // Hash-map between track ID and associated detection bounding box
        std::map<int, cv::Rect> matched;
        // vector of unassociated detections
        std::vector<cv::Rect> unmatched_det;

        // return values - matched, unmatched_det
        AssociateDetectionsToTrackers(detections, tracks, matched, unmatched_det);

        /*** Update tracks with associated bbox ***/
        for (const auto& match : matched) {
            const auto& ID = match.first;
            tracks[ID].Update(match.second);
        }

        /*** Create new tracks for unmatched detections ***/
        for (const auto& det : unmatched_det) {
            Tracker tracker;
            tracker.Init(det);
            // Create new track and generate new ID
            tracks[current_ID++] = tracker;
        }

        /*** Delete lose tracked tracks ***/
        for (auto it = tracks.begin(); it != tracks.end();) {
            if (it->second.coast_cycles_ > kMaxCoastCycles) {
                it = tracks.erase(it);
            }
            else {
                it++;
            }
        }


        // Visualize tracking result
        for (auto& trk : tracks) {
            if (trk.second.frame_count_ < kMinHits || trk.second.hit_streak_ > kMinHits) {
                const auto bbox = trk.second.GetStateAsBbox();
                cv::putText(img_tracking, std::to_string(trk.first), cv::Point(bbox.tl().x, bbox.tl().y - 10),
                            cv::FONT_HERSHEY_DUPLEX, 2, cv::Scalar(255, 255, 255), 2);
                cv::rectangle(img_tracking, bbox, colors[trk.first % num_of_colors], 3);
            }
        }

        // Show our image inside it
        cv::imshow("Original", img);
        cv::imshow("Tracking", img_tracking);

        // Delay in ms
        auto key = cv::waitKey(static_cast<int>(dt * 1000));
        // Exit if ESC pressed
        if (27 == key) {
            break;
        }
        else if (32 == key) {
            // Press Space to pause and press it again to resume
            while(true) {
                key = cv::waitKey(0);
                if (32 == key) {
                    break;
                }
                else if (27 == key) {
                    return 0;
                }
            }
        }

        // Accumulate frame index
        current_frame_index++;
    }

    return 0;
}
