/**
 * SORT: A Simple, Online and Realtime Tracker
 */

#include <iostream>
#include <fstream>
#include <map>
#include <random>
#include <chrono>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <boost/program_options.hpp>

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


/**
 * Assigns detections to tracked object (both represented as bounding boxes)
 * Returns 2 lists of matches, unmatched_detections
 * @param detection
 * @param tracks
 * @param matched
 * @param unmatched_det
 * @param iou_threshold
 */
void AssociateDetectionsToTrackers(const std::vector<cv::Rect>& detection,
        std::map<int, Tracker>& tracks,
        std::map<int, cv::Rect>& matched,
        std::vector<cv::Rect>& unmatched_det,
        float iou_threshold = 0.3) {

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


// TODO: choose dataset
// TODO: unit test
// TODO: log history of each track

int main(int argc, const char *argv[]) {
    // parse program input arguments
    boost::program_options::options_description desc{"Options"};
    desc.add_options()
            ("help,h", "Help screen")
            ("display,d", "Display online tracker output (slow) [False]");

    boost::program_options::variables_map vm;
    boost::program_options::store(parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << '\n';
        return 1;
    }

    bool enable_display_flag = false;
    if (vm.count("display")) {
        enable_display_flag = true;
    }

    // Open label file
    // TODO: process different sequences


    // ADL-Rundle-6, TUD-Campus, PETS09-S2L1
    std::string dataset_name = "TUD-Campus";
    std::string label_path = "../data/" + dataset_name + "/det.txt";
    std::ifstream label_file(label_path);
    if (!label_file.is_open()) {
        std::cerr << "Could not open or find the label!!!" << std::endl;
        return -1;
    }
    std::vector<std::vector<cv::Rect>> all_detections = ProcessLabel(label_file);
    label_file.close();

    // For visualization
    std::vector<cv::String> images;
    std::vector<cv::Scalar> colors;
    constexpr int num_of_colors = 32;
    if (enable_display_flag) {
        // Load images
        cv::String path("../mot_benchmark/train/" + dataset_name + "/img1/*.jpg");
        // Non-recursive
        cv::glob(path, images);

        // Create a window to display original image
        cv::namedWindow("Original", cv::WINDOW_AUTOSIZE);
        // Create a window to display tracking result
        cv::namedWindow("Tracking", cv::WINDOW_AUTOSIZE);

        // Generate random colors to visualize different bbox
        std::random_device rd;  //Will be used to obtain a seed for the random number engine
        std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
        constexpr int max_random_value = 20;
        std::uniform_int_distribution<> dis(0, max_random_value);
        constexpr int factor = 255 / max_random_value;

        for (int n = 0; n < num_of_colors; ++n) {
            //Use dis to transform the random unsigned int generated by gen into an int in [0, 7]
            colors.emplace_back(cv::Scalar(dis(gen) * factor, dis(gen) * factor, dis(gen) * factor));
        }
    }

    std::string output_path = "../output/" + dataset_name + ".txt";
    std::ofstream output_file(output_path);

    std::string output_path_NIS = "../output/" + dataset_name + "-NIS.txt";
    std::ofstream output_file_NIS(output_path_NIS);

    // TODO: check if output folder exist
    if (output_file.is_open()) {
        std::cout << "Result will be exported to " << output_path << std::endl;
    }
    else {
        std::cerr << "Unable to open output file" << std::endl;
        return -1;
    }

    // Assigned ID for each bounding box
    int current_ID = 0;
    // Hash-map between ID and corresponding tracker
    std::map<int, Tracker> tracks;

    size_t total_frames = all_detections.size();

    auto t1 = std::chrono::high_resolution_clock::now();
    for(size_t i = 0; i < total_frames; i++) {
        auto frame_index = i + 1;
        std::cout << "************* NEW FRAME ************* " << std::endl;


        /*** Predict internal tracks from previous frame ***/
        constexpr float dt = 0.1f;
        for (auto& track : tracks) {
            track.second.Predict(dt);
        }

        /*** Build association ***/
        const auto& detections = all_detections[i];

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

        for (auto &trk : tracks) {
            const auto& bbox = trk.second.GetStateAsBbox();

            // TODO: think about if we need to export coasted tracks or if we need to increase kMaxCoastCycles???
            if (trk.second.coast_cycles_< 1 && (trk.second.hit_streak_ >= kMinHits || frame_index < kMinHits)) {
                // Print to terminal for debugging
                std::cout << frame_index << "," << trk.first << "," << bbox.tl().x << "," << bbox.tl().y
                          << "," << bbox.width << "," << bbox.height << ",1,-1,-1,-1"
                          << " Frame Count = "<< trk.second.frame_count_
                          << " Hit Streak = "<< trk.second.hit_streak_
                          << " Coast Cycles = "<< trk.second.coast_cycles_ <<  std::endl;

                // Export to text file for metrics evaluation
                output_file << frame_index << "," << trk.first << "," << bbox.tl().x << "," << bbox.tl().y
                            << "," << bbox.width << "," << bbox.height << ",1,-1,-1,-1\n" ;

                output_file_NIS << trk.second.GetNIS() << "\n";
            }
        }

        // Visualize tracking result
        if(enable_display_flag) {
            // Read image file
            cv::Mat img = imread(images[i]);
            // Make a copy for display
            cv::Mat img_tracking = img.clone();

            // Check for invalid input
            if (img.empty()) {
                std::cerr << "Could not open or find the image!!!" << std::endl;
                return -1;
            }

            for (const auto &det : detections) {
                // Draw detected bounding boxes in red
                cv::rectangle(img, det, cv::Scalar(0, 0, 255), 3);
            }

            for (auto &trk : tracks) {
                if (trk.second.hit_streak_ >= kMinHits || trk.second.frame_count_ < kMinHits) {
                    const auto& bbox = trk.second.GetStateAsBbox();
                    cv::putText(img_tracking, std::to_string(trk.first), cv::Point(bbox.tl().x, bbox.tl().y - 10),
                                cv::FONT_HERSHEY_DUPLEX, 2, cv::Scalar(255, 255, 255), 2);
                    cv::rectangle(img_tracking, bbox, colors[trk.first % num_of_colors], 3);
                }
            }

            // Show our image inside it
            cv::imshow("Original", img);
            cv::imshow("Tracking", img_tracking);

            // Delay in ms
            //auto key = cv::waitKey(static_cast<int>(dt * 1000));
            auto key = cv::waitKey(0);


            // Exit if ESC pressed
            if (27 == key) {
                break;
            } else if (32 == key) {
                // Press Space to pause and press it again to resume
                while (true) {
                    key = cv::waitKey(0);
                    if (32 == key) {
                        break;
                    } else if (27 == key) {
                        return 0;
                    }
                }
            }
        } // end of enable_display_flag
    } // end of iterating all frames
    auto t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);

    std::cout << "********************************" << std::endl;
    std::cout << "Total tracking took: " << time_span.count() << " for " << total_frames << " frames" << std::endl;
    std::cout << "FPS = " << total_frames / time_span.count()  << std::endl;
    if (enable_display_flag) {
        std::cout << "Note: to get real runtime results run without the option: --display" << std::endl;
    }
    std::cout << "********************************" << std::endl;

    output_file.close();

    return 0;
}
