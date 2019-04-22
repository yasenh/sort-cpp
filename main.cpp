#include <iostream>
#include <fstream>
#include <opencv2/core.hpp>
#include "opencv2/imgproc.hpp"
#include <opencv2/highgui.hpp>



int main() {
    // Label
    std::ifstream label_file("../data/ADL-Rundle-6/det.txt");
    if (!label_file.is_open()) {
        std::cerr << "Could not open or find the label!!!" << std::endl;
        return -1;
    }

    // Process labels - group bounding boxes by frame index
    std::string line;
    unsigned int current_frame_index = 1;

    std::vector<std::vector<cv::Rect>> bbox;
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

        //labels.push_back(label);
        bbox_per_frame.emplace_back(label[2], label[3], label[4], label[5]);
    }
    // Last frame
    bbox.push_back(bbox_per_frame);

    // Load images
    cv::String path("../mot_benchmark/train/ADL-Rundle-6/img1/*.jpg");
    std::vector<cv::String> images;
    // Non-recursive
    cv::glob(path, images);

    cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE); // Create a window for display.

    current_frame_index = 0;
    for(const auto & image : images) {
        cv::Mat img = imread(image); // Read the file
        // Check for invalid input
        if (img.empty()) {
            std::cerr << "Could not open or find the image!!!" << std::endl;
            return -1;
        }


        for (const auto& box : bbox[current_frame_index]) {
            cv::rectangle(img, box, cv::Scalar(0,0,255), 3);
        }
        cv::imshow("Display window", img);                // Show our image inside it.

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

    label_file.close();
    return 0;
}
