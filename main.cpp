#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>


int main() {
    cv::String path("../mot_benchmark/train/ADL-Rundle-6/img1/*.jpg");
    std::vector<cv::String> images;
    // Non-recursive
    cv::glob(path, images);

    cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE); // Create a window for display.
    for(const auto & image : images) {
        cv::Mat img = imread(image); // Read the file
        // Check for invalid input
        if (img.empty()) {
            std::cout << "Could not open or find the image" << std::endl;
            return -1;
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
    }

    return 0;
}
