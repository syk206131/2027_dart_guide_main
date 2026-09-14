#include <opencv2/opencv.hpp>
#include "io/hikrobot/hikrobot.hpp"
#include "tools/yaml.hpp"

#include <iostream>
#include <vector>
#include <chrono>

int main(int argc, char **argv) {
    std::string config_path = "configs/test.yaml";
    if (argc > 1) {
        config_path = argv[1];
    }

    auto yaml = tools::load(config_path);

    double exposure = tools::read<double>(yaml, "exposure_ms");
    double gain = tools::read<double>(yaml, "gain");
    std::string vid_pid = tools::read<std::string>(yaml, "vid_pid");

    io::HikRobot camera(exposure, gain, vid_pid);

    cv::Mat img;
    std::chrono::steady_clock::time_point ts;

    // initial HSV bounds, same defaults as python version
    int hl = 35, hh = 85, sl = 100, sh = 255, vl = 100, vh = 255;

    cv::namedWindow("raw", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("circles", cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("H Low", "circles", &hl, 179);
    cv::createTrackbar("H High", "circles", &hh, 179);
    cv::createTrackbar("S Low", "circles", &sl, 255);
    cv::createTrackbar("S High", "circles", &sh, 255);
    cv::createTrackbar("V Low", "circles", &vl, 255);
    cv::createTrackbar("V High", "circles", &vh, 255);

    std::cout << "Press 'q' to quit.\n";
    while (true) {
        camera.read(img, ts);
        if (img.empty()) {
            std::cerr << "empty frame\n";
            continue;
        }

        cv::Mat hsv;
        cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
        cv::Scalar lower(hl, sl, vl);
        cv::Scalar upper(hh, sh, vh);

        cv::Mat mask;
        cv::inRange(hsv, lower, upper, mask);

        cv::Mat circles = cv::Mat::zeros(img.size(), img.type());
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        for (auto &cnt : contours) {
            cv::Point2f center;
            float radius;
            cv::minEnclosingCircle(cnt, center, radius);
            if (radius > 2) {
                cv::circle(circles, center, (int)radius, cv::Scalar(0, 255, 0), 2);
            }
        }

        cv::imshow("raw", img);
        cv::imshow("circles", circles);

        char c = (char)cv::waitKey(1);
        if (c == 'q') break;
    }

    return 0;
}
