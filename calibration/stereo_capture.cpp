// Stereo calibration pair capture tool.
//
// Shows synchronized left/right frames side by side, overlays live
// chessboard detection as framing feedback, and saves the newest synchronized
// full-resolution frame pair when SPACE is pressed. Saved pairs feed
// stereo_calibration_tool.
//
// Usage:
//   stereo_capture [--config <yaml>] [--output <dir>]
//                  [--corners <columns> <rows>]

#include <atomic>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <string>
#include <thread>

#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "tools/exiter.hpp"
#include "tools/stereo_frame_queue.hpp"
#include "tools/yaml.hpp"

namespace
{
struct CaptureOptions
{
  std::string config_path = "configs/test.yaml";
  std::string output_dir = "calib_pairs";
  cv::Size board_size{11, 8};
};

void print_usage(const char * program)
{
  std::cout
    << "Usage: " << program
    << " [--config <yaml>] [--output <dir>] [--corners <columns> <rows>]\n"
    << "  SPACE     save the newest synchronized pair as BMP\n"
    << "  q / ESC   quit\n";
}

cv::Mat draw_feedback(const cv::Mat & frame, const cv::Size & board_size, const char * tag)
{
  cv::Mat display;
  cv::resize(frame, display, cv::Size(640, 480));

  // Live chessboard detection is only a framing aid, so it runs on the
  // downscaled display copy and never affects the saved full-res frames.
  cv::Mat gray;
  cv::cvtColor(display, gray, cv::COLOR_BGR2GRAY);
  std::vector<cv::Point2f> corners;
  const bool found = cv::findChessboardCorners(
    gray, board_size, corners,
    cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);
  cv::drawChessboardCorners(display, board_size, corners, found);

  cv::putText(
    display, tag, cv::Point(10, 28), cv::FONT_HERSHEY_SIMPLEX, 0.8,
    found ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);
  cv::putText(
    display, found ? "board detected" : "board NOT detected", cv::Point(10, 455),
    cv::FONT_HERSHEY_SIMPLEX, 0.6, found ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);
  return display;
}
}  // namespace

int main(int argc, char ** argv)
{
  CaptureOptions options;

  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--config") {
      if (index + 1 >= argc) { print_usage(argv[0]); return 1; }
      options.config_path = argv[++index];
    } else if (argument == "--output") {
      if (index + 1 >= argc) { print_usage(argv[0]); return 1; }
      options.output_dir = argv[++index];
    } else if (argument == "--corners") {
      if (index + 2 >= argc) { print_usage(argv[0]); return 1; }
      options.board_size.width = std::stoi(argv[++index]);
      options.board_size.height = std::stoi(argv[++index]);
    } else {
      print_usage(argv[0]);
      return 1;
    }
  }

  const auto yaml = tools::load(options.config_path);
  const auto sync_max_delta_ms =
    yaml["sync_max_delta_ms"] ? yaml["sync_max_delta_ms"].as<double>() : 8.0;
  const auto sync_queue_size =
    yaml["sync_queue_size"] ? yaml["sync_queue_size"].as<int>() : 8;
  const auto sync_max_delta = std::chrono::duration_cast<tools::StereoFrameQueue::Duration>(
    std::chrono::duration<double, std::milli>(sync_max_delta_ms));

  const std::filesystem::path left_dir =
    std::filesystem::path(options.output_dir) / "left";
  const std::filesystem::path right_dir =
    std::filesystem::path(options.output_dir) / "right";
  std::filesystem::create_directories(left_dir);
  std::filesystem::create_directories(right_dir);

  // Continue numbering after whatever pairs already exist in the directory.
  std::size_t saved_count = 0;
  for (const auto & entry : std::filesystem::directory_iterator(left_dir)) {
    if (entry.path().extension() == ".bmp") ++saved_count;
  }

  io::Camera left_camera(options.config_path, "left_camera");
  io::Camera right_camera(options.config_path, "right_camera");
  tools::Exiter exiter;

  tools::StereoFrameQueue stereo_queue(
    static_cast<std::size_t>(sync_queue_size), sync_max_delta);
  std::atomic<bool> capture_quit{false};

  auto capture_loop = [&](io::Camera & camera, tools::StereoCameraSide side, const char * name) {
    while (!capture_quit.load() && !exiter.exit()) {
      cv::Mat image;
      std::chrono::steady_clock::time_point timestamp;
      if (!camera.read(image, timestamp)) break;
      if (image.empty()) continue;
      stereo_queue.push(side, {std::move(image), timestamp});
    }
    capture_quit.store(true);
    stereo_queue.stop();
    (void)name;
  };

  std::thread left_thread(
    capture_loop, std::ref(left_camera), tools::StereoCameraSide::LEFT, "left");
  std::thread right_thread(
    capture_loop, std::ref(right_camera), tools::StereoCameraSide::RIGHT, "right");

  cv::namedWindow("Stereo Calibration Capture", cv::WINDOW_AUTOSIZE);
  std::cout << "Saving pairs to: " << options.output_dir << "/{left,right}\n"
            << "Press SPACE to save, q or ESC to quit.\n";

  while (!exiter.exit()) {
    tools::StereoFramePair pair;
    if (!stereo_queue.pop(pair)) break;

    const double delta_ms =
      std::chrono::duration<double, std::milli>(pair.left.timestamp - pair.right.timestamp)
        .count();

    const auto left_display = draw_feedback(pair.left.image, options.board_size, "LEFT");
    const auto right_display = draw_feedback(pair.right.image, options.board_size, "RIGHT");
    cv::Mat canvas;
    cv::hconcat(left_display, right_display, canvas);

    const std::string hud =
      "saved pairs: " + std::to_string(saved_count) +
      "  |  frame delta: " + cv::format("%+.2f ms", delta_ms) +
      "  |  SPACE=save  q=quit";
    cv::putText(
      canvas, hud, cv::Point(12, 475), cv::FONT_HERSHEY_SIMPLEX, 0.55,
      cv::Scalar(0, 255, 255), 1);
    cv::imshow("Stereo Calibration Capture", canvas);

    const int key = cv::waitKey(1) & 0xFF;
    if (key == 'q' || key == 27) break;
    if (key == ' ') {
      ++saved_count;
      char file_name[32];
      std::snprintf(file_name, sizeof(file_name), "%04zu.bmp", saved_count);
      const auto left_path = left_dir / file_name;
      const auto right_path = right_dir / file_name;
      if (cv::imwrite(left_path.string(), pair.left.image) &&
          cv::imwrite(right_path.string(), pair.right.image)) {
        std::cout << "saved pair " << saved_count << " ("
                  << left_path.string() << ", " << right_path.string() << ")\n";
      } else {
        std::cerr << "failed to write pair " << saved_count << '\n';
        --saved_count;
      }
    }
  }

  capture_quit.store(true);
  left_camera.stop();
  right_camera.stop();
  stereo_queue.stop();
  if (left_thread.joinable()) left_thread.join();
  if (right_thread.joinable()) right_thread.join();

  std::cout << "Done. " << saved_count << " pair(s) in "
            << std::filesystem::absolute(options.output_dir) << '\n';
  return 0;
}
