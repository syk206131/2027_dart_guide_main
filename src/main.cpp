#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/light_detect/Detect.hpp"
#include "tasks/stereo/stereo_range.hpp"
#include "tools/exiter.hpp"
#include "tools/stereo_frame_queue.hpp"
#include "tools/yaml.hpp"

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <functional>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace
{
struct TargetObservation
{
  bool valid = false;
  double score = 0.0;
  cv::Rect2d box;
  cv::Point2d center;
};

TargetObservation detect_target(
  LightDetect & detector, cv::Mat & image, const cv::Size2d & network_size, bool & startup,
  const cv::Scalar & hsv_lower, const cv::Scalar & hsv_upper)
{
  auto lights = detector.detect(image, network_size, 0, startup);
  startup = false;
  if (lights.empty()) return {};

  const auto best_it = std::max_element(
    lights.begin(), lights.end(),
    [](const auto & left, const auto & right) { return left.score < right.score; });

  TargetObservation result;
  result.valid = true;
  result.score = best_it->score;
  result.box = best_it->box;
  result.center = best_it->center_point;

  const cv::Rect image_bounds(0, 0, image.cols, image.rows);
  const cv::Rect2d clipped_box =
    result.box & cv::Rect2d(0.0, 0.0, image.cols, image.rows);
  const int x0 = std::clamp(static_cast<int>(std::floor(clipped_box.x)), 0, image.cols);
  const int y0 = std::clamp(static_cast<int>(std::floor(clipped_box.y)), 0, image.rows);
  const int x1 = std::clamp(
    static_cast<int>(std::ceil(clipped_box.x + clipped_box.width)), 0, image.cols);
  const int y1 = std::clamp(
    static_cast<int>(std::ceil(clipped_box.y + clipped_box.height)), 0, image.rows);

  if (x1 <= x0 || y1 <= y0) return result;

  const cv::Rect roi(x0, y0, x1 - x0, y1 - y0);
  const cv::Mat patch = image(roi).clone();
  cv::Mat hsv;
  cv::Mat mask;
  cv::cvtColor(patch, hsv, cv::COLOR_BGR2HSV);
  cv::inRange(hsv, hsv_lower, hsv_upper, mask);
  cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 1);
  cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 1);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  if (!contours.empty()) {
    const auto contour_it = std::max_element(
      contours.begin(), contours.end(),
      [](const auto & left, const auto & right) {
        return cv::contourArea(left) < cv::contourArea(right);
      });
    const auto moments = cv::moments(*contour_it);
    if (moments.m00 > 1e-5) {
      result.center = cv::Point2d(
        moments.m10 / moments.m00 + roi.x, moments.m01 / moments.m00 + roi.y);
    }
  }

  return result;
}

cv::Mat make_display(
  const cv::Mat & image, const TargetObservation & observation, double yaw_offset,
  const std::string & name, const cv::Size & display_size)
{
  cv::Mat display;
  cv::resize(image, display, display_size);
  const double scale_x = static_cast<double>(display.cols) / image.cols;
  const double scale_y = static_cast<double>(display.rows) / image.rows;

  cv::line(
    display, cv::Point(display.cols / 2, 0), cv::Point(display.cols / 2, display.rows),
    cv::Scalar(255, 0, 0), 1);

  if (observation.valid) {
    const cv::Rect2d scaled_box(
      observation.box.x * scale_x, observation.box.y * scale_y,
      observation.box.width * scale_x, observation.box.height * scale_y);
    cv::rectangle(display, scaled_box, cv::Scalar(0, 0, 255), 2);
    cv::circle(
      display,
      cv::Point(
        cvRound(observation.center.x * scale_x), cvRound(observation.center.y * scale_y)),
      5, cv::Scalar(255, 255, 0), -1);
  }

  std::ostringstream text;
  text << name << " offset=" << std::fixed << std::setprecision(1) << yaw_offset;
  cv::putText(
    display, text.str(), cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.65,
    cv::Scalar(0, 255, 255), 2);
  return display;
}

template <typename T>
T optional_config(const YAML::Node & yaml, const std::string & key, const T & fallback)
{
  return yaml[key] ? yaml[key].as<T>() : fallback;
}

}  // namespace

int main()
{
  const std::string config_path = "configs/test.yaml";
  const auto yaml = tools::load(config_path);

  const auto network_size_values =
    optional_config<std::vector<int>>(yaml, "network_input_size", {640, 480});
  if (network_size_values.size() != 2 || network_size_values[0] <= 0 || network_size_values[1] <= 0) {
    tools::logger()->error("network_input_size must contain positive [width, height].");
    return 1;
  }
  const cv::Size2d network_size(network_size_values[0], network_size_values[1]);

  const auto green_lower_values =
    optional_config<std::vector<int>>(yaml, "green_lower", {35, 100, 100});
  const auto green_upper_values =
    optional_config<std::vector<int>>(yaml, "green_upper", {85, 255, 255});
  if (green_lower_values.size() != 3 || green_upper_values.size() != 3) {
    tools::logger()->error("green_lower and green_upper must contain [H, S, V].");
    return 1;
  }
  const cv::Scalar hsv_lower(
    green_lower_values[0], green_lower_values[1], green_lower_values[2]);
  const cv::Scalar hsv_upper(
    green_upper_values[0], green_upper_values[1], green_upper_values[2]);

  const auto sync_queue_size = optional_config<int>(yaml, "sync_queue_size", 8);
  const auto sync_max_delta_ms = optional_config<double>(yaml, "sync_max_delta_ms", 8.0);
  if (sync_queue_size <= 0 || sync_max_delta_ms <= 0.0) {
    tools::logger()->error("sync_queue_size and sync_max_delta_ms must be positive.");
    return 1;
  }
  const auto sync_max_delta = std::chrono::duration_cast<tools::StereoFrameQueue::Duration>(
    std::chrono::duration<double, std::milli>(sync_max_delta_ms));
  if (sync_max_delta <= tools::StereoFrameQueue::Duration::zero()) {
    tools::logger()->error("sync_max_delta_ms is too small for the steady clock resolution.");
    return 1;
  }
  tools::StereoFrameQueue stereo_queue(static_cast<std::size_t>(sync_queue_size), sync_max_delta);

  const auto display_enabled = optional_config<bool>(yaml, "display_enabled", true);
  const auto display_size_values =
    optional_config<std::vector<int>>(yaml, "display_size", {640, 480});
  const cv::Size display_size(
    display_size_values.size() == 2 ? display_size_values[0] : 640,
    display_size_values.size() == 2 ? display_size_values[1] : 480);
  if (display_size.width <= 0 || display_size.height <= 0) {
    tools::logger()->error("display_size must contain positive [width, height].");
    return 1;
  }

  io::Camera left_camera(config_path, "left_camera");
  io::Camera right_camera(config_path, "right_camera");
  LightDetect left_detector(config_path);
  LightDetect right_detector(config_path);
  StereoRangeEstimator range_estimator(config_path);
  io::Gimbal gimbal(config_path);
  tools::Exiter exiter;

  std::atomic<bool> capture_quit{false};

  // These are the two application-level acquisition threads. Each camera
  // driver also owns its SDK capture thread; these threads perform the
  // common-clock timestamping and feed the stereo synchronizer.
  auto capture_loop = [&](io::Camera & camera, tools::StereoCameraSide side, const char * name) {
    tools::logger()->info("[{}] acquisition thread started.", name);
    while (!capture_quit.load() && !exiter.exit()) {
      cv::Mat image;
      std::chrono::steady_clock::time_point captured_timestamp;
      if (!camera.read(image, captured_timestamp)) {
        tools::logger()->warn("[{}] camera read stopped.", name);
        break;
      }
      if (image.empty()) continue;

      // Both HikRobot instances use steady_clock, so this timestamp is in one
      // common time domain and represents the frame acquisition time rather
      // than the later time at which the processing thread happens to read it.
      stereo_queue.push(side, {std::move(image), captured_timestamp});
    }
    capture_quit.store(true);
    stereo_queue.stop();
    tools::logger()->info("[{}] acquisition thread stopped.", name);
  };

  std::thread left_capture_thread(
    capture_loop, std::ref(left_camera), tools::StereoCameraSide::LEFT, "left");
  std::thread right_capture_thread(
    capture_loop, std::ref(right_camera), tools::StereoCameraSide::RIGHT, "right");

  bool left_detector_startup = true;
  bool right_detector_startup = true;

  if (display_enabled) cv::namedWindow("Stereo Dart Guide", cv::WINDOW_AUTOSIZE);

  // The main thread is the stereo processing thread: pair frames, detect the
  // guide light in both views, calculate steering data, and triangulate range.
  while (!exiter.exit()) {
    tools::StereoFramePair frame_pair;
    if (!stereo_queue.pop(frame_pair)) break;

    auto left_observation = detect_target(
      left_detector, frame_pair.left.image, network_size, left_detector_startup,
      hsv_lower, hsv_upper);
    auto right_observation = detect_target(
      right_detector, frame_pair.right.image, network_size, right_detector_startup,
      hsv_lower, hsv_upper);

    io::VisionToGimbal send_data;
    send_data.yaw_offset = 0.0f;
    send_data.distance_m = -1.0f;

    if (left_observation.valid && right_observation.valid) {
      const double left_yaw_offset =
        left_observation.center.x - range_estimator.left_principal_x();
      const double right_yaw_offset =
        right_observation.center.x - range_estimator.right_principal_x();
      const double yaw_offset = (left_yaw_offset + right_yaw_offset) * 0.5;
      const auto range = range_estimator.estimate(
        left_observation.center, right_observation.center);

      // Bit 0: both guide lights detected. Bit 1: range passed validity checks.
      send_data.status = 0x01;
      if (range.valid) {
        send_data.status |= 0x02;
        send_data.distance_m = static_cast<float>(range.range_m);
      }
      send_data.yaw_offset = static_cast<float>(yaw_offset);

      std::printf(
        "L %.2f R %.2f | average %.2f px | range %.3f m%s\n",
        left_yaw_offset, right_yaw_offset, yaw_offset,
        range.range_m, range.valid ? "" : " (invalid)");
    }

    gimbal.send(send_data);

    if (display_enabled) {
      const double left_yaw_offset = left_observation.valid
        ? left_observation.center.x - range_estimator.left_principal_x()
        : 0.0;
      const double right_yaw_offset = right_observation.valid
        ? right_observation.center.x - range_estimator.right_principal_x()
        : 0.0;
      const auto left_display = make_display(
        frame_pair.left.image, left_observation, left_yaw_offset, "LEFT", display_size);
      const auto right_display = make_display(
        frame_pair.right.image, right_observation, right_yaw_offset, "RIGHT", display_size);
      cv::Mat stereo_display;
      cv::hconcat(left_display, right_display, stereo_display);
      cv::imshow("Stereo Dart Guide", stereo_display);
      if (cv::waitKey(1) == 'q') break;
    }
  }

  capture_quit.store(true);
  left_camera.stop();
  right_camera.stop();
  stereo_queue.stop();
  if (left_capture_thread.joinable()) left_capture_thread.join();
  if (right_capture_thread.joinable()) right_capture_thread.join();

  if (display_enabled) cv::destroyAllWindows();
  return 0;
}
