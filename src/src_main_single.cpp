#include "io/camera.hpp"
#include "tasks/light_detect/Detect.hpp"
#include "tools/exiter.hpp"
#include "tools/yaml.hpp"

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
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
  LightDetect & detector, cv::Mat & image,
  const cv::Scalar & hsv_lower, const cv::Scalar & hsv_upper)
{
  auto lights = detector.detect(image);
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

// The transmission view shows only the target box, the horizontal pixel
// offset, and the detection confidence.
cv::Mat make_display(
  const cv::Mat & image, const TargetObservation & observation, double yaw_offset,
  const cv::Size & display_size, const std::string & serial_number)
{
  cv::Mat display;
  cv::resize(image, display, display_size);
  const double scale_x = static_cast<double>(display.cols) / image.cols;
  const double scale_y = static_cast<double>(display.rows) / image.rows;

  if (observation.valid) {
    const cv::Rect2d scaled_box(
      observation.box.x * scale_x, observation.box.y * scale_y,
      observation.box.width * scale_x, observation.box.height * scale_y);
    cv::rectangle(display, scaled_box, cv::Scalar(0, 0, 255), 2);
  }

  std::ostringstream text;
  text << "offset=" << std::fixed << std::setprecision(1) << yaw_offset
       << "px conf=" << std::setprecision(2) << observation.score;
  cv::putText(
    display, text.str(), cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.65,
    cv::Scalar(0, 255, 255), 2);

  // [SERIAL-DISPLAY] 画面右下角显示相机序列号，方便确认当前使用的是哪台相机
  cv::putText(
    display, serial_number.empty() ? "SN:(unknown)" : "SN:" + serial_number,
    cv::Point(display.cols - 150, display.rows - 10),
    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

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

  // Horizontal offset reference point: same fallback rule as
  // StereoRangeEstimator (left_camera_matrix first, then camera_matrix).
  const auto left_matrix =
    optional_config<std::vector<double>>(yaml, "left_camera_matrix", {});
  const auto base_matrix = optional_config<std::vector<double>>(yaml, "camera_matrix", {});
  const auto & camera_matrix = left_matrix.size() == 9 ? left_matrix : base_matrix;
  if (camera_matrix.size() != 9) {
    tools::logger()->error("camera_matrix or left_camera_matrix must contain 9 values.");
    return 1;
  }
  const double principal_x = camera_matrix[2];

  io::Camera camera(config_path, "right_camera");
  LightDetect detector(config_path);
  tools::Exiter exiter;

  // [SERIAL-DISPLAY] 从驱动读取实际选中设备的序列号（SDK 上报，与 yaml 配置无关）。
  // 序列号由守护线程在 capture_start 中记录，构造完成时可能尚未就绪（USB 枚举/复位耗时），
  // 因此声明为空串，在主循环里持续刷新直到拿到。
  std::string serial_number;

  if (display_enabled) cv::namedWindow("Single Dart Guide", cv::WINDOW_AUTOSIZE);

  while (!exiter.exit()) {
    cv::Mat image;
    std::chrono::steady_clock::time_point captured_timestamp;
    if (!camera.read(image, captured_timestamp)) {
      tools::logger()->warn("camera read stopped.");
      break;
    }
    if (image.empty()) continue;

    auto observation = detect_target(detector, image, hsv_lower, hsv_upper);

    const double yaw_offset = observation.valid ? observation.center.x - principal_x : 0.0;

    std::printf(
      "offset %.2f px | score %.3f%s\n",
      yaw_offset, observation.score, observation.valid ? "" : " (no target)");

    if (display_enabled) {
      if (serial_number.empty()) serial_number = camera.serial();  // [SERIAL-DISPLAY]
      const auto display = make_display(image, observation, yaw_offset, display_size, serial_number);
      cv::imshow("Single Dart Guide", display);
      if (cv::waitKey(1) == 'q') break;
    }
  }

  camera.stop();
  if (display_enabled) cv::destroyAllWindows();
  return 0;
}
