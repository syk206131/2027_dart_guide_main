#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/dart/dart_calculator.hpp"
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
  double area = 0.0;
  cv::Rect2d box;
  cv::Point2d center;
};

TargetObservation detect_target(
  LightDetect & detector, cv::Mat & image,
  const cv::Scalar & hsv_lower, const cv::Scalar & hsv_upper,
  double dual_target_area_threshold, bool aim_at_outpost)
{
  auto lights = detector.detect(image);
  if (lights.empty()) return {};

  // When one camera sees both guide lights, lock the one matching the target
  // selected by the gimbal: the base is farther away (26.4m vs 17.2m), so its
  // light appears SMALLER, while the outpost light appears LARGER. Only lights
  // above dual_target_area_threshold count as a real second target, so noise
  // blobs cannot trigger the switch.
  const LightDetect::Light * best = nullptr;
  std::vector<const LightDetect::Light *> strong_lights;
  for (const auto & light : lights) {
    if (light.area >= dual_target_area_threshold) strong_lights.push_back(&light);
  }
  if (strong_lights.size() >= 2) {
    best = aim_at_outpost
      ? *std::max_element(
          strong_lights.begin(), strong_lights.end(),
          [](const auto * a, const auto * b) { return a->area < b->area; })
      : *std::min_element(
          strong_lights.begin(), strong_lights.end(),
          [](const auto * a, const auto * b) { return a->area < b->area; });
  } else {
    best = &*std::max_element(
      lights.begin(), lights.end(),
      [](const auto & left, const auto & right) { return left.score < right.score; });
  }

  TargetObservation result;
  result.valid = true;
  result.score = best->score;
  result.box = best->box;
  result.center = best->center_point;
  result.area = best->area;

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

  // Dual-target decision thresholds (outpost ~17.25m on the left, base
  // ~26.45m on the right; the dart aims at the target selected by the gimbal).
  const auto dual_target_area_threshold =
    optional_config<double>(yaml, "dual_target_area_threshold", 30.0);
  const auto stereo_area_diff_threshold =
    optional_config<double>(yaml, "stereo_area_diff_threshold", 100.0);
  const auto outpost_range_m = optional_config<double>(yaml, "outpost_range_m", 17.25);
  const auto base_range_m = optional_config<double>(yaml, "base_range_m", 26.45);

  // 瞄准判定（视觉侧）：|yaw偏移+roll补偿| 连续 aim_consecutive_frames 帧小于
  // aim_deadband_px 时判定瞄准完成（control_state=0）
  const auto aim_deadband_px = optional_config<double>(yaml, "aim_deadband_px", 3.0);
  const auto aim_consecutive_frames =
    optional_config<int>(yaml, "aim_consecutive_frames", 25);
  if (aim_deadband_px <= 0.0 || aim_consecutive_frames <= 0) {
    tools::logger()->error("aim_deadband_px and aim_consecutive_frames must be positive.");
    return 1;
  }

  io::Camera left_camera(config_path, "left_camera");
  io::Camera right_camera(config_path, "right_camera");
  LightDetect left_detector(config_path);
  LightDetect right_detector(config_path);
  StereoRangeEstimator range_estimator(config_path);
  DartCalculator dart_calculator(config_path);
  // 临时注释：串口通信未联调，暂不连接电控（恢复时取消本行及下方各处"临时"注释）
  // io::Gimbal gimbal(config_path);
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

  if (display_enabled) cv::namedWindow("Stereo Dart Guide", cv::WINDOW_AUTOSIZE);

  // Processed-pair frame rate, updated once per second.
  //显示帧率
  std::size_t fps_frame_count = 0;
  double fps_value = 0.0;
  auto fps_last = std::chrono::steady_clock::now();
  //显示帧率

  // The main thread is the stereo processing thread: pair frames, detect the
  // guide light in both views, calculate steering data, and triangulate range.
  // 瞄准状态机：连续 aim_consecutive_frames 帧 |补偿后 yaw 偏移| < aim_deadband_px
  // 时判定瞄准完成。任何搜索/丢目标/空闲帧都会清零计数。
  int aim_consecutive_count = 0;
  while (!exiter.exit()) {
    tools::StereoFramePair frame_pair;
    if (!stereo_queue.pop(frame_pair)) break;

    //显示帧率
    ++fps_frame_count;
    const auto fps_now = std::chrono::steady_clock::now();
    const double fps_elapsed =
      std::chrono::duration<double>(fps_now - fps_last).count();
    if (fps_elapsed >= 1.0) {
      fps_value = static_cast<double>(fps_frame_count) / fps_elapsed;
      fps_frame_count = 0;
      fps_last = fps_now;
      std::printf("[fps] %.1f\n", fps_value);
    }
    //显示帧率

    io::VisionToGimbal send_data;  // 默认 control_state=0, energy_j=0

    // 电控模式门控：仅 RUNNING（0x01，舱门开启）时自瞄；IDLE/ERROR 保持停机
    // 临时：无电控连接，强制进入自瞄（原：gimbal.mode() == io::GimbalMode::RUNNING）
    const bool auto_aim_enabled = true;

    // 目标选择（电控下发，0=前哨站 / 1=基地）：前哨站约 17.25m、位于视野偏左，
    // 基地约 26.45m、位于视野偏右。目标决定：同屏双灯锁大/锁小、锁定错误目标时
    // 的搜索方向、丢目标时的扫描方向；距离/高度/能量拟合均为距离的函数，两目标
    // 通用无需区分。
    // 临时：无电控连接，目标占位为基地（恢复通信后改为 gimbal.target() == 0）
    const bool aim_at_outpost = false;
    // 搜错目标/丢目标时的转向：前哨站在左（1）、基地在右（2）
    const uint8_t search_state = aim_at_outpost ? 1 : 2;
    const char * search_dir = aim_at_outpost ? "LEFT" : "RIGHT";
    const char * target_name = aim_at_outpost ? "outpost" : "base";

    auto left_observation = detect_target(
      left_detector, frame_pair.left.image, hsv_lower, hsv_upper,
      dual_target_area_threshold, aim_at_outpost);
    auto right_observation = detect_target(
      right_detector, frame_pair.right.image, hsv_lower, hsv_upper,
      dual_target_area_threshold, aim_at_outpost);

    if (!auto_aim_enabled) {
      aim_consecutive_count = 0;
    }
    else if (left_observation.valid && right_observation.valid) {
      const double left_yaw_offset =
        left_observation.center.x - range_estimator.left_principal_x();
      const double right_yaw_offset =
        right_observation.center.x - range_estimator.right_principal_x();
      const double yaw_offset = (left_yaw_offset + right_yaw_offset) * 0.5;
      const auto range = range_estimator.estimate(
        left_observation.center, right_observation.center);

      // 左右面积差过大说明锁定了不同目标（一侧前哨/一侧基地），朝目标侧搜索
      const bool area_mismatch =
        std::fabs(left_observation.area - right_observation.area) >
        stereo_area_diff_threshold;
      // 测距更接近非目标工况点说明锁定了错误目标，朝目标侧搜索
      const double target_range_m = aim_at_outpost ? outpost_range_m : base_range_m;
      const double other_range_m = aim_at_outpost ? base_range_m : outpost_range_m;
      const bool locked_wrong_target =
        range.valid &&
        std::fabs(range.range_m - target_range_m) >
          std::fabs(range.range_m - other_range_m);

      if (area_mismatch) {
        aim_consecutive_count = 0;
        send_data.control_state = search_state;
        std::printf(
          "target mismatch: left area %.1f vs right area %.1f | steer %s for %s\n",
          left_observation.area, right_observation.area, search_dir, target_name);
      }
      else if (locked_wrong_target) {
        aim_consecutive_count = 0;
        send_data.control_state = search_state;
        std::printf(
          "locked on wrong target: range %.3f m (raw %.3f) | steer %s for %s\n",
          range.range_m, range.raw_range_m, search_dir, target_name);
      }
      else {
        // DartCalculator：距离修正、高度解算、roll→yaw 瞄准点、能量计算
        // 临时：无电控数据，roll/dart_index 用占位值（原：gimbal.roll_rad(), gimbal.dart_index()）
        const auto comp = dart_calculator.compute(
          range, left_observation.center, right_observation.center, 0.0f, 0);

        send_data.energy_j = comp.energy_j;

        // 瞄准判定：瞄准点 = c0 + c1*roll + c2*roll^2（像素，DartCalculator 给出）。
        // 瞄准误差 = 当前 yaw 偏移 - 瞄准点；|误差| 连续 N 帧 < 死区 → control_state=0。
        // 例：roll 顺时针 2°，弹着偏右，需云台向左补 → 瞄准点为正值（目标需偏右），
        // yaw_offset 收敛到该正值即瞄准完成。
        const double aim_error = yaw_offset - comp.yaw_offset_px;
        if (std::fabs(aim_error) < aim_deadband_px) {
          if (++aim_consecutive_count >= aim_consecutive_frames) {
            send_data.control_state = 0;  // 瞄准完成
          }
          else {
            // 死区内但未收敛：给方向，电控可减速精调
            send_data.control_state = (aim_error > 0) ? 2 : 1;
          }
        }
        else {
          aim_consecutive_count = 0;
          send_data.control_state = (aim_error > 0) ? 2 : 1;
        }

        std::printf(
          "L %.2f R %.2f | avg %.2f err %.2f | roll %.3f dart %u | D %.3f H %.3f E %.2f J | "
          "aim %d/%d -> state %u%s\n",
          left_yaw_offset, right_yaw_offset, yaw_offset, aim_error,
          0.0, 0,  // 临时：原 gimbal.roll_rad() / gimbal.dart_index()
          static_cast<double>(comp.distance_m), static_cast<double>(comp.height_m),
          static_cast<double>(comp.energy_j), aim_consecutive_count,
          aim_consecutive_frames, send_data.control_state,
          comp.valid ? "" : " (range invalid)");
      }
    }
    else if (left_observation.valid || right_observation.valid) {
      // 单路锁定：目标在该相机视场边缘，朝对应方向粗对准，不计能量
      aim_consecutive_count = 0;
      send_data.control_state = left_observation.valid ? 1 : 2;
      std::printf(
        "single-side target: %s camera only | steer %s\n",
        left_observation.valid ? "left" : "right",
        left_observation.valid ? "LEFT" : "RIGHT");
    }
    else {
      // 双路均未锁定：朝目标侧扫描搜索（前哨站在左、基地在右），收敛计数清零
      aim_consecutive_count = 0;
      send_data.control_state = search_state;
    }

    // 临时注释：串口通信未联调（原：gimbal.send(send_data);）

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
      //显示帧率
      std::ostringstream fps_text;
      fps_text << "FPS: " << std::fixed << std::setprecision(1) << fps_value;
      cv::putText(
        stereo_display, fps_text.str(), cv::Point(10, stereo_display.rows - 10),
        cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(0, 255, 255), 2);
      //显示帧率
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
