// 相机光轴安装对准工具（远点对准法）。
//
// 原理：若两相机光轴严格平行，同一目标在两幅图像中的像素坐标只差一个由基线
// 决定的水平视差 fx*b/D，垂直方向完全相等。因此用绿色引导灯做远点目标，实时
// 比较两相机测得的亚像素质心：
//   Δu残差 = (u右 − u左) − 预期视差   → 水平光轴夹角（关键，1px≈0.024°@fx≈2380）
//   Δv     =  v右 − v左               → 垂直光轴夹角（对测距影响小，几 px 内即可）
// 预期视差 = −disparity_sign · fx均值 · baseline_m / D（符号约定与主程序一致：
// u左−u右>0）。D 用激光测距实测，±0.3m 精度即可（20m 处视差≈14px，0.3m 误差
// 只引入≈0.2px 补偿残差）。
//
// 操作：以左相机为基准调整右相机支架，把 Δu残差 的滚动均值压到 ±0.5px 以内后
// 锁死；0.5px 残差对应的 raw 距离非线性 <3%，可被距离二次拟合吸收。
//
// 用法：
//   align_tool [--config <yaml>] [--distance <m>] [--window <帧数>] [--threshold <px>]
//     空格   打印当前滚动统计（调整到位后留档）
//     r      清零重新统计（每次动过支架后按一次）
//     q/ESC  退出
//
// 说明：OpenCV putText 不支持中文，屏幕 HUD 用英文；控制台提示与注释为中文。

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <deque>
#include <iostream>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "tasks/light_detect/Detect.hpp"
#include "tools/exiter.hpp"
#include "tools/stereo_frame_queue.hpp"
#include "tools/yaml.hpp"

namespace
{
constexpr double kRadToDeg = 57.295779513082320876;

struct AlignOptions
{
  std::string config_path = "configs/test.yaml";
  double distance_m = 20.0;   // 引导灯距离（激光实测，米），视差补偿用
  int window = 150;           // 滚动统计窗口长度（帧对）
  double threshold_px = 0.5;  // |均值 Δu残差| 合格判据
};

void print_usage(const char * program)
{
  std::cout
    << "Usage: " << program
    << " [--config <yaml>] [--distance <m>] [--window <n>] [--threshold <px>]\n"
    << "  远点对准：引导灯放 20m 以外，激光测距后用 --distance 传入，\n"
    << "  以左相机为基准调右相机支架，|均值 Δu残差| < 0.5px 后锁死。\n"
    << "  SPACE 打印当前统计  r 清零重计  q/ESC 退出\n";
}

// Δu残差/Δv 滚动统计：均值反映装调残差，标准差反映目标抖动与支架刚性。
class RollingStats
{
public:
  explicit RollingStats(std::size_t window) : window_(window == 0 ? 1 : window) {}

  void clear()
  {
    du_.clear();
    dv_.clear();
  }

  void push(double du, double dv)
  {
    du_.push_back(du);
    dv_.push_back(dv);
    while (du_.size() > window_) {
      du_.pop_front();
      dv_.pop_front();
    }
  }

  std::size_t size() const { return du_.size(); }

  void summary(double & mean_du, double & std_du, double & mean_dv, double & std_dv) const
  {
    mean_du = mean_of(du_);
    mean_dv = mean_of(dv_);
    std_du = std_of(du_, mean_du);
    std_dv = std_of(dv_, mean_dv);
  }

private:
  static double mean_of(const std::deque<double> & values)
  {
    return std::accumulate(values.begin(), values.end(), 0.0) /
           static_cast<double>(values.size());
  }

  static double std_of(const std::deque<double> & values, double mean)
  {
    double acc = 0.0;
    for (double x : values) acc += (x - mean) * (x - mean);
    return std::sqrt(acc / static_cast<double>(values.size()));
  }

  std::size_t window_;
  std::deque<double> du_;
  std::deque<double> dv_;
};

// 在缩放后的显示图上叠加：本机质心绿十字 + 另一相机质心经视差修正后投影过来
// 的红十字。光轴平行时红十字应正好压在绿十字上。
cv::Mat draw_view(
  const cv::Mat & frame, const LightDetect::Light * own, const LightDetect::Light * other,
  double expected_du, const char * tag)
{
  cv::Mat display;
  cv::resize(frame, display, cv::Size(640, 480));
  const double sx = 640.0 / frame.cols;
  const double sy = 480.0 / frame.rows;

  auto draw_cross = [&](cv::Point2d px, const cv::Scalar & color) {
    const cv::Point p(cvRound(px.x * sx), cvRound(px.y * sy));
    cv::drawMarker(display, p, color, cv::MARKER_CROSS, 24, 2);
  };

  const bool detected = own != nullptr;
  if (detected) draw_cross(own->center_point, cv::Scalar(0, 255, 0));
  if (other != nullptr) {
    // 对方质心映射到本机坐标：u本机 = u对方 ∓ expected_du
    const double mapped_u =
      tag[0] == 'L' ? other->center_point.x - expected_du : other->center_point.x + expected_du;
    draw_cross(cv::Point2d(mapped_u, other->center_point.y), cv::Scalar(0, 0, 255));
  }

  cv::putText(
    display, tag, cv::Point(10, 28), cv::FONT_HERSHEY_SIMPLEX, 0.8,
    detected ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);
  cv::putText(
    display, detected ? "light detected" : "NO light", cv::Point(10, 455),
    cv::FONT_HERSHEY_SIMPLEX, 0.6, detected ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255),
    2);
  return display;
}
}  // namespace

int main(int argc, char ** argv)
{
  AlignOptions options;

  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--config") {
      if (index + 1 >= argc) { print_usage(argv[0]); return 1; }
      options.config_path = argv[++index];
    } else if (argument == "--distance") {
      if (index + 1 >= argc) { print_usage(argv[0]); return 1; }
      options.distance_m = std::stod(argv[++index]);
    } else if (argument == "--window") {
      if (index + 1 >= argc) { print_usage(argv[0]); return 1; }
      options.window = std::stoi(argv[++index]);
    } else if (argument == "--threshold") {
      if (index + 1 >= argc) { print_usage(argv[0]); return 1; }
      options.threshold_px = std::stod(argv[++index]);
    } else {
      print_usage(argv[0]);
      return 1;
    }
  }
  if (options.distance_m < 5.0) {
    std::cerr << "[对准] 目标太近（" << options.distance_m
              << "m），视差补偿对 D 误差敏感，请放 15m 以外。\n";
    return 1;
  }

  const auto yaml = tools::load(options.config_path);
  const auto sync_max_delta_ms =
    yaml["sync_max_delta_ms"] ? yaml["sync_max_delta_ms"].as<double>() : 8.0;
  const auto sync_queue_size =
    yaml["sync_queue_size"] ? yaml["sync_queue_size"].as<int>() : 8;
  const auto sync_max_delta = std::chrono::duration_cast<tools::StereoFrameQueue::Duration>(
    std::chrono::duration<double, std::milli>(sync_max_delta_ms));

  // 读取左右相机 fx 与基线，算预期视差（符号约定与主程序一致：u左−u右>0）
  double fx_left = 2380.0, fx_right = 2380.0, fy_avg = 2380.0;
  if (yaml["left_camera_matrix"] && yaml["right_camera_matrix"]) {
    const auto ml = yaml["left_camera_matrix"].as<std::vector<double>>();
    const auto mr = yaml["right_camera_matrix"].as<std::vector<double>>();
    if (ml.size() == 9 && mr.size() == 9) {
      fx_left = ml[0];
      fx_right = mr[0];
      fy_avg = 0.5 * (ml[4] + mr[4]);
    }
  }
  const double fx_avg = 0.5 * (fx_left + fx_right);
  const double baseline_m = yaml["baseline_m"] ? yaml["baseline_m"].as<double>() : 0.12;
  const double disparity_sign =
    yaml["disparity_sign"] ? yaml["disparity_sign"].as<double>() : 1.0;
  // u左−u右 = sign·fx·b/D（主程序视差定义），故 Δu = u右−u左 的预期值取负号
  const double expected_du = -disparity_sign * fx_avg * baseline_m / options.distance_m;

  std::cout << "[对准] 目标距离 D=" << options.distance_m << "m，fx均值=" << fx_avg
            << "px，基线=" << baseline_m << "m\n"
            << "[对准] 预期视差 Δu=u右-u左 ≈ " << expected_du << "px（光轴平行时）\n"
            << "[对准] 操作：调右相机支架使 Δu残差 滚动均值 |x| < " << options.threshold_px
            << "px；Δv 几 px 内即可。调完按空格留档，r 清零重计。\n";

  io::Camera left_camera(options.config_path, "left_camera");
  io::Camera right_camera(options.config_path, "right_camera");
  LightDetect detector(options.config_path);
  tools::Exiter exiter;

  tools::StereoFrameQueue stereo_queue(
    static_cast<std::size_t>(sync_queue_size), sync_max_delta);
  std::atomic<bool> capture_quit{false};

  auto capture_loop = [&](io::Camera & camera, tools::StereoCameraSide side) {
    while (!capture_quit.load() && !exiter.exit()) {
      cv::Mat image;
      std::chrono::steady_clock::time_point timestamp;
      if (!camera.read(image, timestamp)) break;
      if (image.empty()) continue;
      stereo_queue.push(side, {std::move(image), timestamp});
    }
    capture_quit.store(true);
    stereo_queue.stop();
  };

  std::thread left_thread(capture_loop, std::ref(left_camera), tools::StereoCameraSide::LEFT);
  std::thread right_thread(
    capture_loop, std::ref(right_camera), tools::StereoCameraSide::RIGHT);

  RollingStats stats(static_cast<std::size_t>(options.window));
  constexpr std::size_t kMinSamples = 60;  // 滚动样本达到该数量才给合格判定

  cv::namedWindow("Optical Axis Alignment", cv::WINDOW_AUTOSIZE);
  std::cout << "[对准] 窗口已启动，SPACE=打印统计  r=清零  q=退出\n";

  while (!exiter.exit()) {
    tools::StereoFramePair pair;
    if (!stereo_queue.pop(pair)) break;

    const auto left_lights = detector.detect(pair.left.image);
    const auto right_lights = detector.detect(pair.right.image);
    // 圆度最高的作为目标（远点场景通常只有一个引导灯）
    const LightDetect::Light * left_light = left_lights.empty() ? nullptr : &left_lights[0];
    const LightDetect::Light * right_light =
      right_lights.empty() ? nullptr : &right_lights[0];

    double du_residual = 0.0, dv = 0.0;
    const bool both = left_light != nullptr && right_light != nullptr;
    if (both) {
      const double du_raw = right_light->center_point.x - left_light->center_point.x;
      dv = right_light->center_point.y - left_light->center_point.y;
      du_residual = du_raw - expected_du;
      stats.push(du_residual, dv);
    }

    const auto left_display =
      draw_view(pair.left.image, left_light, right_light, expected_du, "LEFT");
    const auto right_display =
      draw_view(pair.right.image, right_light, left_light, expected_du, "RIGHT");
    cv::Mat canvas;
    cv::hconcat(left_display, right_display, canvas);

    // HUD 第一行：瞬时值
    const std::string hud1 = both
      ? cv::format(
          "resid du=%+.2fpx  dv=%+.2fpx  (raw du=%+.1f, expected %+.1f)", du_residual, dv,
          du_residual + expected_du, expected_du)
      : "target missing in one/both views -- check light distance and HSV";
    cv::putText(
      canvas, hud1, cv::Point(12, 424), cv::FONT_HERSHEY_SIMPLEX, 0.55,
      both ? cv::Scalar(0, 255, 255) : cv::Scalar(0, 0, 255), 1);

    // HUD 第二行：滚动统计 + 合格判定
    std::string hud2 = cv::format("collecting... n=%zu", stats.size());
    cv::Scalar hud2_color(0, 255, 255);
    if (stats.size() >= kMinSamples) {
      double mean_du, std_du, mean_dv, std_dv;
      stats.summary(mean_du, std_du, mean_dv, std_dv);
      const bool aligned = std::abs(mean_du) < options.threshold_px;
      hud2 = cv::format(
        "mean du=%+.3f+/-%.3f  mean dv=%+.3f+/-%.3f (n=%zu)  %s", mean_du, std_du, mean_dv,
        std_dv, stats.size(),
        aligned ? "YAW ALIGNED - lock the mount" : "NOT aligned - adjust RIGHT mount");
      hud2_color = aligned ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
    }
    cv::putText(canvas, hud2, cv::Point(12, 448), cv::FONT_HERSHEY_SIMPLEX, 0.55, hud2_color, 1);

    // HUD 第三行：帧同步差 + 按键提示
    const double delta_ms =
      std::chrono::duration<double, std::milli>(pair.left.timestamp - pair.right.timestamp)
        .count();
    cv::putText(
      canvas,
      cv::format("frame delta: %+.2f ms  |  SPACE=log  r=reset  q=quit", delta_ms),
      cv::Point(12, 472), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 255), 1);

    cv::imshow("Optical Axis Alignment", canvas);

    const int key = cv::waitKey(1) & 0xFF;
    if (key == 'q' || key == 27) break;
    if (key == 'r') {
      stats.clear();
      std::cout << "[对准] 统计已清零。\n";
    }
    if (key == ' ') {
      if (stats.size() < kMinSamples) {
        std::cout << "[对准] 样本不足（" << stats.size() << "/" << kMinSamples
                  << "），稍候再记录。\n";
      } else {
        double mean_du, std_du, mean_dv, std_dv;
        stats.summary(mean_du, std_du, mean_dv, std_dv);
        std::cout << cv::format(
                       "[对准记录] n=%zu  du残差 均值=%+.3fpx 标准差=%.3fpx | "
                       "dv 均值=%+.3fpx 标准差=%.3fpx | 折合夹角 du=%+.4f° dv=%+.4f°",
                       stats.size(), mean_du, std_du, mean_dv, std_dv,
                       mean_du / fx_avg * kRadToDeg, mean_dv / fy_avg * kRadToDeg)
                  << std::endl;
      }
    }
  }

  capture_quit.store(true);
  left_camera.stop();
  right_camera.stop();
  stereo_queue.stop();
  if (left_thread.joinable()) left_thread.join();
  if (right_thread.joinable()) right_thread.join();

  std::cout << "[对准] 已退出。\n";
  return 0;
}
