#include "stereo_calibration.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>

namespace calibration
{
namespace
{
std::vector<cv::Point3f> make_object_points(const Options & options)
{
  std::vector<cv::Point3f> points;
  points.reserve(static_cast<std::size_t>(options.board_size.area()));
  for (int row = 0; row < options.board_size.height; ++row) {
    for (int column = 0; column < options.board_size.width; ++column) {
      points.emplace_back(
        column * options.square_size_m, row * options.square_size_m, 0.0F);
    }
  }
  return points;
}

bool detect_corners(
  const std::string & image_path, const cv::Size & board_size, const cv::Size & expected_size,
  std::vector<cv::Point2f> & corners, cv::Size & image_size)
{
  const auto image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
  if (image.empty()) return false;
  if (!expected_size.empty() && image.size() != expected_size) return false;
  image_size = image.size();

  const bool found = cv::findChessboardCorners(
    image, board_size, corners,
    cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
  if (!found) return false;

  cv::cornerSubPix(
    image, corners, cv::Size(11, 11), cv::Size(-1, -1),
    cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001));
  return true;
}

double per_view_rms(
  const std::vector<cv::Point3f> & object_points,
  const std::vector<cv::Point2f> & image_points, const cv::Mat & rotation,
  const cv::Mat & translation, const cv::Mat & camera_matrix, const cv::Mat & distortion)
{
  std::vector<cv::Point2f> projected;
  cv::projectPoints(
    object_points, rotation, translation, camera_matrix, distortion, projected);

  double squared_error = 0.0;
  for (std::size_t i = 0; i < image_points.size(); ++i) {
    const cv::Point2f difference = image_points[i] - projected[i];
    squared_error += difference.dot(difference);
  }
  return std::sqrt(squared_error / static_cast<double>(image_points.size()));
}

// R -> Euler XYZ (radians), used only to report small mounting misalignment.
cv::Vec3d rotation_to_euler_xyz(const cv::Mat & rotation)
{
  const double r00 = rotation.at<double>(0, 0);
  const double r10 = rotation.at<double>(1, 0);
  const double r11 = rotation.at<double>(1, 1);
  const double r12 = rotation.at<double>(1, 2);
  const double r20 = rotation.at<double>(2, 0);
  const double r21 = rotation.at<double>(2, 1);
  const double r22 = rotation.at<double>(2, 2);

  const double sy = std::sqrt(r00 * r00 + r10 * r10);
  double x = 0.0, y = 0.0, z = 0.0;
  if (sy > 1e-6) {
    x = std::atan2(r21, r22);
    y = std::atan2(-r20, sy);
    z = std::atan2(r10, r00);
  } else {  // Gimbal lock fallback.
    x = std::atan2(-r12, r11);
    y = std::atan2(-r20, sy);
  }
  return cv::Vec3d(x, y, z);
}
}  // namespace

StereoResult calibrate_stereo(
  const std::vector<std::string> & image_paths_left,
  const std::vector<std::string> & image_paths_right, const Options & options,
  const cv::Mat & left_camera_matrix, const cv::Mat & left_distortion,
  const cv::Mat & right_camera_matrix, const cv::Mat & right_distortion,
  const bool fix_intrinsic)
{
  StereoResult result;
  result.total_pairs = std::min(image_paths_left.size(), image_paths_right.size());

  if (options.board_size.width <= 0 || options.board_size.height <= 0 ||
      options.square_size_m <= 0.0F || image_paths_left.size() != image_paths_right.size()) {
    return result;
  }

  const auto object_template = make_object_points(options);
  std::vector<std::vector<cv::Point3f>> object_points;
  std::vector<std::vector<cv::Point2f>> left_points;
  std::vector<std::vector<cv::Point2f>> right_points;

  cv::Size expected_size;
  for (std::size_t pair_index = 0; pair_index < result.total_pairs; ++pair_index) {
    std::vector<cv::Point2f> left_corners;
    std::vector<cv::Point2f> right_corners;
    cv::Size left_size;
    cv::Size right_size;
    if (!detect_corners(
          image_paths_left[pair_index], options.board_size, expected_size, left_corners,
          left_size)) {
      continue;
    }
    if (expected_size.empty()) expected_size = left_size;
    if (!detect_corners(
          image_paths_right[pair_index], options.board_size, expected_size, right_corners,
          right_size)) {
      continue;
    }
    object_points.push_back(object_template);
    left_points.push_back(std::move(left_corners));
    right_points.push_back(std::move(right_corners));
  }

  result.detected_pairs = left_points.size();
  result.image_size = expected_size;
  if (result.detected_pairs < 5 || result.image_size.empty()) return result;

  cv::Mat k1 = left_camera_matrix.empty()
    ? cv::Mat::eye(3, 3, CV_64F) : left_camera_matrix.clone();
  cv::Mat d1 = left_distortion.empty()
    ? cv::Mat::zeros(1, 5, CV_64F) : left_distortion.clone();
  cv::Mat k2 = right_camera_matrix.empty()
    ? cv::Mat::eye(3, 3, CV_64F) : right_camera_matrix.clone();
  cv::Mat d2 = right_distortion.empty()
    ? cv::Mat::zeros(1, 5, CV_64F) : right_distortion.clone();

  int flags = cv::CALIB_SAME_FOCAL_LENGTH;
  if (fix_intrinsic) flags |= cv::CALIB_FIX_INTRINSIC;

  cv::Mat essential;
  cv::Mat fundamental;
  result.rms_error = cv::stereoCalibrate(
    object_points, left_points, right_points, k1, d1, k2, d2, result.image_size,
    result.rotation, result.translation, essential, fundamental, flags,
    cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 200, 1e-6));

  if (!std::isfinite(result.rms_error)) return result;

  result.baseline_m = cv::norm(result.translation);

  const cv::Vec3d euler = rotation_to_euler_xyz(result.rotation);
  constexpr double rad_to_deg = 180.0 / CV_PI;
  result.tilt_x_deg = euler[0] * rad_to_deg;
  result.tilt_y_deg = euler[1] * rad_to_deg;
  result.tilt_z_deg = euler[2] * rad_to_deg;

  // Per-camera reprojection RMS. stereoCalibrate does not expose per-view
  // poses; solve the left pose independently, then put it into the right frame
  // with t_right = R * t_left + T.
  double left_squared = 0.0;
  double right_squared = 0.0;
  for (std::size_t view = 0; view < result.detected_pairs; ++view) {
    cv::Mat rvec;
    cv::Mat tvec;
    cv::solvePnP(object_points[view], left_points[view], k1, d1, rvec, tvec);
    const double left_rms = per_view_rms(
      object_points[view], left_points[view], rvec, tvec, k1, d1);
    left_squared += left_rms * left_rms;

    cv::Mat r_left;
    cv::Rodrigues(rvec, r_left);
    const cv::Mat r_right = result.rotation * r_left;
    const cv::Mat t_right = result.rotation * tvec + result.translation;
    const double right_rms = per_view_rms(
      object_points[view], right_points[view], r_right, t_right, k2, d2);
    right_squared += right_rms * right_rms;
  }
  result.left_rms_error = std::sqrt(left_squared / static_cast<double>(result.detected_pairs));
  result.right_rms_error = std::sqrt(right_squared / static_cast<double>(result.detected_pairs));

  // Rectification quality: after stereoRectify corresponding points must lie
  // on the same image row. undistortPoints with R/P outputs rectified pixel
  // coordinates directly.
  cv::Mat r1;
  cv::Mat r2;
  cv::Mat p1;
  cv::Mat p2;
  cv::Mat q;
  cv::stereoRectify(
    k1, d1, k2, d2, result.image_size, result.rotation, result.translation,
    r1, r2, p1, p2, q, 0);

  double epipolar_sum = 0.0;
  double epipolar_max = 0.0;
  std::size_t point_count = 0;
  for (std::size_t view = 0; view < result.detected_pairs; ++view) {
    std::vector<cv::Point2f> rect_left;
    std::vector<cv::Point2f> rect_right;
    cv::undistortPoints(left_points[view], rect_left, k1, d1, r1, p1);
    cv::undistortPoints(right_points[view], rect_right, k2, d2, r2, p2);
    for (std::size_t i = 0; i < rect_left.size(); ++i) {
      const double mismatch = std::abs(rect_left[i].y - rect_right[i].y);
      epipolar_sum += mismatch;
      epipolar_max = std::max(epipolar_max, mismatch);
      ++point_count;
    }
  }
  result.epipolar_mean_px =
    point_count == 0 ? 0.0 : epipolar_sum / static_cast<double>(point_count);
  result.epipolar_max_px = epipolar_max;

  result.valid = true;
  return result;
}

bool write_stereo_yaml(const std::string & output_path, const StereoResult & result)
{
  if (!result.valid) return false;

  std::ofstream output(output_path);
  if (!output.is_open()) return false;

  output << std::setprecision(17);
  output << "# Stereo calibration: X_left = R * X_right + T\n";
  output << "stereo_rotation: [";
  for (int row = 0; row < result.rotation.rows; ++row) {
    for (int column = 0; column < result.rotation.cols; ++column) {
      if (row != 0 || column != 0) output << ", ";
      output << result.rotation.at<double>(row, column);
    }
  }
  output << "]\n";

  output << "stereo_translation: [";
  for (int index = 0; index < result.translation.rows; ++index) {
    if (index != 0) output << ", ";
    output << result.translation.at<double>(index, 0);
  }
  output << "]\n";

  output << "baseline_m: " << result.baseline_m << '\n';
  output << "stereo_tilt_x_deg: " << result.tilt_x_deg << '\n';
  output << "stereo_tilt_y_deg: " << result.tilt_y_deg << '\n';
  output << "stereo_tilt_z_deg: " << result.tilt_z_deg << '\n';
  output << "stereo_rms_error: " << result.rms_error << '\n';
  output << "stereo_left_rms_error: " << result.left_rms_error << '\n';
  output << "stereo_right_rms_error: " << result.right_rms_error << '\n';
  output << "stereo_epipolar_mean_px: " << result.epipolar_mean_px << '\n';
  output << "stereo_epipolar_max_px: " << result.epipolar_max_px << '\n';
  output << "stereo_image_width: " << result.image_size.width << '\n';
  output << "stereo_image_height: " << result.image_size.height << '\n';
  output << "stereo_detected_pairs: " << result.detected_pairs << '\n';
  output << "stereo_total_pairs: " << result.total_pairs << '\n';
  return output.good();
}

}  // namespace calibration
