#include "calibration.hpp"

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

double reprojection_error(
  const std::vector<cv::Point3f> & object_points, const std::vector<cv::Point2f> & image_points,
  const cv::Mat & rotation, const cv::Mat & translation, const cv::Mat & camera_matrix,
  const cv::Mat & distortion_coeffs)
{
  std::vector<cv::Point2f> projected;
  cv::projectPoints(
    object_points, rotation, translation, camera_matrix, distortion_coeffs, projected);

  double squared_error = 0.0;
  for (std::size_t index = 0; index < image_points.size(); ++index) {
    squared_error += cv::norm(image_points[index] - projected[index]) *
      cv::norm(image_points[index] - projected[index]);
  }
  return std::sqrt(squared_error / image_points.size());
}
}  // namespace

Result calibrate_camera(
  const std::vector<std::string> & image_paths, const Options & options)
{
  Result result;
  result.total_views = image_paths.size();

  if (options.board_size.width <= 0 || options.board_size.height <= 0 ||
      options.square_size_m <= 0.0F) {
    return result;
  }

  const auto object_template = make_object_points(options);
  std::vector<std::vector<cv::Point3f>> object_points;
  std::vector<std::vector<cv::Point2f>> image_points;

  for (const auto & image_path : image_paths) {
    const auto image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
    if (image.empty()) continue;
    if (result.image_size.empty()) result.image_size = image.size();
    if (image.size() != result.image_size) continue;

    std::vector<cv::Point2f> corners;
    const bool found = cv::findChessboardCorners(
      image, options.board_size, corners,
      cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
    if (!found) continue;

    cv::cornerSubPix(
      image, corners, cv::Size(11, 11), cv::Size(-1, -1),
      cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001));
    object_points.push_back(object_template);
    image_points.push_back(std::move(corners));
  }

  result.detected_views = image_points.size();
  if (result.detected_views < 3 || result.image_size.empty()) return result;

  std::vector<cv::Mat> rotations;
  std::vector<cv::Mat> translations;
  result.camera_matrix = cv::Mat::eye(3, 3, CV_64F);
  result.distortion_coeffs = cv::Mat::zeros(1, 5, CV_64F);
  result.rms_error = cv::calibrateCamera(
    object_points, image_points, result.image_size, result.camera_matrix,
    result.distortion_coeffs, rotations, translations);

  result.valid = std::isfinite(result.rms_error);
  return result;
}

bool write_yaml(const std::string & output_path, const std::string & name, const Result & result)
{
  if (!result.valid || name.empty()) return false;

  std::ofstream output(output_path);
  if (!output.is_open()) return false;

  output << std::setprecision(17);
  output << name << "_camera_matrix: [";
  for (int row = 0; row < result.camera_matrix.rows; ++row) {
    for (int column = 0; column < result.camera_matrix.cols; ++column) {
      if (row != 0 || column != 0) output << ", ";
      output << result.camera_matrix.at<double>(row, column);
    }
  }
  output << "]\n";

  output << name << "_distort_coeffs: [";
  for (int index = 0; index < result.distortion_coeffs.total(); ++index) {
    if (index != 0) output << ", ";
    output << result.distortion_coeffs.at<double>(0, index);
  }
  output << "]\n";
  output << name << "_image_width: " << result.image_size.width << '\n';
  output << name << "_image_height: " << result.image_size.height << '\n';
  output << name << "_rms_error: " << result.rms_error << '\n';
  return output.good();
}

}  // namespace calibration