#include "stereo_range.hpp"

#include <cmath>
#include <stdexcept>
#include <vector>

#include "tools/yaml.hpp"

namespace
{
std::vector<double> zero_distortion()
{
  return {0.0, 0.0, 0.0, 0.0, 0.0};
}
}  // namespace

StereoRangeEstimator::StereoRangeEstimator(const std::string & config_path)
{
  const auto yaml = tools::load(config_path);
  const auto camera_matrix = tools::read<std::vector<double>>(yaml, "camera_matrix");
  const auto distortion_coeffs = tools::read<std::vector<double>>(yaml, "distort_coeffs");

  left_camera_matrix_ = make_camera_matrix(
    read_vector(yaml, "left_camera_matrix", camera_matrix), "left_camera_matrix");
  right_camera_matrix_ = make_camera_matrix(
    read_vector(yaml, "right_camera_matrix", camera_matrix), "right_camera_matrix");
  left_distortion_coeffs_ = make_distortion_coeffs(
    read_vector(yaml, "left_distort_coeffs", distortion_coeffs));
  right_distortion_coeffs_ = make_distortion_coeffs(
    read_vector(yaml, "right_distort_coeffs", distortion_coeffs));

  baseline_m_ = yaml["baseline_m"] ? yaml["baseline_m"].as<double>() : 0.12;
  disparity_sign_ = yaml["disparity_sign"] ? yaml["disparity_sign"].as<double>() : 1.0;
  min_range_m_ = yaml["min_range_m"] ? yaml["min_range_m"].as<double>() : 0.1;
  max_range_m_ = yaml["max_range_m"] ? yaml["max_range_m"].as<double>() : 100.0;
  min_normalized_disparity_ = yaml["min_normalized_disparity"]
    ? yaml["min_normalized_disparity"].as<double>()
    : 1e-6;
  range_correction_coeffs_ = yaml["range_correction_coeffs"]
    ? yaml["range_correction_coeffs"].as<std::vector<double>>()
    : std::vector<double>{0.0, 1.0, 0.0};

  if (!std::isfinite(baseline_m_) || baseline_m_ <= 0.0) {
    throw std::runtime_error("baseline_m must be a positive finite value.");
  }
  if (!std::isfinite(disparity_sign_) || std::abs(disparity_sign_) < 1e-12) {
    throw std::runtime_error("disparity_sign must be non-zero and finite.");
  }
  if (!std::isfinite(min_range_m_) || !std::isfinite(max_range_m_) ||
      min_range_m_ < 0.0 || max_range_m_ <= min_range_m_) {
    throw std::runtime_error("min_range_m and max_range_m are invalid.");
  }
  if (!std::isfinite(min_normalized_disparity_) || min_normalized_disparity_ < 0.0) {
    throw std::runtime_error("min_normalized_disparity must be finite and non-negative.");
  }
  if (range_correction_coeffs_.size() != 3) {
    throw std::runtime_error("range_correction_coeffs must contain [a0, a1, a2].");
  }
  for (const auto coefficient : range_correction_coeffs_) {
    if (!std::isfinite(coefficient)) {
      throw std::runtime_error("range_correction_coeffs must be finite.");
    }
  }
}

StereoRangeEstimator::Result StereoRangeEstimator::estimate(
  const cv::Point2d & left_pixel, const cv::Point2d & right_pixel) const
{
  Result result;
  result.disparity_px = left_pixel.x - right_pixel.x;

  if (!std::isfinite(left_pixel.x) || !std::isfinite(left_pixel.y) ||
      !std::isfinite(right_pixel.x) || !std::isfinite(right_pixel.y)) {
    return result;
  }

  const std::vector<cv::Point2f> left_input = {
    cv::Point2f(static_cast<float>(left_pixel.x), static_cast<float>(left_pixel.y))};
  const std::vector<cv::Point2f> right_input = {
    cv::Point2f(static_cast<float>(right_pixel.x), static_cast<float>(right_pixel.y))};
  std::vector<cv::Point2f> left_normalized;
  std::vector<cv::Point2f> right_normalized;

  cv::undistortPoints(
    left_input, left_normalized, left_camera_matrix_, left_distortion_coeffs_);
  cv::undistortPoints(
    right_input, right_normalized, right_camera_matrix_, right_distortion_coeffs_);

  result.normalized_disparity =
    disparity_sign_ * (left_normalized.front().x - right_normalized.front().x);
  if (!std::isfinite(result.normalized_disparity) ||
      result.normalized_disparity <= min_normalized_disparity_) {
    return result;
  }

  result.raw_range_m = baseline_m_ / result.normalized_disparity;
  const auto & c = range_correction_coeffs_;
  result.range_m = c[0] + c[1] * result.raw_range_m + c[2] * result.raw_range_m * result.raw_range_m;

  result.valid = std::isfinite(result.range_m) && result.range_m >= min_range_m_ &&
    result.range_m <= max_range_m_;
  return result;
}

cv::Mat StereoRangeEstimator::make_camera_matrix(
  const std::vector<double> & values, const std::string & name)
{
  if (values.size() != 9) {
    throw std::runtime_error(name + " must contain 9 values.");
  }

  return cv::Mat(3, 3, CV_64F, const_cast<double *>(values.data())).clone();
}

cv::Mat StereoRangeEstimator::make_distortion_coeffs(const std::vector<double> & values)
{
  const auto actual_values = values.empty() ? zero_distortion() : values;
  if (actual_values.size() != 4 && actual_values.size() != 5 && actual_values.size() != 8 &&
      actual_values.size() != 12 && actual_values.size() != 14) {
    throw std::runtime_error("distortion coefficients must contain 4, 5, 8, 12, or 14 values.");
  }

  return cv::Mat(1, static_cast<int>(actual_values.size()), CV_64F,
                 const_cast<double *>(actual_values.data())).clone();
}

std::vector<double> StereoRangeEstimator::read_vector(
  const YAML::Node & yaml, const std::string & key, const std::vector<double> & fallback)
{
  return yaml[key] ? yaml[key].as<std::vector<double>>() : fallback;
}
