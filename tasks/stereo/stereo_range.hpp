#ifndef TASKS__STEREO__STEREO_RANGE_HPP_
#define TASKS__STEREO__STEREO_RANGE_HPP_

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include <string>
#include <vector>

class StereoRangeEstimator
{
public:
  struct Result
  {
    bool valid = false;
    double raw_range_m = 0.0;
    double range_m = 0.0;
    double disparity_px = 0.0;
    double normalized_disparity = 0.0;
  };

  explicit StereoRangeEstimator(const std::string & config_path);

  Result estimate(const cv::Point2d & left_pixel, const cv::Point2d & right_pixel) const;

  double left_principal_x() const { return left_camera_matrix_.at<double>(0, 2); }
  double right_principal_x() const { return right_camera_matrix_.at<double>(0, 2); }
  double baseline_m() const { return baseline_m_; }

private:
  static cv::Mat make_camera_matrix(const std::vector<double> & values, const std::string & name);
  static cv::Mat make_distortion_coeffs(const std::vector<double> & values);
  static std::vector<double> read_vector(
    const YAML::Node & yaml, const std::string & key, const std::vector<double> & fallback);

  cv::Mat left_camera_matrix_;
  cv::Mat right_camera_matrix_;
  cv::Mat left_distortion_coeffs_;
  cv::Mat right_distortion_coeffs_;

  double baseline_m_;
  double disparity_sign_;
  double min_range_m_;
  double max_range_m_;
  double min_normalized_disparity_;

  // Coefficient order is [a0, a1, a2]: range = a0 + a1 * raw + a2 * raw^2.
  std::vector<double> range_correction_coeffs_;
};

#endif  // TASKS__STEREO__STEREO_RANGE_HPP_
