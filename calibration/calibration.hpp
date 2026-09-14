#ifndef CALIBRATION__CALIBRATION_HPP_
#define CALIBRATION__CALIBRATION_HPP_

#include <opencv2/opencv.hpp>

#include <string>
#include <vector>

namespace calibration
{
struct Options
{
  cv::Size board_size{11, 8};
  float square_size_m = 0.025F;
};

struct Result
{
  bool valid = false;
  cv::Size image_size;
  cv::Mat camera_matrix;
  cv::Mat distortion_coeffs;
  double rms_error = 0.0;
  std::size_t detected_views = 0;
  std::size_t total_views = 0;
};

Result calibrate_camera(const std::vector<std::string> & image_paths, const Options & options);

bool write_yaml(
  const std::string & output_path, const std::string & name, const Result & result);

}  // namespace calibration

#endif  // CALIBRATION__CALIBRATION_HPP_