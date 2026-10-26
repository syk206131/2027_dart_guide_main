#ifndef CALIBRATION__STEREO_CALIBRATION_HPP_
#define CALIBRATION__STEREO_CALIBRATION_HPP_

#include <opencv2/opencv.hpp>

#include "calibration.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace calibration
{
struct StereoResult
{
  bool valid = false;
  cv::Size image_size;

  // Right-camera-to-left-camera rigid transform: X_left = R * X_right + T.
  cv::Mat rotation;
  cv::Mat translation;

  double rms_error = 0.0;       // joint stereoCalibrate RMS (px)
  double left_rms_error = 0.0;  // per-camera reprojection RMS (px)
  double right_rms_error = 0.0;

  double baseline_m = 0.0;  // ||T||

  // Relative tilt of the two optical axes decomposed from R, in degrees.
  double tilt_x_deg = 0.0;
  double tilt_y_deg = 0.0;
  double tilt_z_deg = 0.0;

  // Mean/max row mismatch of matched corners after stereoRectify, in pixels.
  // For perfectly rectified views corresponding points share the same row.
  double epipolar_mean_px = 0.0;
  double epipolar_max_px = 0.0;

  std::size_t detected_pairs = 0;
  std::size_t total_pairs = 0;
};

/**
 * Stereo calibration from synchronized chessboard image pairs.
 *
 * image_paths_left/right must be aligned: [i] of each vector is the same
 * physical moment. When camera_matrix/distortion are non-empty and
 * fix_intrinsic is true, the already-measured single-camera intrinsics are
 * held fixed and only R/T are estimated (recommended: single-camera
 * calibration typically has many more views than the shared stereo set).
 */
StereoResult calibrate_stereo(
  const std::vector<std::string> & image_paths_left,
  const std::vector<std::string> & image_paths_right,
  const Options & options,
  const cv::Mat & left_camera_matrix,
  const cv::Mat & left_distortion,
  const cv::Mat & right_camera_matrix,
  const cv::Mat & right_distortion,
  bool fix_intrinsic);

bool write_stereo_yaml(const std::string & output_path, const StereoResult & result);

}  // namespace calibration

#endif  // CALIBRATION__STEREO_CALIBRATION_HPP_
