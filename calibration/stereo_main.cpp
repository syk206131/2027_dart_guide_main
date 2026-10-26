#include "stereo_calibration.hpp"

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <map>
#include <regex>
#include <string>
#include <vector>

namespace
{
void print_usage(const char * program)
{
  std::cout
    << "Usage: " << program
    << " --left <dir> --right <dir> --output <yaml>\n"
       "       [--config <yaml>] [--corners <columns> <rows>] [--square-size <meters>]\n"
       "       [--refine-intrinsics]\n"
       "\n"
       "Pairs synchronized chessboard images saved by stereo_capture and\n"
       "computes R/T (baseline) between the two cameras. Intrinsics are read\n"
       "from --config (left/right_camera_matrix) and held FIXED unless\n"
       "--refine-intrinsics is given.\n";
}

bool read_value(int & index, int argc, char ** argv, std::string & value)
{
  if (index + 1 >= argc) return false;
  value = argv[++index];
  return true;
}

cv::Mat to_camera_matrix(const std::vector<double> & values)
{
  if (values.size() != 9) return {};
  return cv::Mat(3, 3, CV_64F, const_cast<double *>(values.data())).clone();
}

cv::Mat to_distortion(const std::vector<double> & values)
{
  if (values.empty()) return cv::Mat::zeros(1, 5, CV_64F);
  cv::Mat distortion(1, static_cast<int>(values.size()), CV_64F);
  for (std::size_t i = 0; i < values.size(); ++i) {
    distortion.at<double>(0, static_cast<int>(i)) = values[i];
  }
  return distortion;
}

std::vector<double> read_vector(const YAML::Node & yaml, const std::string & key)
{
  if (!yaml[key]) return {};
  return yaml[key].as<std::vector<double>>();
}

// trailing number in a file name, used to align left/right pairs
std::string index_key(const std::filesystem::path & path)
{
  static const std::regex number_regex(R"((\d+)(?=\D*$))");
  const std::string stem = path.stem().string();
  std::smatch match;
  if (std::regex_search(stem, match, number_regex)) return match[1].str();
  return stem;  // fallback: pair by whole stem
}

std::vector<std::string> list_images(const std::string & directory)
{
  static const std::vector<std::string> extensions = {
    ".bmp", ".png", ".jpg", ".jpeg"};
  std::map<std::string, std::string> indexed;
  for (const auto & entry : std::filesystem::directory_iterator(directory)) {
    const auto ext = entry.path().extension().string();
    const bool supported = std::any_of(
      extensions.begin(), extensions.end(),
      [&](const std::string & e) {
        return ext.size() == e.size() &&
               std::equal(
                 ext.begin(), ext.end(), e.begin(),
                 [](char a, char b) { return std::tolower(a) == std::tolower(b); });
      });
    if (!supported) continue;
    indexed[index_key(entry.path())] = entry.path().string();
  }

  std::vector<std::string> paths;
  paths.reserve(indexed.size());
  for (const auto & [key, path] : indexed) paths.push_back(path);
  return paths;
}
}  // namespace

int main(int argc, char ** argv)
{
  std::string left_directory;
  std::string right_directory;
  std::string output_path;
  std::string config_path = "configs/test.yaml";
  calibration::Options options;
  bool refine_intrinsics = false;

  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--left") {
      if (!read_value(index, argc, argv, left_directory)) { print_usage(argv[0]); return 1; }
    } else if (argument == "--right") {
      if (!read_value(index, argc, argv, right_directory)) { print_usage(argv[0]); return 1; }
    } else if (argument == "--output") {
      if (!read_value(index, argc, argv, output_path)) { print_usage(argv[0]); return 1; }
    } else if (argument == "--config") {
      if (!read_value(index, argc, argv, config_path)) { print_usage(argv[0]); return 1; }
    } else if (argument == "--corners") {
      if (index + 2 >= argc) { print_usage(argv[0]); return 1; }
      options.board_size.width = std::stoi(argv[++index]);
      options.board_size.height = std::stoi(argv[++index]);
    } else if (argument == "--square-size") {
      if (index + 1 >= argc) { print_usage(argv[0]); return 1; }
      options.square_size_m = std::stof(argv[++index]);
    } else if (argument == "--refine-intrinsics") {
      refine_intrinsics = true;
    } else {
      print_usage(argv[0]);
      return 1;
    }
  }

  if (left_directory.empty() || right_directory.empty() || output_path.empty()) {
    print_usage(argv[0]);
    return 1;
  }

  YAML::Node config;
  try {
    config = YAML::LoadFile(config_path);
  } catch (const std::exception & e) {
    std::cerr << "Failed to load config: " << e.what() << '\n';
    return 1;
  }

  const cv::Mat k1 = to_camera_matrix(read_vector(config, "left_camera_matrix"));
  const cv::Mat d1 = to_distortion(read_vector(config, "left_distort_coeffs"));
  const cv::Mat k2 = to_camera_matrix(read_vector(config, "right_camera_matrix"));
  const cv::Mat d2 = to_distortion(read_vector(config, "right_distort_coeffs"));
  if (!refine_intrinsics && (k1.empty() || k2.empty())) {
    std::cerr << "left/right_camera_matrix missing in " << config_path
              << ". Run single-camera calibration first, or pass --refine-intrinsics.\n";
    return 1;
  }

  auto left_paths = list_images(left_directory);
  auto right_paths = list_images(right_directory);
  if (left_paths.empty() || right_paths.empty()) {
    std::cerr << "No images found in " << left_directory << " or " << right_directory << '\n';
    return 1;
  }
  if (left_paths.size() != right_paths.size()) {
    std::cerr << "Pair count mismatch: " << left_paths.size() << " left vs "
              << right_paths.size() << " right. Pairs are aligned by file index; "
              << "delete stray frames from either directory.\n";
    return 1;
  }

  const auto result = calibration::calibrate_stereo(
    left_paths, right_paths, options, k1, d1, k2, d2, !refine_intrinsics);

  if (!result.valid) {
    std::cerr << "Stereo calibration failed: only " << result.detected_pairs
              << " of " << result.total_pairs << " pairs had the board in BOTH views "
              << "(need at least 5).\n";
    return 1;
  }

  if (!calibration::write_stereo_yaml(output_path, result)) {
    std::cerr << "Failed to write: " << output_path << '\n';
    return 1;
  }

  std::cout << std::fixed << std::setprecision(6)
            << "Stereo calibration succeeded\n"
            << "  detected pairs: " << result.detected_pairs << "/" << result.total_pairs << '\n'
            << "  image size: " << result.image_size.width << " x "
            << result.image_size.height << '\n'
            << "  joint RMS: " << result.rms_error << " px\n"
            << "  left RMS:  " << result.left_rms_error << " px\n"
            << "  right RMS: " << result.right_rms_error << " px\n"
            << "  baseline:  " << std::setprecision(4) << result.baseline_m * 1000.0
            << " mm (" << std::setprecision(6) << result.baseline_m << " m)\n"
            << "  T (m):     [" << result.translation.at<double>(0) << ", "
            << result.translation.at<double>(1) << ", "
            << result.translation.at<double>(2) << "]\n"
            << "  tilt (deg): x=" << result.tilt_x_deg
            << " y=" << result.tilt_y_deg
            << " z=" << result.tilt_z_deg << '\n'
            << "  epipolar row error: mean=" << result.epipolar_mean_px
            << " px max=" << result.epipolar_max_px << " px\n"
            << "  output: " << output_path << '\n';

  return 0;
}
