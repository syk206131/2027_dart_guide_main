#include "calibration.hpp"

#include <opencv2/opencv.hpp>

#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace
{
void print_usage(const char * program)
{
  std::cout << "Usage: " << program
            << " --images <directory> --output <yaml> --name <left|right>"
               " [--corners <columns> <rows>] [--square-size <meters>]\n";
}

bool read_value(int & index, int argc, char ** argv, std::string & value)
{
  if (index + 1 >= argc) return false;
  value = argv[++index];
  return true;
}
}  // namespace

int main(int argc, char ** argv)
{
  std::string image_directory;
  std::string output_path;
  std::string name;
  calibration::Options options;

  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--images") {
      if (!read_value(index, argc, argv, image_directory)) return 1;
    } else if (argument == "--output") {
      if (!read_value(index, argc, argv, output_path)) return 1;
    } else if (argument == "--name") {
      if (!read_value(index, argc, argv, name)) return 1;
    } else if (argument == "--corners") {
      if (index + 2 >= argc) return 1;
      options.board_size.width = std::stoi(argv[++index]);
      options.board_size.height = std::stoi(argv[++index]);
    } else if (argument == "--square-size") {
      if (index + 1 >= argc) return 1;
      options.square_size_m = std::stof(argv[++index]);
    } else {
      print_usage(argv[0]);
      return 1;
    }
  }

  if (image_directory.empty() || output_path.empty() || name.empty()) {
    print_usage(argv[0]);
    return 1;
  }

  std::vector<std::string> image_paths;
  cv::glob(image_directory + "/*", image_paths, false);
  if (image_paths.empty()) {
    std::cerr << "No images found in: " << image_directory << '\n';
    return 1;
  }

  const auto result = calibration::calibrate_camera(image_paths, options);
  if (!result.valid) {
    std::cerr << "Calibration failed: detected " << result.detected_views << " of "
              << result.total_views << " views.\n";
    return 1;
  }

  if (!calibration::write_yaml(output_path, name, result)) {
    std::cerr << "Failed to write calibration file: " << output_path << '\n';
    return 1;
  }

  std::cout << std::fixed << std::setprecision(6)
            << "Calibration succeeded\n"
            << "  detected views: " << result.detected_views << "/" << result.total_views << '\n'
            << "  image size: " << result.image_size.width << " x " << result.image_size.height << '\n'
            << "  RMS error: " << result.rms_error << " px\n"
            << "  output: " << output_path << '\n';

  return 0;
}