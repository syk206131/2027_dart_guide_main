#include "camera.hpp"

#include <stdexcept>
#include <type_traits>

#include "hikrobot/hikrobot.hpp"
#include "mindvision/mindvision.hpp"
#include "tools/yaml.hpp"

namespace io
{
Camera::Camera(const std::string & config_path)
: Camera(config_path, "")
{
}

Camera::Camera(const std::string & config_path, const std::string & camera_role)
{
  auto yaml = tools::load(config_path);
  const auto role_yaml =
    (!camera_role.empty() && yaml[camera_role]) ? yaml[camera_role] : YAML::Node{};

  const auto read_option = [&role_yaml, &yaml](const std::string & key, const auto & fallback) {
    using T = std::decay_t<decltype(fallback)>;
    if (role_yaml && role_yaml[key]) return role_yaml[key].as<T>();
    if (yaml[key]) return yaml[key].as<T>();
    return fallback;
  };

  const auto camera_name = read_option("camera_name", std::string("hikrobot"));
  const auto exposure_ms = read_option("exposure_ms", 2.0);
  const auto gain = read_option("gain", 16.0);
  const auto gamma = read_option("gamma", 1.0);
  const auto vid_pid = read_option("vid_pid", std::string(""));
  const auto device_index = read_option("device_index", -1);
  const auto serial_number = read_option("serial_number", std::string(""));
  const auto user_defined_name = read_option("user_defined_name", std::string(""));

  if (camera_name == "mindvision") {
    camera_ = std::make_unique<MindVision>(exposure_ms, gamma, vid_pid);
  }

  else if (camera_name == "hikrobot") {
    camera_ = std::make_unique<HikRobot>(
      exposure_ms, gain, vid_pid, device_index, serial_number, user_defined_name);
  }

  else {
    throw std::runtime_error("Unknow camera_name: " + camera_name + "!");
  }
}

bool Camera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  return camera_->read(img, timestamp);
}

void Camera::stop()
{
  if (camera_) camera_->stop();
}

}  // namespace io
