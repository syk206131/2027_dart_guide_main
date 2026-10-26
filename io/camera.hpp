#ifndef IO__CAMERA_HPP
#define IO__CAMERA_HPP

#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

namespace io
{
class CameraBase
{
public:
  virtual ~CameraBase() = default;
  virtual bool read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) = 0;
  virtual void stop() {}
  virtual std::string serial() const { return ""; }
};

class Camera
{
public:
  Camera(const std::string & config_path);
  Camera(const std::string & config_path, const std::string & camera_role);
  bool read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp);
  void stop();
  std::string serial() const;

private:
  std::unique_ptr<CameraBase> camera_;
};

}  // namespace io

#endif  // IO__CAMERA_HPP
