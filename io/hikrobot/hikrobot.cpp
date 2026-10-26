#include "hikrobot.hpp"

#include <libusb-1.0/libusb.h>

#include <cstring>
#include <cstddef>
#include <mutex>
#include <unordered_map>

#include "tools/logger.hpp"

using namespace std::chrono_literals;

namespace io
{
namespace
{
const std::unordered_map<MvGvspPixelType, cv::ColorConversionCodes> k_pixel_type_map = {
  {PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2RGB},
  {PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2RGB},
  {PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2RGB},
  {PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2RGB}};

std::mutex device_open_mutex;

std::string device_string(const unsigned char * value, std::size_t size)
{
  std::size_t length = 0;
  while (length < size && value[length] != '\0') ++length;
  return std::string(reinterpret_cast<const char *>(value), length);
}

void free_image_buffer(void * handle, MV_FRAME_OUT & raw, bool acquired)
{
  if (!acquired) return;

  const auto ret = MV_CC_FreeImageBuffer(handle, &raw);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_FreeImageBuffer failed: {:#x}", ret);
  }
}
}  // namespace

HikRobot::HikRobot(
  double exposure_ms, double gain, const std::string & vid_pid, int width, int height,
  int device_index, const std::string & serial_number, const std::string & user_defined_name)
: exposure_us_(exposure_ms * 1e3),
  gain_(gain),
  width_(width),
  height_(height),
  daemon_quit_(false),
  handle_(nullptr),
  capturing_(false),
  capture_quit_(true),
  queue_(1),
  vid_(-1),
  pid_(-1),
  device_index_(device_index),
  serial_number_(serial_number),
  user_defined_name_(user_defined_name),
  libusb_initialized_(false)
{
  set_vid_pid(vid_pid);
  libusb_initialized_ = (libusb_init(NULL) == 0);
  if (!libusb_initialized_) tools::logger()->warn("Unable to init libusb!");

  daemon_thread_ = std::thread{[this] {
    tools::logger()->info("HikRobot's daemon thread started.");

    capture_start();

    while (!daemon_quit_) {
      std::this_thread::sleep_for(100ms);

      if (capturing_) continue;

      capture_stop();
      if (daemon_quit_) break;

      reset_usb();
      capture_start();
    }

    capture_stop();

    tools::logger()->info("HikRobot's daemon thread stopped.");
  }};
}

HikRobot::~HikRobot()
{
  stop();
  if (daemon_thread_.joinable()) daemon_thread_.join();

  if (capture_thread_.joinable()) capture_thread_.join();
  release_handle();

  if (libusb_initialized_) libusb_exit(NULL);

  tools::logger()->info("HikRobot destructed.");
}

void HikRobot::stop()
{
  daemon_quit_ = true;
  capture_quit_ = true;
  queue_.stop();
}

bool HikRobot::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  CameraData data;
  if (!queue_.pop(data)) return false;

  img = data.img;
  timestamp = data.timestamp;
  return true;
}

MV_CC_DEVICE_INFO * HikRobot::find_device(MV_CC_DEVICE_INFO_LIST & device_list) const
{
  if (device_list.nDeviceNum == 0) return nullptr;

  int matching_index = 0;
  const bool has_selector =
    device_index_ >= 0 || !serial_number_.empty() || !user_defined_name_.empty();

  for (unsigned int i = 0; i < device_list.nDeviceNum; ++i) {
    auto * device = device_list.pDeviceInfo[i];
    if (device == nullptr || device->nTLayerType != MV_USB_DEVICE) continue;

    const auto & usb_info = device->SpecialInfo.stUsb3VInfo;
    if (vid_ != -1 && pid_ != -1 &&
        (usb_info.idVendor != static_cast<unsigned short>(vid_) ||
         usb_info.idProduct != static_cast<unsigned short>(pid_))) {
      continue;
    }

    const auto serial = device_string(usb_info.chSerialNumber, sizeof(usb_info.chSerialNumber));
    const auto user_name =
      device_string(usb_info.chUserDefinedName, sizeof(usb_info.chUserDefinedName));
    if (!serial_number_.empty() && serial != serial_number_) continue;
    if (!user_defined_name_.empty() && user_name != user_defined_name_) continue;

    if (device_index_ >= 0) {
      if (matching_index++ != device_index_) continue;
    }

    tools::logger()->info(
      "Selected HikRobot camera: serial='{}', user_defined_name='{}', device_index={}",
      serial, user_name, i);
    return device;
  }

  if (has_selector) {
    tools::logger()->warn(
      "HikRobot camera selector did not match: index={}, serial='{}', user_defined_name='{}'",
      device_index_, serial_number_, user_defined_name_);
    return nullptr;
  }

  tools::logger()->warn("HikRobot camera selector is empty; using the first enumerated camera.");
  return device_list.pDeviceInfo[0];
}

void HikRobot::capture_start()
{
  if (handle_ != nullptr) return;

  std::lock_guard<std::mutex> device_lock(device_open_mutex);
  if (daemon_quit_) return;

  capture_quit_ = false;
  capturing_ = false;

  unsigned int ret;

  MV_CC_DEVICE_INFO_LIST device_list{};
  ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_EnumDevices failed: {:#x}", ret);
    return;
  }

  auto * device = find_device(device_list);
  if (device == nullptr) {
    tools::logger()->warn("Not found camera!");
    return;
  }
  selected_serial_ = device_string(
    device->SpecialInfo.stUsb3VInfo.chSerialNumber,
    sizeof(device->SpecialInfo.stUsb3VInfo.chSerialNumber));

  ret = MV_CC_CreateHandle(&handle_, device);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_CreateHandle failed: {:#x}", ret);
    handle_ = nullptr;
    return;
  }

  ret = MV_CC_OpenDevice(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_OpenDevice failed: {:#x}", ret);
    release_handle();
    return;
  }

  set_enum_value("TriggerMode", MV_TRIGGER_MODE_OFF);
  set_enum_value("BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
  set_enum_value("ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
  set_enum_value("GainAuto", MV_GAIN_MODE_OFF);
  set_float_value("ExposureTime", exposure_us_);
  set_float_value("Gain", gain_);
  if (width_ > 0 && height_ > 0) {
    set_int_value("Width", width_);
    set_int_value("Height", height_);
  }
  MV_CC_SetFrameRate(handle_, 150);

  ret = MV_CC_StartGrabbing(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_StartGrabbing failed: {:#x}", ret);
    release_handle();
    return;
  }

  capturing_ = true;

  capture_thread_ = std::thread{[this] {
    tools::logger()->info("HikRobot's capture thread started.");

    MV_FRAME_OUT raw{};

    while (!capture_quit_) {
      std::this_thread::sleep_for(1ms);

      unsigned int ret;
      constexpr unsigned int nMsec = 100;
      bool buffer_acquired = false;

      ret = MV_CC_GetImageBuffer(handle_, &raw, nMsec);
      if (ret != MV_OK) {
        tools::logger()->warn("MV_CC_GetImageBuffer failed: {:#x}", ret);
        break;
      }
      buffer_acquired = true;

      const auto timestamp = std::chrono::steady_clock::now();
      cv::Mat img(cv::Size(raw.stFrameInfo.nWidth, raw.stFrameInfo.nHeight), CV_8U, raw.pBufAddr);

      const auto pixel_type = raw.stFrameInfo.enPixelType;
      const auto type_it = k_pixel_type_map.find(pixel_type);
      if (type_it == k_pixel_type_map.end()) {
        tools::logger()->warn("Unsupported pixel type: {:#x}", static_cast<unsigned int>(pixel_type));
        free_image_buffer(handle_, raw, buffer_acquired);
        break;
      }

      cv::Mat dst_image;
      cv::cvtColor(img, dst_image, type_it->second);
      img = dst_image;

      queue_.push({img, timestamp});

      free_image_buffer(handle_, raw, buffer_acquired);
      buffer_acquired = false;
    }

    capturing_ = false;
    tools::logger()->info("HikRobot's capture thread stopped.");
  }};
}

void HikRobot::release_handle()
{
  if (handle_ == nullptr) return;

  unsigned int ret;

  ret = MV_CC_StopGrabbing(handle_);
  if (ret != MV_OK) tools::logger()->warn("MV_CC_StopGrabbing failed: {:#x}", ret);

  ret = MV_CC_CloseDevice(handle_);
  if (ret != MV_OK) tools::logger()->warn("MV_CC_CloseDevice failed: {:#x}", ret);

  ret = MV_CC_DestroyHandle(handle_);
  if (ret != MV_OK) tools::logger()->warn("MV_CC_DestroyHandle failed: {:#x}", ret);

  handle_ = nullptr;
}

void HikRobot::capture_stop()
{
  capture_quit_ = true;
  if (capture_thread_.joinable()) capture_thread_.join();
  release_handle();
}

void HikRobot::set_float_value(const std::string & name, double value)
{
  if (handle_ == nullptr) return;

  const auto ret = MV_CC_SetFloatValue(handle_, name.c_str(), value);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_SetFloatValue(\"{}\", {}) failed: {:#x}", name, value, ret);
  }
}

void HikRobot::set_enum_value(const std::string & name, unsigned int value)
{
  if (handle_ == nullptr) return;

  const auto ret = MV_CC_SetEnumValue(handle_, name.c_str(), value);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_SetEnumValue(\"{}\", {}) failed: {:#x}", name, value, ret);
  }
}

void HikRobot::set_int_value(const std::string & name, int value)
{
  if (handle_ == nullptr) return;

  const auto ret = MV_CC_SetIntValue(handle_, name.c_str(), value);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_SetIntValue(\"{}\", {}) failed: {:#x}", name, value, ret);
  }
}

void HikRobot::set_vid_pid(const std::string & vid_pid)
{
  auto index = vid_pid.find(':');
  if (index == std::string::npos) {
    tools::logger()->warn("Invalid vid_pid: \"{}\"", vid_pid);
    return;
  }

  auto vid_str = vid_pid.substr(0, index);
  auto pid_str = vid_pid.substr(index + 1);

  try {
    vid_ = std::stoi(vid_str, 0, 16);
    pid_ = std::stoi(pid_str, 0, 16);
  } catch (const std::exception &) {
    tools::logger()->warn("Invalid vid_pid: \"{}\"", vid_pid);
  }
}

void HikRobot::reset_usb() const
{
  if (vid_ == -1 || pid_ == -1) return;

  // https://github.com/ralight/usb-reset/blob/master/usb-reset.c
  auto handle = libusb_open_device_with_vid_pid(NULL, vid_, pid_);
  if (!handle) {
    tools::logger()->warn("Unable to open usb!");
    return;
  }

  if (libusb_reset_device(handle))
    tools::logger()->warn("Unable to reset usb!");
  else
    tools::logger()->info("Reset usb successfully :)");

  libusb_close(handle);
}

}  // namespace io
