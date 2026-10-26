#include "gimbal.hpp"
#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io
{
Gimbal::Gimbal(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto com_port = tools::read<std::string>(yaml, "com_port");
  auto baud_rate = tools::read<int>(yaml, "baud_rate");
  serial_.setBaudrate(baud_rate);

  try {
    serial_.setPort(com_port);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(20);  //可能需要调整
    serial_.setTimeout(timeout);

    serial_.open();
  } catch (const std::exception & e) {
    tools::logger()->error("[Gimbal] Failed to open serial: {}", e.what());
    exit(1);
  }


  thread_ = std::thread(&Gimbal::read_thread, this);
  tools::logger()->info("[Gimbal] Serial port opened, read thread started.");
}

Gimbal::~Gimbal()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();
  serial_.close();
}

GimbalMode Gimbal::mode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

float Gimbal::roll_rad() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return rx_data_.roll_rad;
}

uint8_t Gimbal::dart_index() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return rx_data_.dart_index;
}

uint8_t Gimbal::target() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return rx_data_.target;
}

std::string Gimbal::str(GimbalMode mode) const
{
  switch (mode) {
    case GimbalMode::IDLE:
      return "IDLE";
    case GimbalMode::ERROR:
      return "ERROR";
    case GimbalMode::RUNNING:
      return "RUNNING";
    default:
      return "INVALID";
  }
}

void Gimbal::send(io::VisionToGimbal vision_to_gimbal)
{
  std::lock_guard<std::mutex> serial_lock(serial_mutex_);
  tx_data_ = vision_to_gimbal;
  tx_data_.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data_), 
    sizeof(tx_data_) - sizeof(tx_data_.crc16)
  );

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

bool Gimbal::read(uint8_t * buffer, size_t size)
{
  std::lock_guard<std::mutex> serial_lock(serial_mutex_);
  try {
    return serial_.read(buffer, size) == size;
  } catch (const std::exception & e) {
    return false;
  }
}

void Gimbal::read_thread()
{
  tools::logger()->info("[Gimbal] read_thread started.");
  int error_count = 0;

  while (!quit_) {
    if (error_count > 5000) {
      error_count = 0;
      tools::logger()->warn("[Gimbal] Too many errors, attempting to reconnect...");
      reconnect();
      continue;
    }

    if (!read(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_))) {
      error_count++;
      std::this_thread::sleep_for(std::chrono::microseconds(10));
      continue;
    }

    if (rx_data_.head[0] != 'E' || rx_data_.head[1] != 'C') {
      error_count++;
      continue;
    }

    if (!tools::check_crc16(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_))) {
      tools::logger()->debug("[Gimbal] CRC16 check failed.");
      error_count++;
      continue;
    }

    error_count = 0;
    std::lock_guard<std::mutex> lock(mutex_);
    switch (rx_data_.state) {
      case 0x00:
        mode_ = GimbalMode::IDLE;  // 空闲：舱门关闭，视觉不自瞄
        break;
      case 0x01:
        mode_ = GimbalMode::RUNNING;
        break;
      case 0x02:
        mode_ = GimbalMode::ERROR;
        break;
      default:
        mode_ = GimbalMode::IDLE;
        tools::logger()->warn("[Gimbal] Invalid state from gimbal: {}", rx_data_.state);
        break;
    }
    tools::logger()->debug("[Gimbal] Received state: {}, mode: {}", 
                           rx_data_.state, str(mode_));
  }

  tools::logger()->info("[Gimbal] read_thread stopped.");
}

void Gimbal::reconnect()
{
  std::lock_guard<std::mutex> serial_lock(serial_mutex_);
  int max_retry_count = 10;
  for (int i = 0; i < max_retry_count && !quit_; ++i) {
    tools::logger()->warn("[Gimbal] Reconnecting serial, attempt {}/{}...", i + 1, max_retry_count);
    try {
      serial_.close();
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    } catch (...) {
    }

    try {
      serial_.setPort(serial_.getPort()); 
      serial_.open();
      tools::logger()->info("[Gimbal] Reconnected serial successfully.");
      break;
    } catch (const std::exception & e) {
      tools::logger()->warn("[Gimbal] Reconnect failed: {}", e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}

}  // namespace io 
