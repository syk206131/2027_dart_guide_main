#ifndef IO__GIMBAL_HPP
#define IO__GIMBAL_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>

#include "serial/serial.h"
#include "tools/thread_safe_queue.hpp"

namespace io
{
struct __attribute__((packed)) GimbalToVision
{
  uint8_t head[2] = {'E', 'C'};
  uint8_t state; 
  uint16_t crc16;
};

static_assert(sizeof(GimbalToVision) == 5); 
struct __attribute__((packed)) VisionToGimbal
{
  uint8_t head[2] = {'V', 'C'};
  uint8_t status = 0;
  float yaw_offset = 0.0f;
  float distance_m = -1.0f;
  uint16_t crc16 = 0;
};
static_assert(sizeof(VisionToGimbal) == 13);

enum class GimbalMode
{
  ERROR,       // 对应电控00（错误）
  RUNNING,     // 对应电控01（开始）
  IDLE         // 视觉本地空闲状态（用于初始化）
};

class Gimbal
{
public:
  Gimbal(const std::string & config_path);

  ~Gimbal();

  GimbalMode mode() const;
  std::string str(GimbalMode mode) const;

  void send(io::VisionToGimbal VisionToGimbal);

private:
  serial::Serial serial_;

  std::thread thread_;
  std::atomic<bool> quit_ = false;
  mutable std::mutex mutex_;
  mutable std::mutex serial_mutex_;

  GimbalToVision rx_data_;
  VisionToGimbal tx_data_;

  GimbalMode mode_ = GimbalMode::IDLE;

  int reconnect_max_retry_count_ = 10;
  std::chrono::milliseconds reconnect_timeout_{5000};
  std::chrono::milliseconds slow_packet_threshold_{200};
  std::chrono::microseconds idle_read_sleep_{1000};

  bool read(uint8_t * buffer, size_t size);
  void read_thread();
  void reconnect();
};

}  // namespace io

#endif  // IO__GIMBAL_HPP
