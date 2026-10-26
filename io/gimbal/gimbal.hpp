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
// 电控 → 视觉：状态（0x00 空闲/舱门关，0x01 运行/自瞄）+ 当前镖序号
// + 目标（0x00 前哨站 / 0x01 基地）+ roll 轴偏角（顺时针为正）
struct __attribute__((packed)) GimbalToVision
{
  uint8_t head[2] = {'E', 'C'};
  uint8_t state;
  uint8_t dart_index = 0;
  uint8_t target = 1;  // 0=前哨站，1=基地
  float roll_rad = 0.0f;
  uint16_t crc16;
};

static_assert(sizeof(GimbalToVision) == 11);

struct __attribute__((packed)) VisionToGimbal
{
  uint8_t head[2] = {'V', 'C'};
  uint8_t control_state = 0;
  float energy_j = 0.0f;
  uint16_t crc16;
};
static_assert(sizeof(VisionToGimbal) == 9);

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
  float roll_rad() const;
  uint8_t dart_index() const;
  uint8_t target() const;  // 0=前哨站，1=基地

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
