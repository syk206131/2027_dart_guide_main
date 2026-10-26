#ifndef TOOLS__STEREO_FRAME_QUEUE_HPP
#define TOOLS__STEREO_FRAME_QUEUE_HPP

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <utility>

namespace tools
{
enum class StereoCameraSide
{
  LEFT,
  RIGHT
};

struct TimestampedFrame
{
  cv::Mat image;
  std::chrono::steady_clock::time_point timestamp;
};

struct StereoFramePair
{
  TimestampedFrame left;
  TimestampedFrame right;
  std::chrono::steady_clock::time_point timestamp;
};

/**
 * Matches frames from two independently captured cameras by one steady-clock
 * time base. Frames outside max_delta are discarded as stale instead of being
 * paired with a frame from the wrong moment.
 */
class StereoFrameQueue
{
public:
  using Duration = std::chrono::steady_clock::duration;

  StereoFrameQueue(std::size_t max_size, Duration max_delta)
  : max_size_(max_size == 0 ? 1 : max_size), max_delta_(max_delta)
  {
  }

  void push(StereoCameraSide side, TimestampedFrame frame)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (stopped_) return;

    auto & queue = side == StereoCameraSide::LEFT ? left_ : right_;
    queue.emplace_back(std::move(frame));
    while (queue.size() > max_size_) queue.pop_front();

    match_locked();
    condition_.notify_all();
  }

  /// Returns false when stopped and no matched pair remains.
  bool pop(StereoFramePair & pair)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    condition_.wait(lock, [this] { return !pairs_.empty() || stopped_; });

    if (pairs_.empty()) return false;
    pair = std::move(pairs_.front());
    pairs_.pop_front();
    return true;
  }

  void stop()
  {
    stopped_.store(true);
    condition_.notify_all();
  }

  bool stopped() const { return stopped_.load(); }

private:
  void match_locked()
  {
    while (!left_.empty() && !right_.empty()) {
      const auto delta = left_.front().timestamp - right_.front().timestamp;

      if (delta > max_delta_) {
        // The left front is newer than the right front by more than max_delta,
        // so the stale right frame can never match; drop it.
        right_.pop_front();
        continue;
      }

      if (delta < -max_delta_) {
        // The left front is older than the right front by more than max_delta,
        // so the stale left frame can never match; drop it.
        left_.pop_front();
        continue;
      }

      const auto pair_timestamp =
        left_.front().timestamp + (right_.front().timestamp - left_.front().timestamp) / 2;
      pairs_.push_back(
        {std::move(left_.front()), std::move(right_.front()), pair_timestamp});
      left_.pop_front();
      right_.pop_front();

      while (pairs_.size() > max_size_) pairs_.pop_front();
    }
  }

  const std::size_t max_size_;
  const Duration max_delta_;

  std::deque<TimestampedFrame> left_;
  std::deque<TimestampedFrame> right_;
  std::deque<StereoFramePair> pairs_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::atomic<bool> stopped_{false};
};

}  // namespace tools

#endif  // TOOLS__STEREO_FRAME_QUEUE_HPP
