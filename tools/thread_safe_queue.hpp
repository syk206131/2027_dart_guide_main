#ifndef TOOLS__THREAD_SAFE_QUEUE_HPP
#define TOOLS__THREAD_SAFE_QUEUE_HPP

#include <atomic>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <mutex>
#include <queue>

namespace tools
{
template <typename T, bool PopWhenFull = false>
class ThreadSafeQueue
{
public:
  ThreadSafeQueue(
    size_t max_size, std::function<void(void)> full_handler = [] {})
  : max_size_(max_size), full_handler_(full_handler)
  {
  }

  void push(const T & value)
  {
    std::unique_lock<std::mutex> lock(mutex_);

    if (stopped_) return;

    if (queue_.size() >= max_size_) {
      if (PopWhenFull) {
        queue_.pop();
      } else {
        full_handler_();
        return;
      }
    }

    queue_.push(value);
    not_empty_condition_.notify_all();
  }

  void stop()
  {
    stopped_.store(true);
    not_empty_condition_.notify_all();
  }

  bool stopped() const { return stopped_.load(); }

  /// Returns false when the queue has been stopped and is empty.
  bool pop(T & value)
  {
    std::unique_lock<std::mutex> lock(mutex_);

    not_empty_condition_.wait(lock, [this] { return !queue_.empty() || stopped_; });

    if (queue_.empty()) return false;

    value = queue_.front();
    queue_.pop();
    return true;
  }

  T pop()
  {
    T value;
    pop(value);
    return value;
  }

  T front()
  {
    std::unique_lock<std::mutex> lock(mutex_);

    not_empty_condition_.wait(lock, [this] { return !queue_.empty() || stopped_; });

    return queue_.front();
  }

  void back(T & value)
  {
    std::unique_lock<std::mutex> lock(mutex_);

    if (queue_.empty()) {
      std::cerr << "Error: Attempt to access the back of an empty queue." << std::endl;
      return;
    }

    value = queue_.back();
  }

  bool empty()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return queue_.empty();
  }

  void clear()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    while (!queue_.empty()) {
      queue_.pop();
    }
    not_empty_condition_.notify_all();  // 如果其他线程正在等待队列不为空，这样可以唤醒它们
  }

private:
  std::queue<T> queue_;
  size_t max_size_;
  mutable std::mutex mutex_;
  std::condition_variable not_empty_condition_;
  std::function<void(void)> full_handler_;
  std::atomic<bool> stopped_{false};
};

}  // namespace tools

#endif  // TOOLS__THREAD_SAFE_QUEUE_HPP
