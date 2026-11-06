// 作用：为单块控制板提供独占周期线程和优先服务槽；用法：硬件插件及离线模拟测试包含本文件。
#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <condition_variable>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace robot_hardware
{
class TimingWindow
{
public:
  void add(double milliseconds, bool failed = false)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (samples_.size() == 1024) samples_.erase(samples_.begin());
    samples_.push_back(milliseconds);
    ++count_;
    failures_ += failed;
    maximum_ = std::max(maximum_, milliseconds);
  }
  struct Summary {double p50{}, p95{}, p99{}, maximum{}; uint64_t count{}, failures{};};
  Summary summary() const
  {
    std::vector<double> copy;
    Summary value;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      copy = samples_;
      value.maximum = maximum_;
      value.count = count_;
      value.failures = failures_;
    }
    if (copy.empty()) return value;
    std::sort(copy.begin(), copy.end());
    const auto at = [&](double q) {return copy[static_cast<size_t>(
        std::ceil(q * copy.size())) - 1];};
    value.p50 = at(0.50); value.p95 = at(0.95); value.p99 = at(0.99);
    return value;
  }
private:
  mutable std::mutex mutex_;
  std::vector<double> samples_;
  uint64_t count_{0}, failures_{0};
  double maximum_{0};
};

class BoardWorker
{
public:
  using Clock = std::chrono::steady_clock;
  ~BoardWorker() {stop();}
  void start(double hz, std::function<bool()> cycle, std::function<void()> finish)
  {
    stop();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      running_ = true;
    }
    thread_ = std::thread([this, hz, cycle, finish] {
      const auto period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(1.0 / hz));
      auto next = Clock::now();
      while (true) {
        std::shared_ptr<Task> task;
        {
          std::unique_lock<std::mutex> lock(mutex_);
          cv_.wait_until(lock, next, [&] {return !running_ || urgent_ || normal_;});
          if (!running_) break;
          if (urgent_) task = std::move(urgent_);
          else if (normal_) task = std::move(normal_);
        }
        if (task) {
          try {task->promise.set_value(task->call());}
          catch (...) {task->promise.set_exception(std::current_exception());}
          continue;
        }
        if (!cycle()) break;
        next += period;
        if (next < Clock::now()) next = Clock::now() + period;
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        running_ = false;
        cancel(normal_); cancel(urgent_);
      }
      finish();
    });
  }
  std::future<bool> submit(std::function<bool()> call, bool urgent = false)
  {
    auto task = std::make_shared<Task>();
    task->call = std::move(call);
    auto future = task->promise.get_future();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      auto & slot = urgent ? urgent_ : normal_;
      if (!running_ || slot) {task->promise.set_value(false); return future;}
      if (urgent) cancel(normal_);
      slot = std::move(task);
    }
    cv_.notify_all();
    return future;
  }
  bool urgent_pending() {std::lock_guard<std::mutex> lock(mutex_); return static_cast<bool>(urgent_);}
  bool owns_thread() const {return thread_.get_id() == std::this_thread::get_id();}
  void request_stop()
  {
    {std::lock_guard<std::mutex> lock(mutex_); running_ = false;}
    cv_.notify_all();
  }
  void stop() {request_stop(); if (thread_.joinable()) thread_.join();}
private:
  struct Task {std::function<bool()> call; std::promise<bool> promise;};
  static void cancel(std::shared_ptr<Task> & task)
  {if (task) {task->promise.set_value(false); task.reset();}}
  std::mutex mutex_;
  std::condition_variable cv_;
  std::thread thread_;
  bool running_{false};
  std::shared_ptr<Task> normal_, urgent_;
};
}  // namespace robot_hardware
