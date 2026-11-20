#include "franka_opcua_bridge/performance_monitor.hpp"

#include <algorithm>
#include <utility>

namespace franka_opcua_bridge
{

void PerformanceMonitor::record(
  const std::string & operation,
  std::chrono::nanoseconds duration,
  bool success)
{
  duration = std::max(duration, std::chrono::nanoseconds::zero());
  std::lock_guard<std::mutex> lock(mutex_);
  auto & stats = stats_[operation];
  ++stats.count;
  if (!success) {
    ++stats.failures;
  }
  stats.total_duration += duration;
  stats.max_duration = std::max(stats.max_duration, duration);
}

std::map<std::string, PerformanceStats> PerformanceMonitor::takeSnapshot()
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::map<std::string, PerformanceStats> snapshot;
  snapshot.swap(stats_);
  return snapshot;
}

ScopedPerformanceMeasurement::ScopedPerformanceMeasurement(
  std::shared_ptr<PerformanceMonitor> monitor,
  std::string operation)
: monitor_(std::move(monitor)),
  operation_(std::move(operation)),
  started_at_(std::chrono::steady_clock::now())
{}

ScopedPerformanceMeasurement::~ScopedPerformanceMeasurement()
{
  if (monitor_) {
    monitor_->record(
      operation_, std::chrono::steady_clock::now() - started_at_, success_);
  }
}

void ScopedPerformanceMeasurement::setSuccess(bool success)
{
  success_ = success;
}

}  // namespace franka_opcua_bridge
