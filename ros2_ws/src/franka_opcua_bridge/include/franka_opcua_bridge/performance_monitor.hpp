#ifndef FRANKA_OPCUA_BRIDGE__PERFORMANCE_MONITOR_HPP_
#define FRANKA_OPCUA_BRIDGE__PERFORMANCE_MONITOR_HPP_

#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>

namespace franka_opcua_bridge
{

struct PerformanceStats
{
  std::uint64_t count{0};
  std::uint64_t failures{0};
  std::chrono::nanoseconds total_duration{0};
  std::chrono::nanoseconds max_duration{0};
};

class PerformanceMonitor
{
public:
  void record(
    const std::string & operation,
    std::chrono::nanoseconds duration,
    bool success);

  std::map<std::string, PerformanceStats> takeSnapshot();

private:
  std::mutex mutex_;
  std::map<std::string, PerformanceStats> stats_;
};

class ScopedPerformanceMeasurement
{
public:
  ScopedPerformanceMeasurement(
    std::shared_ptr<PerformanceMonitor> monitor,
    std::string operation);
  ~ScopedPerformanceMeasurement();

  ScopedPerformanceMeasurement(const ScopedPerformanceMeasurement &) = delete;
  ScopedPerformanceMeasurement & operator=(const ScopedPerformanceMeasurement &) = delete;

  void setSuccess(bool success = true);

private:
  std::shared_ptr<PerformanceMonitor> monitor_;
  std::string operation_;
  std::chrono::steady_clock::time_point started_at_;
  bool success_{false};
};

}  // namespace franka_opcua_bridge

#endif
