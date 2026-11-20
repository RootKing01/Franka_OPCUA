#include <gtest/gtest.h>

#include <chrono>

#include "franka_opcua_bridge/performance_monitor.hpp"

using franka_opcua_bridge::PerformanceMonitor;
using franka_opcua_bridge::ScopedPerformanceMeasurement;

TEST(PerformanceMonitorTest, AggregatesCountsFailuresAndDurations)
{
  PerformanceMonitor monitor;
  monitor.record("opcua.read_value", std::chrono::nanoseconds(10), true);
  monitor.record("opcua.read_value", std::chrono::nanoseconds(25), false);

  const auto snapshot = monitor.takeSnapshot();

  ASSERT_EQ(snapshot.size(), 1u);
  EXPECT_EQ(snapshot.at("opcua.read_value").count, 2u);
  EXPECT_EQ(snapshot.at("opcua.read_value").failures, 1u);
  EXPECT_EQ(snapshot.at("opcua.read_value").total_duration, std::chrono::nanoseconds(35));
  EXPECT_EQ(snapshot.at("opcua.read_value").max_duration, std::chrono::nanoseconds(25));
  EXPECT_TRUE(monitor.takeSnapshot().empty());
}

TEST(PerformanceMonitorTest, ScopedMeasurementRecordsFailureByDefault)
{
  auto monitor = std::make_shared<PerformanceMonitor>();
  {
    ScopedPerformanceMeasurement measurement(monitor, "opcua.call_method.StopTask");
  }

  const auto snapshot = monitor->takeSnapshot();
  ASSERT_EQ(snapshot.size(), 1u);
  EXPECT_EQ(snapshot.at("opcua.call_method.StopTask").count, 1u);
  EXPECT_EQ(snapshot.at("opcua.call_method.StopTask").failures, 1u);
}
