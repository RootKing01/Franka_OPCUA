#include <gtest/gtest.h>
#include <thread>
#include <vector>
#include "franka_opcua_bridge/robot_state_cache.hpp"

using franka_opcua_bridge::RobotStateCache;
using franka_opcua_bridge::RobotStatus;

TEST(RobotStateCacheTest, BasicReadWrite)
{
  RobotStateCache cache;
  RobotStatus s;
  s.is_running = true;
  s.active_task_name = "test";
  cache.updateStatus(s);

  auto result = cache.getStatus();
  EXPECT_TRUE(result.is_running);
  EXPECT_EQ(result.active_task_name, "test");
}

// Test reale di concorrenza: un thread scrive di continuo,
// molti thread leggono di continuo, per un intervallo di tempo.
// Se ci fosse una race condition, questo test crasherebbe
// o fallirebbe in modo intermittente (utile lanciarlo piu' volte).
TEST(RobotStateCacheTest, ConcurrentReadWriteDoesNotCrash)
{
  RobotStateCache cache;
  std::atomic<bool> stop{false};

  std::thread writer([&cache, &stop]() {
      int i = 0;
      while (!stop) {
        std::vector<double> joints = {
          static_cast<double>(i), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        cache.updateJointAngles(joints);
        i++;
      }
    });

  std::vector<std::thread> readers;
  for (int r = 0; r < 4; r++) {
    readers.emplace_back(
      [&cache, &stop]() {
        while (!stop) {
          auto joints = cache.getJointAngles();
          // nessuna asserzione sul valore: verifichiamo solo che
          // la lettura non corrompa memoria / non crashi
          (void)joints;
        }
      });
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  stop = true;
  writer.join();
  for (auto & t : readers) {
    t.join();
  }

  SUCCEED();  // se siamo arrivati qui senza crash, il test passa
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
