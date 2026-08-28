cmake_minimum_required(VERSION 3.8)
project(franka_opcua_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(franka_bridge_node src/franka_bridge_node.cpp)
ament_target_dependencies(franka_bridge_node rclcpp geometry_msgs sensor_msgs std_msgs)

install(TARGETS
  franka_bridge_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_fake_robot
    test/test_fake_robot.cpp
    src/fake_robot.cpp)
  target_include_directories(test_fake_robot PUBLIC include)
  ament_target_dependencies(test_fake_robot geometry_msgs)
endif()

ament_package()