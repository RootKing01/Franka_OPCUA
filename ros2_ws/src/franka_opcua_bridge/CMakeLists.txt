cmake_minimum_required(VERSION 3.8)
project(franka_opcua_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(franka_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)


include_directories(include)

add_executable(franka_bridge_node
  src/franka_bridge_node.cpp
  src/franka_robot.cpp
  src/opcua_client.cpp
  src/franka_command_manager.cpp
  src/opcua_worker.cpp)

ament_target_dependencies(franka_bridge_node franka_interfaces rclcpp geometry_msgs sensor_msgs)


find_package(open62541 REQUIRED)
ua_generate_datatypes(
  NAME "opc_ua_service_types"
  TARGET_SUFFIX "opc-ua-service-types"
  FILE_CSV "${CMAKE_CURRENT_SOURCE_DIR}/type_generate/opc_ua_service.csv"
  FILES_BSD "${CMAKE_CURRENT_SOURCE_DIR}/type_generate/opc_ua_service_types.bsd"
  NAMESPACE_MAP "2:http://franka.de/opcua/"
)

target_sources(franka_bridge_node PRIVATE
  ${PROJECT_BINARY_DIR}/src_generated/open62541/opc_ua_service_types_generated.c
)

ament_target_dependencies(franka_bridge_node franka_interfaces rclcpp geometry_msgs sensor_msgs)
target_link_libraries(franka_bridge_node
  Eigen3::Eigen
  Threads::Threads
  open62541::open62541)
add_dependencies(franka_bridge_node open62541-generator-opc-ua-service-types)
target_include_directories(franka_bridge_node PRIVATE
    ${PROJECT_BINARY_DIR}/src_generated/open62541
    /opt/open62541/src
)
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
    src/fake_test_version/fake_robot.cpp)
  target_include_directories(test_fake_robot PUBLIC include)
  ament_target_dependencies(test_fake_robot geometry_msgs)

  ament_add_gtest(test_robot_state_cache
    test/test_robot_state_cache.cpp)
  target_include_directories(test_robot_state_cache PUBLIC include)
  ament_target_dependencies(test_robot_state_cache geometry_msgs)

  ament_add_gtest(test_fake_protocol_client
    test/test_fake_protocol_client.cpp
    src/fake_test_version/fake_protocol_client.cpp)
  target_include_directories(test_fake_protocol_client PUBLIC include)

  ament_add_gtest(test_franka_command_manager
    test/test_franka_command_manager.cpp
    src/franka_command_manager.cpp)
  target_include_directories(test_franka_command_manager PUBLIC include)
  ament_target_dependencies(test_franka_command_manager geometry_msgs)

  ament_add_gtest(test_opcua_worker
    test/test_opcua_worker.cpp
    src/opcua_worker.cpp
    src/franka_command_manager.cpp)
  target_include_directories(test_opcua_worker PUBLIC include)
  ament_target_dependencies(test_opcua_worker rclcpp geometry_msgs)
  target_link_libraries(test_opcua_worker Threads::Threads)

  ament_add_gtest(test_franka
    test/test_franka.cpp
    src/franka_robot.cpp
    src/fake_test_version/fake_protocol_client.cpp)
  target_include_directories(test_franka PUBLIC include)
  target_include_directories(test_franka PUBLIC include /usr/include/eigen3)
  ament_target_dependencies(test_franka geometry_msgs Eigen3)
endif()

ament_package()