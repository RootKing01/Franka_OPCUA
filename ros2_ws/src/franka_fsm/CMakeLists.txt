cmake_minimum_required(VERSION 3.8)
project(franka_fsm)

find_package(ament_cmake REQUIRED)
find_package(franka_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(franka_fsm_node
	src/main.cpp
	src/franka_fsm_node.cpp)

target_include_directories(franka_fsm_node PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}/include)

ament_target_dependencies(franka_fsm_node
	franka_interfaces
	geometry_msgs
	rclcpp)

install(TARGETS franka_fsm_node
	DESTINATION lib/${PROJECT_NAME})

ament_package()
