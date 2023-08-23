cmake_minimum_required(VERSION 3.5)
project(robomaster_can_ros_bridge)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(emergency_stop_msgs REQUIRED)
find_package(robomaster_msgs REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(${CMAKE_SOURCE_DIR}/robomaster_sdk_can/inc)

add_executable(controlling src/controlling.cpp)
add_executable(sensing src/sensing.cpp)

ament_target_dependencies(controlling rclcpp std_msgs geometry_msgs robomaster_msgs emergency_stop_msgs)
ament_target_dependencies(sensing rclcpp std_msgs geometry_msgs sensor_msgs robomaster_msgs)

target_link_libraries(sensing Eigen3::Eigen)

install(TARGETS
  controlling
  sensing
  DESTINATION lib/${PROJECT_NAME})

ament_package()
