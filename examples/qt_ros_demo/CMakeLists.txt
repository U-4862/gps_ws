cmake_minimum_required(VERSION 3.16)
project(qt_ros_demo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)

add_executable(qt_ros_demo 06_ros2_pose.cpp)
target_link_libraries(qt_ros_demo Qt5::Widgets)
ament_target_dependencies(qt_ros_demo rclcpp nav_msgs)

install(TARGETS qt_ros_demo DESTINATION lib/${PROJECT_NAME})

ament_package()
