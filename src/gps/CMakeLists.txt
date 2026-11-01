cmake_minimum_required(VERSION 3.10)
project(gps)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)


# 执行文件编译
add_executable(gps_node
  src/gps_node.cpp
  src/Nav/PurePursuit.cpp
  src/Sensor/Sensor.cpp
)
ament_target_dependencies(gps_node
  rclcpp
  nav_msgs
  sensor_msgs
  behaviortree_cpp
  ament_index_cpp
)

#本地库文件
add_library(SerialPort STATIC
src/SerialPort/usart.cpp
)
target_include_directories(SerialPort PUBLIC
src
src/SerialPort
)

target_link_libraries(gps_node behaviortree_cpp SerialPort)
install(TARGETS
  gps_node
  DESTINATION lib/${PROJECT_NAME}
)

# ---- 测试底盘：用 radar_msg + 30 字节协议与下位机通信 ----------------------
# 独立可执行文件，离线自测不需要硬件；不影响 gps_node
add_executable(test_chassis
  src/Test/test.cpp
  src/Test/TestChassis.cpp
)
target_include_directories(test_chassis PRIVATE
  src
  src/Test
)
target_link_libraries(test_chassis SerialPort)
install(TARGETS
  test_chassis
  DESTINATION lib/${PROJECT_NAME}
)
# Install behavior tree XML
install(
  FILES tree.xml
  DESTINATION share/${PROJECT_NAME}
)
# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
