cmake_minimum_required(VERSION 3.10)
project(gps)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#本地库文件
add_library(SerialPort STATIC
src/SerialPort/usart.cpp
)
target_include_directories(SerialPort PUBLIC 
src/SerialPort
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)


# 执行文件编译
add_executable(gps_node src/gps_node.cpp)
ament_target_dependencies(gps_node
  rclcpp
  nav_msgs
  sensor_msgs
  behaviortree_cpp
)

target_link_libraries(gps_node behaviortree_cpp SerialPort)
install(TARGETS
  gps_node
  DESTINATION lib/${PROJECT_NAME}
)
# Install behavior tree XML
install(
  FILES tree.xml
  DESTINATION share/${PROJECT_NAME}
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
