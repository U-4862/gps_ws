#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "map/map.h"
#include "SerialPort/usart.hpp"
#include "Chassis/SerialChassis.hpp"
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>






int main(int argc ,char** argv)
{
    rclcpp.init(argc , argv);
    auto app_node = std::make_shared<rclcpp::Node>("gps_simple_node");


}