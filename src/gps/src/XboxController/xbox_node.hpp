#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>


class XboxNode : rclcpp::Node 
{
public:
    explicit XboxNode(
        const std::string& stick_name = "/joy"
    );




    
}