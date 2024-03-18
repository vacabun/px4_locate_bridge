#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_locate_bridge.hpp>
using namespace std::chrono_literals;

PX4LocateBridge::PX4LocateBridge() : Node("px4_uwb_locate_node")
{

    this->declare_parameter("tag_name", "x500_0");
    tag_name = this->get_parameter("tag_name").get_parameter_value().get<std::string>();

    std::string uwb_pose_topic_name = "/uwbLocationRes/" + tag_name;
    std::string visual_odometry_publisher_name = "/" + tag_name + "/fmu/in/vehicle_visual_odometry";
    std::string vehicle_status_topic_name = "/" + tag_name + "/fmu/out/vehicle_status";
    RCLCPP_INFO(this->get_logger(), "tag name : %s", tag_name.c_str());
    RCLCPP_INFO(this->get_logger(), "uwb pose topic name : %s", uwb_pose_topic_name.c_str());
    RCLCPP_INFO(this->get_logger(), "vision pose publisher name : %s", visual_odometry_publisher_name.c_str());
    RCLCPP_INFO(this->get_logger(), "vehicle status topic name : %s", vehicle_status_topic_name.c_str());

    timestamp_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(vehicle_status_topic_name, 10, std::bind(&PX4LocateBridge::timestampCallback, this, std::placeholders::_1));

    uwb_position_subscriber_ = this->create_subscription<geometry_msgs::msg::Point>(uwb_pose_topic_name, 10, std::bind(&PX4LocateBridge::uwb_position_cb, this, std::placeholders::_1));
    visual_odometry_publisher_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(visual_odometry_publisher_name, 10);

    timer_ = this->create_wall_timer(20ms, std::bind(&PX4LocateBridge::timer_callback, this));
}

void PX4LocateBridge::uwb_position_cb(const geometry_msgs::msg::Point msg)
{
    position = msg;
}

void PX4LocateBridge::timer_callback()
{
    px4_msgs::msg::VehicleOdometry message;

    message.position = {(float)position.x, (float)position.y, (float)position.z};

    // 根据 ROS 时间的变化来调整 PX4 时间戳
    rclcpp::Time now = this->now();
    rclcpp::Duration elapsed = now - last_ros_time_;
    uint64_t adjusted_timestamp = last_px4_timestamp_ + elapsed.nanoseconds() / 1000; // 将纳秒转换为微秒
    message.timestamp = adjusted_timestamp;

    visual_odometry_publisher_->publish(message);
}
void PX4LocateBridge::timestampCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
    last_px4_timestamp_ = msg->timestamp; // 获取 PX4 时间戳
    last_ros_time_ = this->now();         // 获取当前的 ROS 时间
}
int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PX4LocateBridge>());
    rclcpp::shutdown();

    return 0;
}