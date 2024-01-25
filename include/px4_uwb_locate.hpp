#ifndef _PX4_UWB_LOCATE_HPP_
#define _PX4_UWB_LOCATE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>

class PX4UwbLocate : public rclcpp::Node
{
public:
    PX4UwbLocate();

private:
    void uwb_position_cb(const geometry_msgs::msg::Point msg);
    void timer_callback();

    std::string tag_name, uas_name;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr uwb_position_sub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vision_pose_pub;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::Point position;
};

#endif