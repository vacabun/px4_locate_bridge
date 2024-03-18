#ifndef _PX4_UWB_LOCATE_HPP_
#define _PX4_UWB_LOCATE_HPP_

class PX4LocateBridge : public rclcpp::Node
{
public:
    PX4LocateBridge();

private:
    void uwb_position_cb(const geometry_msgs::msg::Point msg);
    void timestampCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) void timer_callback();
    void timer_callback();

    std::string tag_name, uas_name;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr uwb_position_subscriber_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr visual_odometry_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr timestamp_subscriber_;
    geometry_msgs::msg::Point position;
    uint64_t last_px4_timestamp_;
    rclcpp::Time last_ros_time_;
};

#endif