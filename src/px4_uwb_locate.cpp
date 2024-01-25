#include <px4_uwb_locate.hpp>
PX4UwbLocate::PX4UwbLocate() : Node("px4_uwb_locate_node")
{

    this->declare_parameter("tag_name", "x500_0");
    this->declare_parameter("uas_name", "uas_1");
    tag_name = this->get_parameter("tag_name").get_parameter_value().get<std::string>();
    uas_name = this->get_parameter("uas_name").get_parameter_value().get<std::string>();
    RCLCPP_INFO(this->get_logger(), "tag name : %s", tag_name.c_str());
    RCLCPP_INFO(this->get_logger(), "uas name : %s", uas_name.c_str());

    std::string uwb_pose_topic_name = "/uwbLocationRes/" + tag_name;
    std::string vision_pose_publisher_name = "/mavros/" + uas_name + "/vision_pose/pose";
    RCLCPP_INFO(this->get_logger(), "uwb pose topic name : %s", uwb_pose_topic_name.c_str());
    RCLCPP_INFO(this->get_logger(), "vision pose publisher name : %s", vision_pose_publisher_name.c_str());

    uwb_position_sub =
        this->create_subscription<geometry_msgs::msg::Point>(
            uwb_pose_topic_name, 10, std::bind(&PX4UwbLocate::uwb_position_cb, this, std::placeholders::_1));
    vision_pose_pub =
        this->create_publisher<geometry_msgs::msg::PoseStamped>(vision_pose_publisher_name, 10);
    double dt = 0.02;    
    timer_ =
        this->create_wall_timer(
            std::chrono::milliseconds((int)(dt * 1000)),
            std::bind(&PX4UwbLocate::timer_callback, this));
}
void PX4UwbLocate::uwb_position_cb(const geometry_msgs::msg::Point msg)
{
    position = msg;

}
void PX4UwbLocate::timer_callback()
{
    geometry_msgs::msg::PoseStamped vision_msg;

    vision_msg.pose.position.x = position.x;
    vision_msg.pose.position.y = position.y;
    vision_msg.pose.position.z = position.z;
    
    vision_msg.header.stamp = rclcpp::Clock().now();

    vision_pose_pub->publish(vision_msg);
}
int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PX4UwbLocate>());
    rclcpp::shutdown();

    return 0;
}