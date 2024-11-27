#ifndef RC9_CONTROLLER_HPP
#define RC9_CONTROLLER_HPP

#include <string>
#include <stdexcept>
#include <cstdint>
#include <cstdlib>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "bcap_service_interfaces/srv/bcap.hpp"
#include "bcap_service_interfaces/msg/variant.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "cobotta_pro_control/ros_client.hpp"


class RC9Controller
{
public:
    explicit RC9Controller(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node);
    void get_controller_handler();
    void get_robot_handler();
    bool take_arm(uint16_t arm_group);
    bool set_motor(bool enable);
    void slave_move(const float* goal);
    bool change_mode(uint16_t mode);

private:
    void current_mode_cb_(std_msgs::msg::Int32::ConstSharedPtr msg);

    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
    std::shared_ptr<rclcpp::TimerBase> timer_;

    ROSClient ros_client_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_sub_;

    int16_t controller_handler_;
    int16_t robot_handler_;
    std::string current_mode_;
};

#endif // RC9_CONTROLLER_HPP