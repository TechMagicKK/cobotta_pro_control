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

#include "cobotta_pro_control/bcap_client.hpp"


class RC9Controller
{
public:
    explicit RC9Controller(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node);
    void get_controller_handler();
    void get_robot_identifier();
    bool take_arm(uint16_t arm_group);
    bool set_motor(bool enable);

private:
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
    std::shared_ptr<rclcpp::TimerBase> timer_;

    int16_t controller_handler_;
    std::string robot_indentifier_;
    BCapClient bcap_client_;
};

#endif // RC9_CONTROLLER_HPP