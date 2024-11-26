#ifndef RC9_DRIVER_HPP
#define RC9_DRIVER_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "bcap_service_interfaces/srv/bcap.hpp"
#include "bcap_service_interfaces/msg/variant.hpp"
#include "cobotta_pro_control/bcap_client.hpp"


class RC9Driver
{
public:
    explicit RC9Driver(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node);
    uint16_t get_controller_handle();

private:
    uint16_t controller_handler_;
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
    BCapClient bcap_client_;
    std::shared_ptr<rclcpp::TimerBase> timer_;
};

#endif // RC9_DRIVER_HPP