#ifndef COBOTTA_PRO_CONTROL_HPP
#define COBOTTA_PRO_CONTROL_HPP

#include "cobotta_pro_control_interfaces/srv/take_arm.hpp"
#include "cobotta_pro_control_interfaces/srv/set_motor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"
#include "cobotta_pro_control/rc9_controller.hpp"

class CobottaProControl : public rclcpp_lifecycle::LifecycleNode
{
public:
    explicit CobottaProControl(const std::string & node_name, bool intra_process_comms = false);
    void initialize();

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &);

    void take_arm(
        const std::shared_ptr<cobotta_pro_control_interfaces::srv::TakeArm::Request> request,
        std::shared_ptr<cobotta_pro_control_interfaces::srv::TakeArm::Response> response);
    void set_motor(
        const std::shared_ptr<cobotta_pro_control_interfaces::srv::SetMotor::Request> request,
        std::shared_ptr<cobotta_pro_control_interfaces::srv::SetMotor::Response> response);

private:
    std::shared_ptr<RC9Controller> rc9_controller_;
    rclcpp::Service<cobotta_pro_control_interfaces::srv::TakeArm>::SharedPtr take_arm_srv_;
    rclcpp::Service<cobotta_pro_control_interfaces::srv::SetMotor>::SharedPtr set_motor_srv_;
    
    void initialize_controller();
    void initialize_ros_services();
};

#endif // COBOTTA_PRO_CONTROL_HPP
