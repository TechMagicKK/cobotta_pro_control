#include "cobotta_pro_control/rc9_controller.hpp"
#include "bcap_core/bcap_funcid.h"
#include "bcap_core/dn_common.h"

RC9Controller::RC9Controller(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node)
: node_(node), ros_client_(node)
{
    mode_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
        "CurMode", 
        10, 
        std::bind(&RC9Controller::current_mode_cb_, this, std::placeholders::_1)
    );
}

void RC9Controller::current_mode_cb_(std_msgs::msg::Int32::ConstSharedPtr msg)
{
    current_mode_ = msg->data;
    RCLCPP_INFO(node_->get_logger(), "Current mode: [%d]", current_mode_);
}

void RC9Controller::get_controller_handler()
{
    std::vector<bcap_service_interfaces::msg::Variant> vnt_args;

    bcap_service_interfaces::msg::Variant var;
    var.vt = VT_BSTR;
    var.value = "b-CAP";
    vnt_args.push_back(var);

    var.vt = VT_BSTR;
    var.value = "CaoProv.DENSO.VRC";
    vnt_args.push_back(var);

    var.vt = VT_BSTR;
    var.value = "localhost";
    vnt_args.push_back(var);

    var.vt = VT_BSTR;
    var.value = "";
    vnt_args.push_back(var);

    auto response = this->ros_client_.call_bcap_service(ID_CONTROLLER_CONNECT, vnt_args);
    if (!response.has_value()) {
        controller_handler_ = -1;
        RCLCPP_WARN(node_->get_logger(), "Response is empty. Setting controller_handler_ to -1.");
        return;
    } else {
        const auto &res = response.value();  // または *response
        if (res.hresult != 0) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to get controller handle");
            controller_handler_ = -1;
        } else {
            controller_handler_ = static_cast<int16_t>(std::stoi(res.vnt_ret.value));
            RCLCPP_INFO(node_->get_logger(), "Controller handle: %d", controller_handler_);
        }
    }
}

void RC9Controller::get_robot_handler()
{
    std::vector<bcap_service_interfaces::msg::Variant> vnt_args;

    bcap_service_interfaces::msg::Variant var;
    var.vt = VT_UI4;
    var.value = controller_handler_;
    vnt_args.push_back(var);

    var.vt = VT_BSTR;
    var.value = "Arm0";
    vnt_args.push_back(var);

    var.vt = VT_BSTR;
    var.value = "";
    vnt_args.push_back(var);

    auto response = this->ros_client_.call_bcap_service(ID_CONTROLLER_GETROBOT, vnt_args);
    if (!response.has_value()) {
        robot_handler_ = -1;
        RCLCPP_WARN(node_->get_logger(), "Response is empty. Setting robot indentifier to -1.");
        return;
    } else {
        const auto &res = response.value();  // または *response
        if (res.hresult != 0) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to get robot identifier.");
            robot_handler_ = -1;
        } else {
            robot_handler_ = static_cast<int16_t>(std::stoi(res.vnt_ret.value));
            RCLCPP_INFO(node_->get_logger(), "Robot Identifier: %d", robot_handler_);
        }
    }
}

bool RC9Controller::set_motor(bool enable)
{
// pass
}

bool RC9Controller::take_arm(uint16_t arm_group)
{
// pass
}
