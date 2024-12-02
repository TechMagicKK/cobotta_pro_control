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
    var.value = "CaoProv.DENSO.VRC9";
    vnt_args.push_back(var);

    var.vt = VT_BSTR;
    var.value = "192.168.0.102";
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
    var.value = "109";  // "" string type, important!!
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
        RCLCPP_WARN(node_->get_logger(), "Response is empty. Setting robot handler to -1.");
        return;
    } else {
        const auto &res = response.value();  // または *response
        if (res.hresult != 0) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to get robot handler. %d", res.hresult);
            robot_handler_ = -1;
        } else {
            robot_handler_ = static_cast<int16_t>(std::stoi(res.vnt_ret.value));
            RCLCPP_INFO(node_->get_logger(), "Robot handler: %d", robot_handler_);
        }
    }
}

void RC9Controller::slave_move(const float* goal)
{
    if (!this->change_mode(1))
    {
        return;
    }

    std::vector<bcap_service_interfaces::msg::Variant> vnt_args;

    bcap_service_interfaces::msg::Variant var;
    var.vt = VT_UI4;
    var.value = robot_handler_;
    vnt_args.push_back(var);

    var.vt = VT_BSTR;
    var.value = "slvMove";
    vnt_args.push_back(var);

    // vector_to_string
    std::string goal_pos;
    size_t size = sizeof(goal) / sizeof(goal[0]);
    for (size_t i = 0; i < size; ++i) {
        goal_pos += std::to_string(goal[i]);
        if (i < size - 1) {
            goal_pos += ", ";
        }
    }
    var.vt = VT_R8;
    var.value = goal_pos;
    vnt_args.push_back(var);

    auto response = this->ros_client_.call_bcap_service(ID_ROBOT_EXECUTE, vnt_args);
    if (!response.has_value()) {
        RCLCPP_WARN(node_->get_logger(), "Response is empty.");
        return;
    } else {
        const auto &res = response.value();  // または *response
        if (res.hresult != 0) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to move in slave mode.");
        } else {
            RCLCPP_INFO(node_->get_logger(), "Moved: %s", res.vnt_ret.value);
        }
    }

    if (!this->change_mode(0))
    {
        return;
    }
}

bool RC9Controller::change_mode(uint16_t mode)
{
    auto response = ros_client_.change_mode(mode);
    if (!response.has_value()) {
        RCLCPP_WARN(node_->get_logger(), "Response is empty");
        return false;
    } else {
        const auto &res = response.value();
        if (res.hresult != 0) {
            RCLCPP_ERROR(node_->get_logger(), "Failed changing to slave mode.");
            return false;
        } else {
            RCLCPP_INFO(node_->get_logger(), "Changed to slave mode.");
        }
    }
    return true;
}

bool RC9Controller::set_motor(bool enable)
{
// pass
}

bool RC9Controller::take_arm(uint16_t arm_group)
{
// pass
}
