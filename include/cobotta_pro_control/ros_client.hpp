#include "chrono"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "bcap_service_interfaces/srv/bcap.hpp"
#include "bcap_service_interfaces/msg/variant.hpp"
#include "denso_robot_core_interfaces/srv/change_mode.hpp"


class ROSClient
{
public:
    explicit ROSClient(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node)
    : node_(node)
    {
        bcap_client_ = node_->create_client<bcap_service_interfaces::srv::Bcap>("bcap_service");
        change_mode_client_ = node->create_client<denso_robot_core_interfaces::srv::ChangeMode>("change_mode");
    }

    std::optional<bcap_service_interfaces::srv::Bcap::Response>
    call_bcap_service(int32_t func_id, const std::vector<bcap_service_interfaces::msg::Variant>& vnt_args)
    {
        while (!bcap_client_->wait_for_service(std::chrono::seconds(10))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return std::nullopt;
            }
            RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
        }

        auto request = std::make_shared<bcap_service_interfaces::srv::Bcap::Request>();
        
        request->func_id = func_id;
        request->vnt_args = vnt_args;

        auto future = bcap_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS) {
            const auto &response = future.get();
            RCLCPP_INFO(node_->get_logger(), "Service call succeeded, response received.");
            return *response;
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Failed to call service");
            return std::nullopt;
        }
    }

    std::optional<denso_robot_core_interfaces::srv::ChangeMode::Response>
    change_mode(int32_t mode)
    {
        while (!change_mode_client_->wait_for_service(std::chrono::seconds(10))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return std::nullopt;
            }
            RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
        }

        auto request = std::make_shared<denso_robot_core_interfaces::srv::ChangeMode::Request>();
        
        request->mode = mode;

        auto future = change_mode_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS) {
            const auto &response = future.get();
            RCLCPP_INFO(node_->get_logger(), "Service call succeeded, response received.");
            return *response;
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Failed to call service");
            return std::nullopt;
        }
    }

private:
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
    rclcpp::Client<bcap_service_interfaces::srv::Bcap>::SharedPtr bcap_client_;
    rclcpp::Client<denso_robot_core_interfaces::srv::ChangeMode>::SharedPtr change_mode_client_;
};
