#include "chrono"
#include "rclcpp/rclcpp.hpp"
#include "cobotta_pro_control/rc9_driver.hpp"
#include "bcap_service_interfaces/srv/bcap.hpp"
#include "bcap_service_interfaces/msg/variant.hpp"

RC9Driver::RC9Driver(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node)
: node_(node), bcap_client_(node)
{}

uint16_t RC9Driver::get_controller_handle()
{
    std::vector<bcap_service_interfaces::msg::Variant> vnt_args;

    bcap_service_interfaces::msg::Variant var;
    var.vt = 8;  // VT_BSTR
    var.value = "b-CAP";
    vnt_args.push_back(var);

    var.vt = 8;  // VT_BSTR
    var.value = "CaoProv.DENSO.VRC";
    vnt_args.push_back(var);

    var.vt = 8;  // VT_BSTR
    var.value = "localhost";
    vnt_args.push_back(var);

    var.vt = 8;  // VT_BSTR
    var.value = "";  // 空文字列
    vnt_args.push_back(var);

    auto response = this->bcap_client_.call_bcap_service(3, vnt_args);

    controller_handler_ = 0;
    if (response.has_value()) {
        // オブジェクトを取得
        const auto &res = response.value();  // または *response

        if (res.hresult != 0) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to get controller handle");
            controller_handler_ = 0;
        } else {
            controller_handler_ = 2;
            // controller_handler_ = res.vnt_ret.value;
            RCLCPP_INFO(node_->get_logger(), "Controller handle: %d", controller_handler_);
        }
    }
    return controller_handler_;
}
