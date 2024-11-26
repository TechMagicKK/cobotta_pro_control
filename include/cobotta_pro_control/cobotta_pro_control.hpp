#ifndef COBOTTA_PRO_CONTROL_HPP
#define COBOTTA_PRO_CONTROL_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"
#include "cobotta_pro_control/rc9_driver.hpp"

// CobottaProControlクラス
class CobottaProControl : public rclcpp_lifecycle::LifecycleNode
{
public:
    // コンストラクタ
    explicit CobottaProControl(const std::string & node_name, bool intra_process_comms = false);
    void initialize();

    // ライフサイクルコールバック
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &);

private:
    std::shared_ptr<rclcpp::TimerBase> timer_; // タイマーオブジェクト
    std::shared_ptr<RC9Driver> rc9_driver_;   // RC9ドライバ
};

#endif // COBOTTA_PRO_CONTROL_HPP
