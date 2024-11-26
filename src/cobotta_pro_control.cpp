#include "cobotta_pro_control/cobotta_pro_control.hpp"

CobottaProControl::CobottaProControl(const std::string & node_name, bool intra_process_comms)
: rclcpp_lifecycle::LifecycleNode(
    node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
{}

void CobottaProControl::initialize()
{
  try {
    rc9_driver_ = std::make_shared<RC9Driver>(shared_from_this());
    // srv_ = this->create_service<TakeArm>


  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize RC9Driver: %s", e.what());
    throw;
  }
}

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn
CobottaProControl::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "on_configure() is called.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn
CobottaProControl::on_activate(const rclcpp_lifecycle::State &)
{
  try {
    uint16_t controller_handle = rc9_driver_->get_controller_handle();
    RCLCPP_INFO(get_logger(), "RC9 controller handle: %u", controller_handle);
    RCLCPP_INFO(get_logger(), "on_activate() is called.");
    return CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to activate node: %s", e.what());
    return CallbackReturn::FAILURE;
  }
}

CallbackReturn
CobottaProControl::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "on_deactivate() is called.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn
CobottaProControl::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "on_cleanup() is called.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn
CobottaProControl::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "on_shutdown() is called.");
  return CallbackReturn::SUCCESS;
}

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<CobottaProControl>("cobotta_pro_control");
  node->initialize();
  executor.add_node(node->get_node_base_interface());

  // スピン開始
  RCLCPP_INFO(node->get_logger(), "Launched cobotta_pro_control node...");
  executor.spin();

  // シャットダウン処理
  rclcpp::shutdown();
  return 0;
}
