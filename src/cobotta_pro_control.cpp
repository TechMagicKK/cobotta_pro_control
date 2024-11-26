#include "cobotta_pro_control/cobotta_pro_control.hpp"

CobottaProControl::CobottaProControl(const std::string & node_name, bool intra_process_comms)
: rclcpp_lifecycle::LifecycleNode(
    node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
{}

void CobottaProControl::initialize()
{
  this->initialize_controller();
  this->initialize_ros_services();
}

void CobottaProControl::initialize_controller()
{
  try {
    rc9_controller_ = std::make_shared<RC9Controller>(shared_from_this());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize RC9Controller: %s", e.what());
    throw;
  }
}

void CobottaProControl::initialize_ros_services() 
{
    take_arm_srv_ = this->create_service<cobotta_pro_control_interfaces::srv::TakeArm>(
      "take_arm", 
      std::bind(&CobottaProControl::take_arm, this, std::placeholders::_1, std::placeholders::_2));
    set_motor_srv_ = this->create_service<cobotta_pro_control_interfaces::srv::SetMotor>(
      "set_motor", 
      std::bind(&CobottaProControl::set_motor, this, std::placeholders::_1, std::placeholders::_2));
}

void CobottaProControl::take_arm(
    const std::shared_ptr<cobotta_pro_control_interfaces::srv::TakeArm::Request> request, 
    std::shared_ptr<cobotta_pro_control_interfaces::srv::TakeArm::Response> response) 
{
// pass
}

void CobottaProControl::set_motor(
    const std::shared_ptr<cobotta_pro_control_interfaces::srv::SetMotor::Request> request, 
    std::shared_ptr<cobotta_pro_control_interfaces::srv::SetMotor::Response> response) 
{
// pass
}

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn
CobottaProControl::on_configure(const rclcpp_lifecycle::State &)
{
  rc9_controller_->get_controller_handler();
  rc9_controller_->get_robot_handler();
  RCLCPP_INFO(get_logger(), "on_configure() is called.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn
CobottaProControl::on_activate(const rclcpp_lifecycle::State &)
{
  try {
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

  RCLCPP_INFO(node->get_logger(), "Launched cobotta_pro_control node...");
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
