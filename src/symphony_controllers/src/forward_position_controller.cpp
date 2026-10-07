#include "symphony_controllers/forward_position_controller.hpp"

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "symphony_controllers/loaned_iface.hpp"

namespace symphony_controllers
{

controller_interface::InterfaceConfiguration
ForwardPositionController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = {
    "joint0/position",
    "joint1/position",
    "joint2/position",
    "joint3/position",
    "joint4/position",
    "joint5/position",
    "servo_q/seq",
  };
  return config;
}

controller_interface::InterfaceConfiguration
ForwardPositionController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::CallbackReturn ForwardPositionController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>{});
    auto_declare<std::string>("interface_name", "position");
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "forward_position_controller on_init: %s", ex.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ForwardPositionController::on_configure(
  const rclcpp_lifecycle::State &)
{
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  interface_name_ = get_node()->get_parameter("interface_name").as_string();
  const std::vector<std::string> expected = {
    "joint0", "joint1", "joint2", "joint3", "joint4", "joint5"};
  if (joint_names_ != expected || interface_name_ != "position") {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "forward_position_controller needs joint0..joint5 position");
    return controller_interface::CallbackReturn::ERROR;
  }

  cmd_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    "~/commands", rclcpp::QoS(10),
    [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
      if (!active_.load() || msg->data.size() != target_.size()) {
        return;
      }
      const uint64_t epoch = accept_epoch_.load();
      write_seq_.fetch_add(1);
      for (size_t i = 0; i < target_.size(); ++i) {
        target_[i].store(msg->data[i]);
      }
      if (active_.load() && accept_epoch_.load() == epoch) {
        cmd_seq_.fetch_add(1);
      }
      write_seq_.fetch_add(1);
    });
  return controller_interface::CallbackReturn::SUCCESS;
}

void ForwardPositionController::write_command(
  const std::array<double, 6> & q, uint64_t seq)
{
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const std::string name = joint_names_[i] + "/" + interface_name_;
    for (auto & iface : command_interfaces_) {
      if (iface.get_name() == name) {
        set_command(iface, q[i]);
        break;
      }
    }
  }
  for (auto & iface : command_interfaces_) {
    if (iface.get_name() == "servo_q/seq") {
      set_command(iface, static_cast<double>(seq));
      break;
    }
  }
}

controller_interface::CallbackReturn ForwardPositionController::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (command_interfaces_.size() != target_.size() + 1) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "forward_position_controller expected %zu command interfaces, got %zu",
      target_.size() + 1, command_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  accept_epoch_.fetch_add(1);
  applied_seq_.store(cmd_seq_.load());
  active_.store(true);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ForwardPositionController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  active_.store(false);
  accept_epoch_.fetch_add(1);
  applied_seq_.store(cmd_seq_.load());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ForwardPositionController::update(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_.load()) {
    return controller_interface::return_type::OK;
  }
  std::array<double, 6> q{};
  uint64_t seq = 0;
  bool stable = false;
  for (int attempt = 0; attempt < 3 && !stable; ++attempt) {
    const uint64_t s1 = write_seq_.load();
    if (s1 & 1ULL) {
      return controller_interface::return_type::OK;
    }
    for (size_t i = 0; i < q.size(); ++i) {
      q[i] = target_[i].load();
    }
    seq = cmd_seq_.load();
    stable = write_seq_.load() == s1;
  }
  if (!stable || seq == applied_seq_.load()) {
    return controller_interface::return_type::OK;
  }
  write_command(q, seq);
  if (cmd_seq_.load() == seq && (write_seq_.load() & 1ULL) == 0ULL) {
    applied_seq_.store(seq);
  }
  return controller_interface::return_type::OK;
}

}  // namespace symphony_controllers

PLUGINLIB_EXPORT_CLASS(
  symphony_controllers::ForwardPositionController,
  controller_interface::ControllerInterface)
