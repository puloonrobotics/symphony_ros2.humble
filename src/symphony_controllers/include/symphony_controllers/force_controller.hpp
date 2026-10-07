#ifndef SYMPHONY_CONTROLLERS__FORCE_CONTROLLER_HPP_
#define SYMPHONY_CONTROLLERS__FORCE_CONTROLLER_HPP_

#include <array>
#include <atomic>
#include <cstdint>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "symphony_controllers/visibility_control.h"
#include "symphony_msgs/srv/set_force_parameters.hpp"
#include "symphony_msgs/srv/set_gravity.hpp"
#include "symphony_msgs/srv/set_tool_info.hpp"
#include "symphony_msgs/srv/start_force_control.hpp"

namespace symphony_controllers
{
class ForceController : public controller_interface::ControllerInterface
{
public:
  SYMPHONY_CONTROLLERS_PUBLIC
  ForceController() = default;

  SYMPHONY_CONTROLLERS_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  SYMPHONY_CONTROLLERS_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  SYMPHONY_CONTROLLERS_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  SYMPHONY_CONTROLLERS_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  SYMPHONY_CONTROLLERS_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  SYMPHONY_CONTROLLERS_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  SYMPHONY_CONTROLLERS_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  const hardware_interface::LoanedStateInterface * find_state(const std::string & name) const;
  hardware_interface::LoanedCommandInterface * find_command(const std::string & name);
  void clear_interface_cache();

  const hardware_interface::LoanedStateInterface * force_enabled_state_{nullptr};
  std::array<hardware_interface::LoanedCommandInterface *, 50> cmds_{};

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr force_enabled_pub_;
  rclcpp::Service<symphony_msgs::srv::SetToolInfo>::SharedPtr set_tool_info_srv_;
  rclcpp::Service<symphony_msgs::srv::SetGravity>::SharedPtr set_gravity_vector_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_force_bias_srv_;
  rclcpp::Service<symphony_msgs::srv::SetForceParameters>::SharedPtr set_force_parameters_srv_;
  rclcpp::Service<symphony_msgs::srv::StartForceControl>::SharedPtr start_force_control_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_force_control_srv_;

  std::atomic<bool> active_{false};
  std::atomic<uint64_t> payload_seq_{0};
  std::array<std::atomic<double>, 7> payload_{};
  std::atomic<uint64_t> gravity_seq_{0};
  std::array<std::atomic<double>, 3> gravity_{};
  std::atomic<uint64_t> bias_seq_{0};
  std::atomic<uint64_t> force_seq_{0};
  std::atomic<double> force_enable_{0.0};
  std::array<std::atomic<double>, 6> wrench_{};
  std::atomic<uint64_t> param_seq_{0};
  std::atomic<double> set_imp_mass_{0.0};
  std::array<std::atomic<double>, 6> imp_mass_{};
  std::atomic<double> set_imp_stiffness_{0.0};
  std::array<std::atomic<double>, 6> imp_stiffness_{};
  std::atomic<double> set_imp_damping_{0.0};
  std::array<std::atomic<double>, 6> imp_damping_{};
  std::atomic<double> set_compliance_{0.0};
  std::array<std::atomic<double>, 6> compliance_{};
};

}  // namespace symphony_controllers

#endif  // SYMPHONY_CONTROLLERS__FORCE_CONTROLLER_HPP_
