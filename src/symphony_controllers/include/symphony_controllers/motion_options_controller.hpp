#ifndef SYMPHONY_CONTROLLERS__MOTION_OPTIONS_CONTROLLER_HPP_
#define SYMPHONY_CONTROLLERS__MOTION_OPTIONS_CONTROLLER_HPP_

#include <atomic>
#include <cstdint>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "symphony_controllers/visibility_control.h"
#include "symphony_msgs/srv/set_motion_options.hpp"

namespace symphony_controllers
{

class MotionOptionsController : public controller_interface::ControllerInterface
{
public:
  SYMPHONY_CONTROLLERS_PUBLIC
  MotionOptionsController() = default;

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
  hardware_interface::LoanedCommandInterface * find_command(const std::string & name);
  const hardware_interface::LoanedStateInterface * find_state(const std::string & name);
  void clear_interface_cache();
  void write_defaults();
  bool bind_interfaces();

  hardware_interface::LoanedCommandInterface * work_cmd_{nullptr};
  hardware_interface::LoanedCommandInterface * tool_cmd_{nullptr};
  hardware_interface::LoanedCommandInterface * rel_cmd_{nullptr};
  hardware_interface::LoanedCommandInterface * fixedspeed_cmd_{nullptr};
  hardware_interface::LoanedCommandInterface * cfg_cmd_{nullptr};
  hardware_interface::LoanedCommandInterface * coord_cmd_{nullptr};
  hardware_interface::LoanedCommandInterface * weaving_cmd_{nullptr};
  hardware_interface::LoanedCommandInterface * fixedorient_cmd_{nullptr};
  hardware_interface::LoanedCommandInterface * ext_cmd_{nullptr};
  hardware_interface::LoanedCommandInterface * tol_cmd_{nullptr};
  hardware_interface::LoanedCommandInterface * servo_relative_cmd_{nullptr};

  const hardware_interface::LoanedStateInterface * work_state_{nullptr};
  const hardware_interface::LoanedStateInterface * tool_state_{nullptr};
  const hardware_interface::LoanedStateInterface * rel_state_{nullptr};
  const hardware_interface::LoanedStateInterface * fixedspeed_state_{nullptr};
  const hardware_interface::LoanedStateInterface * cfg_state_{nullptr};
  const hardware_interface::LoanedStateInterface * coord_state_{nullptr};
  const hardware_interface::LoanedStateInterface * weaving_state_{nullptr};
  const hardware_interface::LoanedStateInterface * fixedorient_state_{nullptr};
  const hardware_interface::LoanedStateInterface * ext_state_{nullptr};
  const hardware_interface::LoanedStateInterface * tol_state_{nullptr};
  const hardware_interface::LoanedStateInterface * servo_relative_state_{nullptr};

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr options_pub_;
  rclcpp::Service<symphony_msgs::srv::SetMotionOptions>::SharedPtr set_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;

  std::atomic<bool> active_{false};
  std::atomic<bool> pending_reset_{false};
  std::atomic<bool> pending_set_{false};
  std::atomic<int32_t> pending_work_{-1};
  std::atomic<int32_t> pending_tool_{-1};
  std::atomic<bool> pending_rel_{false};
  std::atomic<bool> pending_fixedspeed_{false};
  std::atomic<int32_t> pending_cfg_{-1};
  std::atomic<int32_t> pending_coord_{-1};
  std::atomic<bool> pending_weaving_{false};
  std::atomic<bool> pending_fixedorient_{false};
  std::atomic<int32_t> pending_ext_{0};
  std::atomic<int32_t> pending_tol_{-1};
  std::atomic<bool> pending_servo_relative_{false};
};

}  // namespace symphony_controllers

#endif  // SYMPHONY_CONTROLLERS__MOTION_OPTIONS_CONTROLLER_HPP_
