#ifndef SYMPHONY_CONTROLLERS__IO_CONTROLLER_HPP_
#define SYMPHONY_CONTROLLERS__IO_CONTROLLER_HPP_

#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "symphony_controllers/visibility_control.h"
#include "symphony_msgs/msg/io_states.hpp"
#include "symphony_msgs/msg/tool_io_states.hpp"
#include "symphony_msgs/srv/set_analog_output.hpp"
#include "symphony_msgs/srv/set_digital_output.hpp"
#include "symphony_msgs/srv/set_tool_io.hpp"

namespace symphony_controllers
{
class IoController : public controller_interface::ControllerInterface
{
public:
  SYMPHONY_CONTROLLERS_PUBLIC
  IoController() = default;

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
  double state_value(size_t index) const;
  void queue_digital(double bank, double pin, double value);
  void queue_analog(double bank, double channel, double value);

  std::vector<const hardware_interface::LoanedStateInterface *> states_;
  hardware_interface::LoanedCommandInterface * digital_seq_cmd_{nullptr};
  hardware_interface::LoanedCommandInterface * digital_bank_cmd_{nullptr};
  hardware_interface::LoanedCommandInterface * digital_pin_cmd_{nullptr};
  hardware_interface::LoanedCommandInterface * digital_value_cmd_{nullptr};
  hardware_interface::LoanedCommandInterface * analog_seq_cmd_{nullptr};
  hardware_interface::LoanedCommandInterface * analog_bank_cmd_{nullptr};
  hardware_interface::LoanedCommandInterface * analog_channel_cmd_{nullptr};
  hardware_interface::LoanedCommandInterface * analog_value_cmd_{nullptr};

  rclcpp::Publisher<symphony_msgs::msg::IOStates>::SharedPtr get_io_pub_;
  rclcpp::Publisher<symphony_msgs::msg::ToolIOStates>::SharedPtr get_tool_io_pub_;
  rclcpp::Service<symphony_msgs::srv::SetDigitalOutput>::SharedPtr set_digital_output_srv_;
  rclcpp::Service<symphony_msgs::srv::SetAnalogOutput>::SharedPtr set_analog_output_srv_;
  rclcpp::Service<symphony_msgs::srv::SetToolIO>::SharedPtr set_tool_io_srv_;

  std::atomic<bool> active_{false};
  std::atomic<uint64_t> pending_digital_seq_{0};
  std::atomic<double> pending_digital_bank_{-1.0};
  std::atomic<double> pending_digital_pin_{-1.0};
  std::atomic<double> pending_digital_value_{-1.0};
  std::atomic<uint64_t> pending_analog_seq_{0};
  std::atomic<double> pending_analog_bank_{-1.0};
  std::atomic<double> pending_analog_channel_{-1.0};
  std::atomic<double> pending_analog_value_{0.0};
};

}  // namespace symphony_controllers

#endif  // SYMPHONY_CONTROLLERS__IO_CONTROLLER_HPP_
