#ifndef SYMPHONY_CONTROLLERS__TOOL_MODBUS_CONTROLLER_HPP_
#define SYMPHONY_CONTROLLERS__TOOL_MODBUS_CONTROLLER_HPP_

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "symphony_controllers/visibility_control.h"
#include "symphony_msgs/msg/tool_modbus_status.hpp"
#include "symphony_msgs/srv/open_tool_modbus.hpp"
#include "symphony_msgs/srv/set_tool_analog_mode.hpp"
#include "symphony_msgs/srv/tool_modbus_read.hpp"
#include "symphony_msgs/srv/tool_modbus_write.hpp"

namespace symphony_controllers
{
class ToolModbusController : public controller_interface::ControllerInterface
{
public:
  SYMPHONY_CONTROLLERS_PUBLIC
  ToolModbusController() = default;

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
  bool wait_done(uint64_t seq, std::string & message);
  bool run_op(
    int op, uint16_t device, uint16_t address, int size,
    const std::vector<uint16_t> & values, std::vector<uint16_t> * out,
    std::string & message);

  std::array<const hardware_interface::LoanedStateInterface *, 14> states_{};
  std::array<hardware_interface::LoanedCommandInterface *, 13> cmds_{};

  rclcpp::CallbackGroup::SharedPtr service_cb_group_;
  rclcpp::Publisher<symphony_msgs::msg::ToolModbusStatus>::SharedPtr status_pub_;
  rclcpp::Service<symphony_msgs::srv::SetToolAnalogMode>::SharedPtr set_analog_mode_srv_;
  rclcpp::Service<symphony_msgs::srv::OpenToolModbus>::SharedPtr open_modbus_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr close_modbus_srv_;
  rclcpp::Service<symphony_msgs::srv::ToolModbusRead>::SharedPtr read_srv_;
  rclcpp::Service<symphony_msgs::srv::ToolModbusWrite>::SharedPtr write_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_command_srv_;

  std::mutex request_mutex_;
  std::atomic<bool> active_{false};
  std::atomic<uint64_t> pending_seq_{0};
  std::atomic<double> pending_op_{0.0};
  std::atomic<double> pending_device_{0.0};
  std::atomic<double> pending_address_{0.0};
  std::atomic<double> pending_size_{1.0};
  std::array<std::atomic<double>, 8> pending_value_{};
  std::atomic<uint64_t> last_done_seq_{0};
  std::atomic<double> last_ok_{0.0};
  std::array<std::atomic<uint16_t>, 8> last_result_{};
};

}  // namespace symphony_controllers

#endif  // SYMPHONY_CONTROLLERS__TOOL_MODBUS_CONTROLLER_HPP_
