#ifndef SYMPHONY_CONTROLLERS__FORWARD_POSITION_CONTROLLER_HPP_
#define SYMPHONY_CONTROLLERS__FORWARD_POSITION_CONTROLLER_HPP_

#include <array>
#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/subscription.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "symphony_controllers/visibility_control.h"

namespace symphony_controllers
{

class ForwardPositionController : public controller_interface::ControllerInterface
{
public:
  SYMPHONY_CONTROLLERS_PUBLIC
  ForwardPositionController() = default;

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
  void write_command(const std::array<double, 6> & q, uint64_t seq);

  std::vector<std::string> joint_names_;
  std::string interface_name_{"position"};
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_sub_;

  std::array<std::atomic<double>, 6> target_{};
  std::atomic<bool> active_{false};
  std::atomic<uint64_t> accept_epoch_{0};
  std::atomic<uint64_t> write_seq_{0};
  std::atomic<uint64_t> cmd_seq_{0};
  std::atomic<uint64_t> applied_seq_{0};
};

}  // namespace symphony_controllers

#endif  // SYMPHONY_CONTROLLERS__FORWARD_POSITION_CONTROLLER_HPP_
