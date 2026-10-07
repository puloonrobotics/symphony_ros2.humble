#ifndef SYMPHONY_CONTROLLERS__STATUS_CONTROLLER_HPP_
#define SYMPHONY_CONTROLLERS__STATUS_CONTROLLER_HPP_

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "symphony_controllers/visibility_control.h"
#include "symphony_msgs/msg/robot_error.hpp"
#include "symphony_msgs/msg/safety_status.hpp"
#include "symphony_msgs/srv/set_speed.hpp"
#include "symphony_msgs/srv/clear_fault.hpp"
#include "symphony_msgs/srv/set_rt_stream.hpp"

namespace symphony_controllers
{
constexpr size_t kArmErrorSlots = 10;

class StatusController : public controller_interface::ControllerInterface
{
public:
  SYMPHONY_CONTROLLERS_PUBLIC
  StatusController() = default;

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
  void publish_robot_error(const rclcpp::Time & time);

  const hardware_interface::LoanedStateInterface * speed_state_{nullptr};
  const hardware_interface::LoanedStateInterface * op_mode_state_{nullptr};
  const hardware_interface::LoanedStateInterface * safety_status_state_{nullptr};
  const hardware_interface::LoanedStateInterface * safety_zone_state_{nullptr};
  const hardware_interface::LoanedStateInterface * arm_state_state_{nullptr};
  const hardware_interface::LoanedStateInterface * servo_enable_state_{nullptr};
  std::array<const hardware_interface::LoanedStateInterface *, kArmErrorSlots>
  arm_error_states_{};

  hardware_interface::LoanedCommandInterface * speed_cmd_{nullptr};
  hardware_interface::LoanedCommandInterface * servo_enable_cmd_{nullptr};
  hardware_interface::LoanedCommandInterface * clear_fault_cmd_{nullptr};
  hardware_interface::LoanedCommandInterface * clear_fault_warning_cmd_{nullptr};
  hardware_interface::LoanedCommandInterface * collision_enable_cmd_{nullptr};
  hardware_interface::LoanedCommandInterface * rt_period_cmd_{nullptr};
  hardware_interface::LoanedCommandInterface * rt_filter_cmd_{nullptr};
  hardware_interface::LoanedCommandInterface * pause_rt_cmd_{nullptr};
  hardware_interface::LoanedCommandInterface * resume_rt_cmd_{nullptr};

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_rate_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr op_mode_pub_;
  rclcpp::Publisher<symphony_msgs::msg::SafetyStatus>::SharedPtr safety_status_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr arm_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr servo_enable_pub_;
  rclcpp::Publisher<symphony_msgs::msg::RobotError>::SharedPtr robot_error_pub_;
  std::vector<uint32_t> published_errors_;

  rclcpp::Service<symphony_msgs::srv::SetSpeed>::SharedPtr set_speed_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_servo_enable_srv_;
  rclcpp::Service<symphony_msgs::srv::ClearFault>::SharedPtr clear_fault_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_collision_srv_;
  rclcpp::Service<symphony_msgs::srv::SetRtStream>::SharedPtr set_rt_stream_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_rt_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_rt_srv_;

  std::atomic<bool> active_{false};
  std::atomic<double> pending_speed_{-1.0};
  std::atomic<double> pending_servo_enable_{-1.0};
  std::atomic<uint64_t> pending_clear_fault_seq_{0};
  std::atomic<bool> pending_clear_fault_warning_{false};
  std::atomic<double> pending_collision_{-1.0};
  std::atomic<double> pending_rt_period_{-1.0};
  std::atomic<double> pending_rt_filter_{-1.0};
  std::atomic<uint64_t> pending_pause_rt_seq_{0};
  std::atomic<uint64_t> pending_resume_rt_seq_{0};
};

}  // namespace symphony_controllers

#endif  // SYMPHONY_CONTROLLERS__STATUS_CONTROLLER_HPP_
