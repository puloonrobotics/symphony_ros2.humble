#include "symphony_controllers/status_controller.hpp"

#include <algorithm>
#include <cstdint>
#include <string>

#include "symphony_controllers/loaned_iface.hpp"

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace symphony_controllers
{

//----- interface names -----
namespace
{
constexpr const char * kSpeedRate = "speed_rate/value";
constexpr const char * kOpMode = "robot_status/op_mode";
constexpr const char * kSafetyStatus = "robot_status/safety_status";
constexpr const char * kSafetyZone = "robot_status/safety_zone";
constexpr const char * kArmState = "robot_status/arm_state";
constexpr const char * kServoEnable = "robot_status/servo_enable";

constexpr const char * kSpeedCmd = "speed_rate/command";
constexpr const char * kServoEnableCmd = "robot_status/servo_enable_cmd";
constexpr const char * kClearFaultCmd = "robot_status/clear_fault_cmd";
constexpr const char * kClearFaultWarning = "robot_status/clear_fault_warning";
constexpr const char * kCollisionEnableCmd = "robot_status/collision_enable_cmd";
constexpr const char * kRtPeriodCmd = "robot_status/rt_period_cmd";
constexpr const char * kRtFilterCmd = "robot_status/rt_filter_cmd";
constexpr const char * kPauseRtCmd = "robot_status/pause_rt_cmd";
constexpr const char * kResumeRtCmd = "robot_status/resume_rt_cmd";

constexpr double kNoCommand = -1.0;

std::string arm_error_name(size_t index)
{
  return "robot_status/arm_error_" + std::to_string(index);
}
}  // namespace

//----- lifecycle -----
controller_interface::InterfaceConfiguration
StatusController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = {kSpeedCmd, kServoEnableCmd, kClearFaultCmd, kClearFaultWarning,
    kCollisionEnableCmd, kRtPeriodCmd, kRtFilterCmd, kPauseRtCmd, kResumeRtCmd};
  return config;
}

controller_interface::InterfaceConfiguration
StatusController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = {
    kSpeedRate, kOpMode, kSafetyStatus, kSafetyZone, kArmState, kServoEnable};
  for (size_t i = 0; i < kArmErrorSlots; ++i) {
    config.names.push_back(arm_error_name(i));
  }
  return config;
}

controller_interface::CallbackReturn StatusController::on_init()
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn StatusController::on_configure(
  const rclcpp_lifecycle::State &)
{
  speed_rate_pub_ = get_node()->create_publisher<std_msgs::msg::Float64>(
    "~/speed_rate", rclcpp::SystemDefaultsQoS());
  op_mode_pub_ = get_node()->create_publisher<std_msgs::msg::UInt8>(
    "~/op_mode", rclcpp::SystemDefaultsQoS());
  safety_status_pub_ = get_node()->create_publisher<symphony_msgs::msg::SafetyStatus>(
    "~/safety_status", rclcpp::SystemDefaultsQoS());
  arm_state_pub_ = get_node()->create_publisher<std_msgs::msg::UInt32>(
    "~/arm_state", rclcpp::SystemDefaultsQoS());
  servo_enable_pub_ = get_node()->create_publisher<std_msgs::msg::Bool>(
    "~/servo_enable", rclcpp::SystemDefaultsQoS());
  robot_error_pub_ = get_node()->create_publisher<symphony_msgs::msg::RobotError>(
    "~/robot_error", rclcpp::QoS(1).transient_local());

  set_speed_srv_ = get_node()->create_service<symphony_msgs::srv::SetSpeed>(
    "~/set_speed",
    [this](
      const std::shared_ptr<symphony_msgs::srv::SetSpeed::Request> request,
      std::shared_ptr<symphony_msgs::srv::SetSpeed::Response> response)
    {
      if (!active_.load()) {
        response->success = false;
        response->message = "status_controller is not active";
        return;
      }
      if (request->speed < 0.0 || request->speed > 1.0) {
        response->success = false;
        response->message = "speed must be in [0, 1]";
        return;
      }
      pending_speed_.store(request->speed);
      response->success = true;
      response->message = "accepted";
    });

  set_servo_enable_srv_ = get_node()->create_service<std_srvs::srv::SetBool>(
    "~/set_servo_enable",
    [this](
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
      if (!active_.load()) {
        response->success = false;
        response->message = "status_controller is not active";
        return;
      }
      pending_servo_enable_.store(request->data ? 1.0 : 0.0);
      response->success = true;
      response->message = "accepted";
    });

  clear_fault_srv_ = get_node()->create_service<symphony_msgs::srv::ClearFault>(
    "~/clear_fault",
    [this](
      const std::shared_ptr<symphony_msgs::srv::ClearFault::Request> request,
      std::shared_ptr<symphony_msgs::srv::ClearFault::Response> response)
    {
      if (!active_.load()) {
        response->success = false;
        response->message = "status_controller is not active";
        return;
      }
      pending_clear_fault_warning_.store(request->warning);
      pending_clear_fault_seq_.fetch_add(1);
      response->success = true;
      response->message = "accepted";
    });

  set_collision_srv_ = get_node()->create_service<std_srvs::srv::SetBool>(
    "~/set_collision_detection",
    [this](
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
      if (!active_.load()) {
        response->success = false;
        response->message = "status_controller is not active";
        return;
      }
      pending_collision_.store(request->data ? 1.0 : 0.0);
      response->success = true;
      response->message = "accepted";
    });

  set_rt_stream_srv_ = get_node()->create_service<symphony_msgs::srv::SetRtStream>(
    "~/set_rt_stream",
    [this](
      const std::shared_ptr<symphony_msgs::srv::SetRtStream::Request> request,
      std::shared_ptr<symphony_msgs::srv::SetRtStream::Response> response)
    {
      if (!active_.load()) {
        response->success = false;
        response->message = "status_controller is not active";
        return;
      }
      if (request->period <= 0.0) {
        response->success = false;
        response->message = "period must be > 0 (default 0.002)";
        return;
      }
      pending_rt_period_.store(request->period);
      pending_rt_filter_.store(static_cast<double>(request->filter));
      response->success = true;
      response->message = "accepted (applies on next startRt*)";
    });

  pause_rt_srv_ = get_node()->create_service<std_srvs::srv::Trigger>(
    "~/pause_rt",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      if (!active_.load()) {
        response->success = false;
        response->message = "status_controller is not active";
        return;
      }
      pending_pause_rt_seq_.fetch_add(1);
      response->success = true;
      response->message = "accepted (requires puloon_rtpri >= 1.3.0)";
    });

  resume_rt_srv_ = get_node()->create_service<std_srvs::srv::Trigger>(
    "~/resume_rt",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      if (!active_.load()) {
        response->success = false;
        response->message = "status_controller is not active";
        return;
      }
      pending_resume_rt_seq_.fetch_add(1);
      response->success = true;
      response->message = "accepted (requires puloon_rtpri >= 1.3.0)";
    });

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn StatusController::on_activate(
  const rclcpp_lifecycle::State &)
{
  constexpr size_t kExpectedStates = 6 + kArmErrorSlots;
  if (state_interfaces_.size() != kExpectedStates || command_interfaces_.size() != 9) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected %zu state and 9 command interfaces, got %zu state / %zu command",
      kExpectedStates, state_interfaces_.size(), command_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  speed_state_ = find_state(kSpeedRate);
  op_mode_state_ = find_state(kOpMode);
  safety_status_state_ = find_state(kSafetyStatus);
  safety_zone_state_ = find_state(kSafetyZone);
  arm_state_state_ = find_state(kArmState);
  servo_enable_state_ = find_state(kServoEnable);
  bool arm_errors_bound = true;
  for (size_t i = 0; i < kArmErrorSlots; ++i) {
    arm_error_states_[i] = find_state(arm_error_name(i));
    arm_errors_bound = arm_errors_bound && arm_error_states_[i] != nullptr;
  }
  speed_cmd_ = find_command(kSpeedCmd);
  servo_enable_cmd_ = find_command(kServoEnableCmd);
  clear_fault_cmd_ = find_command(kClearFaultCmd);
  clear_fault_warning_cmd_ = find_command(kClearFaultWarning);
  collision_enable_cmd_ = find_command(kCollisionEnableCmd);
  rt_period_cmd_ = find_command(kRtPeriodCmd);
  rt_filter_cmd_ = find_command(kRtFilterCmd);
  pause_rt_cmd_ = find_command(kPauseRtCmd);
  resume_rt_cmd_ = find_command(kResumeRtCmd);

  if (!speed_state_ || !op_mode_state_ || !safety_status_state_ || !safety_zone_state_ ||
      !arm_state_state_ || !servo_enable_state_ || !arm_errors_bound || !speed_cmd_ ||
      !servo_enable_cmd_ || !clear_fault_cmd_ || !clear_fault_warning_cmd_ ||
      !collision_enable_cmd_ || !rt_period_cmd_ || !rt_filter_cmd_ ||
      !pause_rt_cmd_ || !resume_rt_cmd_)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Failed to bind status gpio interfaces");
    clear_interface_cache();
    return controller_interface::CallbackReturn::ERROR;
  }

  published_errors_ = {0};

  set_command(*speed_cmd_, kNoCommand);
  set_command(*servo_enable_cmd_, kNoCommand);
  set_command(*clear_fault_warning_cmd_, 0.0);
  set_command(*clear_fault_cmd_, static_cast<double>(pending_clear_fault_seq_.load()));
  set_command(*collision_enable_cmd_, kNoCommand);
  set_command(*rt_period_cmd_, kNoCommand);
  set_command(*rt_filter_cmd_, kNoCommand);
  set_command(*pause_rt_cmd_, static_cast<double>(pending_pause_rt_seq_.load()));
  set_command(*resume_rt_cmd_, static_cast<double>(pending_resume_rt_seq_.load()));

  active_.store(true);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn StatusController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  active_.store(false);
  clear_interface_cache();
  return controller_interface::CallbackReturn::SUCCESS;
}

//----- helpers -----
void StatusController::clear_interface_cache()
{
  speed_state_ = nullptr;
  op_mode_state_ = nullptr;
  safety_status_state_ = nullptr;
  safety_zone_state_ = nullptr;
  arm_state_state_ = nullptr;
  servo_enable_state_ = nullptr;
  arm_error_states_.fill(nullptr);
  speed_cmd_ = nullptr;
  servo_enable_cmd_ = nullptr;
  clear_fault_cmd_ = nullptr;
  clear_fault_warning_cmd_ = nullptr;
  collision_enable_cmd_ = nullptr;
  rt_period_cmd_ = nullptr;
  rt_filter_cmd_ = nullptr;
  pause_rt_cmd_ = nullptr;
  resume_rt_cmd_ = nullptr;
}

const hardware_interface::LoanedStateInterface *
StatusController::find_state(const std::string & name) const
{
  for (const auto & iface : state_interfaces_) {
    if (iface.get_name() == name) {
      return &iface;
    }
  }
  return nullptr;
}

hardware_interface::LoanedCommandInterface *
StatusController::find_command(const std::string & name)
{
  for (auto & iface : command_interfaces_) {
    if (iface.get_name() == name) {
      return &iface;
    }
  }
  return nullptr;
}

void StatusController::publish_robot_error(const rclcpp::Time & time)
{
  symphony_msgs::msg::RobotError msg;
  msg.codes.reserve(kArmErrorSlots);
  for (const auto * state : arm_error_states_) {
    const auto code = static_cast<uint32_t>(get_double(*state));
    if (code == 0) {
      break;
    }
    msg.codes.push_back(code);
  }

  if (msg.codes == published_errors_) {
    return;
  }

  msg.header.stamp = time;
  msg.latest = msg.codes.empty() ? 0u : msg.codes.back();
  published_errors_ = msg.codes;
  robot_error_pub_->publish(msg);
}

//----- update -----
controller_interface::return_type StatusController::update(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  if (!speed_state_ || !op_mode_state_ || !safety_status_state_ || !safety_zone_state_ ||
      !arm_state_state_ || !servo_enable_state_ || !speed_cmd_ || !servo_enable_cmd_ ||
      !clear_fault_cmd_ || !clear_fault_warning_cmd_ || !collision_enable_cmd_ ||
      !rt_period_cmd_ || !rt_filter_cmd_ || !pause_rt_cmd_ || !resume_rt_cmd_ ||
      std::find(arm_error_states_.begin(), arm_error_states_.end(), nullptr) !=
      arm_error_states_.end())
  {
    return controller_interface::return_type::ERROR;
  }

  set_command(*speed_cmd_, pending_speed_.load());
  set_command(*servo_enable_cmd_, pending_servo_enable_.load());
  set_command(*clear_fault_warning_cmd_, pending_clear_fault_warning_.load() ? 1.0 : 0.0);
  set_command(*clear_fault_cmd_, static_cast<double>(pending_clear_fault_seq_.load()));
  set_command(*collision_enable_cmd_, pending_collision_.load());
  set_command(*rt_period_cmd_, pending_rt_period_.load());
  set_command(*rt_filter_cmd_, pending_rt_filter_.load());
  set_command(*pause_rt_cmd_, static_cast<double>(pending_pause_rt_seq_.load()));
  set_command(*resume_rt_cmd_, static_cast<double>(pending_resume_rt_seq_.load()));

  std_msgs::msg::Float64 speed_msg;
  speed_msg.data = get_double(*speed_state_);
  speed_rate_pub_->publish(speed_msg);

  std_msgs::msg::UInt8 op_mode_msg;
  op_mode_msg.data = static_cast<uint8_t>(get_double(*op_mode_state_));
  op_mode_pub_->publish(op_mode_msg);

  const auto arm_state = static_cast<uint32_t>(get_double(*arm_state_state_));

  symphony_msgs::msg::SafetyStatus safety_msg;
  safety_msg.header.stamp = time;
  safety_msg.mode = static_cast<uint8_t>(get_double(*safety_status_state_));
  safety_msg.zone = static_cast<uint8_t>(get_double(*safety_zone_state_));
  safety_msg.arm_state = arm_state;
  safety_status_pub_->publish(safety_msg);

  std_msgs::msg::UInt32 arm_state_msg;
  arm_state_msg.data = arm_state;
  arm_state_pub_->publish(arm_state_msg);

  std_msgs::msg::Bool servo_msg;
  servo_msg.data = get_double(*servo_enable_state_) > 0.5;
  servo_enable_pub_->publish(servo_msg);

  publish_robot_error(time);

  return controller_interface::return_type::OK;
}

}  // namespace symphony_controllers

PLUGINLIB_EXPORT_CLASS(
  symphony_controllers::StatusController, controller_interface::ControllerInterface)
