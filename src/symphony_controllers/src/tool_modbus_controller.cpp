#include "symphony_controllers/tool_modbus_controller.hpp"

#include <chrono>
#include <iterator>
#include <thread>

#include "symphony_controllers/loaned_iface.hpp"

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "symphony_msgs/hw_constants.hpp"

namespace symphony_controllers
{

//----- interface names -----
namespace
{
constexpr const char * kStateNames[] = {
  "tool_modbus/done_seq",
  "tool_modbus/ok",
  "tool_modbus/command",
  "tool_modbus/address",
  "tool_modbus/number",
  "tool_modbus/result_0",
  "tool_modbus/result_1",
  "tool_modbus/result_2",
  "tool_modbus/result_3",
  "tool_modbus/result_4",
  "tool_modbus/result_5",
  "tool_modbus/result_6",
  "tool_modbus/result_7",
  "robot_status/tool_io_status",
};
constexpr size_t kNumStates = sizeof(kStateNames) / sizeof(kStateNames[0]);

constexpr const char * kCmdNames[] = {
  "tool_modbus/seq",
  "tool_modbus/op",
  "tool_modbus/device",
  "tool_modbus/address",
  "tool_modbus/size",
  "tool_modbus/value_0",
  "tool_modbus/value_1",
  "tool_modbus/value_2",
  "tool_modbus/value_3",
  "tool_modbus/value_4",
  "tool_modbus/value_5",
  "tool_modbus/value_6",
  "tool_modbus/value_7",
};
constexpr size_t kNumCmds = sizeof(kCmdNames) / sizeof(kCmdNames[0]);

constexpr size_t kDoneSeq = 0;
constexpr size_t kOk = 1;
constexpr size_t kCommand = 2;
constexpr size_t kAddress = 3;
constexpr size_t kNumber = 4;
constexpr size_t kResult0 = 5;
constexpr size_t kIoStatus = 13;

constexpr size_t kSeqCmd = 0;
constexpr size_t kOpCmd = 1;
constexpr size_t kDeviceCmd = 2;
constexpr size_t kAddressCmd = 3;
constexpr size_t kSizeCmd = 4;
constexpr size_t kValue0Cmd = 5;

using symphony_msgs::kTmAnalogOff;
using symphony_msgs::kTmAnalogOn;
using symphony_msgs::kTmClear;
using symphony_msgs::kTmClose;
using symphony_msgs::kTmOpen;
using symphony_msgs::kTmReadBit;
using symphony_msgs::kTmReadInputBit;
using symphony_msgs::kTmReadReg;
using symphony_msgs::kTmWriteBit;
using symphony_msgs::kTmWriteReg;

constexpr uint16_t kToolStatusModbus = 0x0008;
constexpr auto kWaitTimeout = std::chrono::seconds(3);

bool size_ok(int size)
{
  return size >= 1 && size <= 8;
}
}  // namespace

//----- lifecycle -----
controller_interface::InterfaceConfiguration
ToolModbusController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.assign(std::begin(kCmdNames), std::end(kCmdNames));
  return config;
}

controller_interface::InterfaceConfiguration
ToolModbusController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.assign(std::begin(kStateNames), std::end(kStateNames));
  return config;
}

controller_interface::CallbackReturn ToolModbusController::on_init()
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ToolModbusController::on_configure(
  const rclcpp_lifecycle::State &)
{
  service_cb_group_ = get_node()->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  status_pub_ = get_node()->create_publisher<symphony_msgs::msg::ToolModbusStatus>(
    "~/modbus_status", rclcpp::SystemDefaultsQoS());

  set_analog_mode_srv_ = get_node()->create_service<symphony_msgs::srv::SetToolAnalogMode>(
    "~/set_analog_mode",
    [this](
      const std::shared_ptr<symphony_msgs::srv::SetToolAnalogMode::Request> request,
      std::shared_ptr<symphony_msgs::srv::SetToolAnalogMode::Response> response)
    {
      std::vector<uint16_t> unused;
      response->success = run_op(
        request->enable_analog ? kTmAnalogOn : kTmAnalogOff,
        0, 0, 1, {}, &unused, response->message);
    },
    rmw_qos_profile_services_default,
    service_cb_group_);

  open_modbus_srv_ = get_node()->create_service<symphony_msgs::srv::OpenToolModbus>(
    "~/open_modbus",
    [this](
      const std::shared_ptr<symphony_msgs::srv::OpenToolModbus::Request> request,
      std::shared_ptr<symphony_msgs::srv::OpenToolModbus::Response> response)
    {
      std::vector<uint16_t> unused;
      response->success = run_op(
        kTmOpen, request->device, 0, 1, {}, &unused, response->message);
    },
    rmw_qos_profile_services_default,
    service_cb_group_);

  close_modbus_srv_ = get_node()->create_service<std_srvs::srv::Trigger>(
    "~/close_modbus",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      std::vector<uint16_t> unused;
      response->success = run_op(kTmClose, 0, 0, 1, {}, &unused, response->message);
    },
    rmw_qos_profile_services_default,
    service_cb_group_);

  read_srv_ = get_node()->create_service<symphony_msgs::srv::ToolModbusRead>(
    "~/read",
    [this](
      const std::shared_ptr<symphony_msgs::srv::ToolModbusRead::Request> request,
      std::shared_ptr<symphony_msgs::srv::ToolModbusRead::Response> response)
    {
      int op = 0;
      switch (request->type) {
        case symphony_msgs::srv::ToolModbusRead::Request::TYPE_BIT:
          op = kTmReadBit;
          break;
        case symphony_msgs::srv::ToolModbusRead::Request::TYPE_INPUT_BIT:
          op = kTmReadInputBit;
          break;
        case symphony_msgs::srv::ToolModbusRead::Request::TYPE_REGISTER:
          op = kTmReadReg;
          break;
        default:
          response->success = false;
          response->message = "type must be 0=coil 1=discrete input 2=holding register";
          return;
      }
      if (!size_ok(static_cast<int>(request->size))) {
        response->success = false;
        response->message = "size must be 1-8";
        return;
      }
      response->success = run_op(
        op, 0, request->address, static_cast<int>(request->size), {},
        &response->data, response->message);
    },
    rmw_qos_profile_services_default,
    service_cb_group_);

  write_srv_ = get_node()->create_service<symphony_msgs::srv::ToolModbusWrite>(
    "~/write",
    [this](
      const std::shared_ptr<symphony_msgs::srv::ToolModbusWrite::Request> request,
      std::shared_ptr<symphony_msgs::srv::ToolModbusWrite::Response> response)
    {
      int op = 0;
      switch (request->type) {
        case symphony_msgs::srv::ToolModbusWrite::Request::TYPE_BIT:
          op = kTmWriteBit;
          break;
        case symphony_msgs::srv::ToolModbusWrite::Request::TYPE_REGISTER:
          op = kTmWriteReg;
          break;
        default:
          response->success = false;
          response->message = "type must be 0=coil 1=holding register";
          return;
      }
      const int size = static_cast<int>(request->values.size());
      if (!size_ok(size)) {
        response->success = false;
        response->message = "values length must be 1-8";
        return;
      }
      std::vector<uint16_t> unused;
      response->success = run_op(
        op, 0, request->address, size, request->values, &unused, response->message);
    },
    rmw_qos_profile_services_default,
    service_cb_group_);

  clear_command_srv_ = get_node()->create_service<std_srvs::srv::Trigger>(
    "~/clear_command",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      std::vector<uint16_t> unused;
      response->success = run_op(kTmClear, 0, 0, 1, {}, &unused, response->message);
    },
    rmw_qos_profile_services_default,
    service_cb_group_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ToolModbusController::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (state_interfaces_.size() != kNumStates || command_interfaces_.size() != kNumCmds) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected %zu state and %zu command interfaces, got %zu state / %zu command",
      kNumStates, kNumCmds, state_interfaces_.size(), command_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  for (size_t i = 0; i < kNumStates; ++i) {
    states_[i] = find_state(kStateNames[i]);
    if (!states_[i]) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Missing tool Modbus state %s", kStateNames[i]);
      clear_interface_cache();
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  for (size_t i = 0; i < kNumCmds; ++i) {
    cmds_[i] = find_command(kCmdNames[i]);
    if (!cmds_[i]) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Missing tool Modbus command %s", kCmdNames[i]);
      clear_interface_cache();
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  last_done_seq_.store(
    static_cast<uint64_t>(get_double(*states_[kDoneSeq])), std::memory_order_relaxed);
  pending_seq_.store(last_done_seq_.load(std::memory_order_relaxed), std::memory_order_relaxed);
  set_command(*cmds_[kSeqCmd], static_cast<double>(pending_seq_.load(std::memory_order_relaxed)));

  active_.store(true);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ToolModbusController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  active_.store(false);
  clear_interface_cache();
  return controller_interface::CallbackReturn::SUCCESS;
}

//----- helpers -----
void ToolModbusController::clear_interface_cache()
{
  states_.fill(nullptr);
  cmds_.fill(nullptr);
}

const hardware_interface::LoanedStateInterface *
ToolModbusController::find_state(const std::string & name) const
{
  for (const auto & iface : state_interfaces_) {
    if (iface.get_name() == name) {
      return &iface;
    }
  }
  return nullptr;
}

hardware_interface::LoanedCommandInterface *
ToolModbusController::find_command(const std::string & name)
{
  for (auto & iface : command_interfaces_) {
    if (iface.get_name() == name) {
      return &iface;
    }
  }
  return nullptr;
}

bool ToolModbusController::wait_done(uint64_t seq, std::string & message)
{
  // Service thread polls until update() advances last_done_seq_.
  const auto deadline = std::chrono::steady_clock::now() + kWaitTimeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (last_done_seq_.load(std::memory_order_acquire) >= seq) {
      if (last_ok_.load(std::memory_order_relaxed) > 0.5) {
        message = "ok";
        return true;
      }
      message = "RTPRI tool Modbus command failed";
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  message = "timeout waiting for tool Modbus RTPRI";
  return false;
}

bool ToolModbusController::run_op(
  int op, uint16_t device, uint16_t address, int size,
  const std::vector<uint16_t> & values, std::vector<uint16_t> * out,
  std::string & message)
{
  if (!active_.load()) {
    message = "tool_modbus_controller is not active";
    return false;
  }
  if (!size_ok(size)) {
    message = "size must be 1-8";
    return false;
  }

  std::lock_guard<std::mutex> lock(request_mutex_);
  pending_op_.store(static_cast<double>(op), std::memory_order_relaxed);
  pending_device_.store(static_cast<double>(device), std::memory_order_relaxed);
  pending_address_.store(static_cast<double>(address), std::memory_order_relaxed);
  pending_size_.store(static_cast<double>(size), std::memory_order_relaxed);
  for (size_t i = 0; i < 8; ++i) {
    pending_value_[i].store(
      (i < values.size()) ? static_cast<double>(values[i]) : 0.0,
      std::memory_order_relaxed);
  }
  const uint64_t seq = pending_seq_.fetch_add(1, std::memory_order_release) + 1;
  if (!wait_done(seq, message)) {
    return false;
  }
  if (out) {
    out->assign(static_cast<size_t>(size), 0);
    for (int i = 0; i < size; ++i) {
      (*out)[static_cast<size_t>(i)] =
        last_result_[static_cast<size_t>(i)].load(std::memory_order_relaxed);
    }
  }
  return true;
}

//----- update -----
controller_interface::return_type ToolModbusController::update(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!states_[kDoneSeq] || !cmds_[kSeqCmd]) {
    return controller_interface::return_type::ERROR;
  }

  set_command(*cmds_[kOpCmd], pending_op_.load(std::memory_order_relaxed));
  set_command(*cmds_[kDeviceCmd], pending_device_.load(std::memory_order_relaxed));
  set_command(*cmds_[kAddressCmd], pending_address_.load(std::memory_order_relaxed));
  set_command(*cmds_[kSizeCmd], pending_size_.load(std::memory_order_relaxed));
  for (size_t i = 0; i < 8; ++i) {
    set_command(*cmds_[kValue0Cmd + i], pending_value_[i].load(std::memory_order_relaxed));
  }
  set_command(
    *cmds_[kSeqCmd],
    static_cast<double>(pending_seq_.load(std::memory_order_acquire)));

  last_ok_.store(get_double(*states_[kOk]), std::memory_order_relaxed);
  for (size_t i = 0; i < 8; ++i) {
    last_result_[i].store(
      static_cast<uint16_t>(get_double(*states_[kResult0 + i])),
      std::memory_order_relaxed);
  }
  last_done_seq_.store(
    static_cast<uint64_t>(get_double(*states_[kDoneSeq])), std::memory_order_release);

  symphony_msgs::msg::ToolModbusStatus msg;
  msg.header.stamp = get_node()->now();
  msg.command = static_cast<uint16_t>(get_double(*states_[kCommand]));
  msg.address = static_cast<uint16_t>(get_double(*states_[kAddress]));
  msg.number = static_cast<uint16_t>(get_double(*states_[kNumber]));
  for (size_t i = 0; i < 8; ++i) {
    msg.value[i] = last_result_[i].load(std::memory_order_relaxed);
  }
  msg.io_status = static_cast<uint16_t>(get_double(*states_[kIoStatus]));
  msg.modbus_ok = (msg.io_status & kToolStatusModbus) != 0;
  status_pub_->publish(msg);

  return controller_interface::return_type::OK;
}

}  // namespace symphony_controllers

PLUGINLIB_EXPORT_CLASS(
  symphony_controllers::ToolModbusController, controller_interface::ControllerInterface)
