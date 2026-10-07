#include "symphony_controllers/io_controller.hpp"

#include <cmath>
#include <cstddef>
#include <iterator>

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
  "io/safety_di",   "io/safety_do",
  "io/safety_ai0",  "io/safety_ai1",  "io/safety_ao0",  "io/safety_ao1",
  "io/tool_di",     "io/tool_do",     "io/tool_ai0",    "io/tool_ai1",
  "io/ex1_di",      "io/ex1_do",
  "io/ex1_ai0",     "io/ex1_ai1",     "io/ex1_ai2",     "io/ex1_ai3",
  "io/ex1_ao0",     "io/ex1_ao1",     "io/ex1_ao2",     "io/ex1_ao3",
  "io/ex2_di",      "io/ex2_do",
  "io/ex2_ai0",     "io/ex2_ai1",     "io/ex2_ai2",     "io/ex2_ai3",
  "io/ex2_ao0",     "io/ex2_ao1",     "io/ex2_ao2",     "io/ex2_ao3",
};
constexpr size_t kNumIoStates = sizeof(kStateNames) / sizeof(kStateNames[0]);

constexpr const char * kDigitalSeq = "io/digital_seq";
constexpr const char * kDigitalBank = "io/digital_bank";
constexpr const char * kDigitalPin = "io/digital_pin";
constexpr const char * kDigitalValue = "io/digital_value";
constexpr const char * kAnalogSeq = "io/analog_seq";
constexpr const char * kAnalogBank = "io/analog_bank";
constexpr const char * kAnalogChannel = "io/analog_channel";
constexpr const char * kAnalogValue = "io/analog_value";

enum IoStateIndex : size_t
{
  kSafetyDi = 0,
  kSafetyDo,
  kSafetyAi0,
  kSafetyAi1,
  kSafetyAo0,
  kSafetyAo1,
  kToolDi,
  kToolDo,
  kToolAi0,
  kToolAi1,
  kEx1Di,
  kEx1Do,
  kEx1Ai0,
  kEx1Ai1,
  kEx1Ai2,
  kEx1Ai3,
  kEx1Ao0,
  kEx1Ao1,
  kEx1Ao2,
  kEx1Ao3,
  kEx2Di,
  kEx2Do,
  kEx2Ai0,
  kEx2Ai1,
  kEx2Ai2,
  kEx2Ai3,
  kEx2Ao0,
  kEx2Ao1,
  kEx2Ao2,
  kEx2Ao3,
};

uint32_t as_u32(double value)
{
  if (value < 0.0) {
    return 0;
  }
  return static_cast<uint32_t>(value);
}

bool digital_pin_ok(uint8_t bank, uint8_t pin)
{
  return pin >= 1 && pin <= 32 &&
    bank <= symphony_msgs::srv::SetDigitalOutput::Request::BANK_SAFETY_EX2;
}

bool analog_args_ok(uint8_t bank, uint8_t channel)
{
  switch (bank) {
    case symphony_msgs::srv::SetAnalogOutput::Request::BANK_SAFETY:
      return channel >= 1 && channel <= 2;
    case symphony_msgs::srv::SetAnalogOutput::Request::BANK_SAFETY_EX1:
    case symphony_msgs::srv::SetAnalogOutput::Request::BANK_SAFETY_EX2:
      return channel >= 1 && channel <= 4;
    default:
      return false;
  }
}

constexpr double kHwBankTool = static_cast<double>(symphony_msgs::kIoBankTool);
}  // namespace

//----- lifecycle -----
controller_interface::InterfaceConfiguration
IoController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = {
    kDigitalSeq, kDigitalBank, kDigitalPin, kDigitalValue,
    kAnalogSeq, kAnalogBank, kAnalogChannel, kAnalogValue};
  return config;
}

controller_interface::InterfaceConfiguration
IoController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.assign(std::begin(kStateNames), std::end(kStateNames));
  return config;
}

controller_interface::CallbackReturn IoController::on_init()
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn IoController::on_configure(
  const rclcpp_lifecycle::State &)
{
  get_io_pub_ = get_node()->create_publisher<symphony_msgs::msg::IOStates>(
    "~/get_io", rclcpp::SystemDefaultsQoS());
  get_tool_io_pub_ = get_node()->create_publisher<symphony_msgs::msg::ToolIOStates>(
    "~/get_tool_io", rclcpp::SystemDefaultsQoS());

  set_digital_output_srv_ = get_node()->create_service<symphony_msgs::srv::SetDigitalOutput>(
    "~/set_digital_output",
    [this](
      const std::shared_ptr<symphony_msgs::srv::SetDigitalOutput::Request> request,
      std::shared_ptr<symphony_msgs::srv::SetDigitalOutput::Response> response)
    {
      if (!active_.load()) {
        response->success = false;
        response->message = "io_controller is not active";
        return;
      }
      if (!digital_pin_ok(request->bank, request->pin)) {
        response->success = false;
        response->message = "bank must be 0-2 (safety/ex1/ex2) and pin 1-32";
        return;
      }
      queue_digital(
        static_cast<double>(request->bank), static_cast<double>(request->pin),
        request->state ? 1.0 : 0.0);
      response->success = true;
      response->message = "accepted";
    });

  set_analog_output_srv_ = get_node()->create_service<symphony_msgs::srv::SetAnalogOutput>(
    "~/set_analog_output",
    [this](
      const std::shared_ptr<symphony_msgs::srv::SetAnalogOutput::Request> request,
      std::shared_ptr<symphony_msgs::srv::SetAnalogOutput::Response> response)
    {
      if (!active_.load()) {
        response->success = false;
        response->message = "io_controller is not active";
        return;
      }
      if (!analog_args_ok(request->bank, request->channel)) {
        response->success = false;
        response->message = "invalid analog bank/channel (safety 1-2, ex 1-4)";
        return;
      }
      queue_analog(
        static_cast<double>(request->bank), static_cast<double>(request->channel),
        request->value);
      response->success = true;
      response->message = "accepted";
    });

  set_tool_io_srv_ = get_node()->create_service<symphony_msgs::srv::SetToolIO>(
    "~/set_tool_io",
    [this](
      const std::shared_ptr<symphony_msgs::srv::SetToolIO::Request> request,
      std::shared_ptr<symphony_msgs::srv::SetToolIO::Response> response)
    {
      if (!active_.load()) {
        response->success = false;
        response->message = "io_controller is not active";
        return;
      }
      using Req = symphony_msgs::srv::SetToolIO::Request;
      if (request->type == Req::TYPE_DIGITAL) {
        if (request->pin < 1 || request->pin > 32) {
          response->success = false;
          response->message = "tool digital pin must be 1-32";
          return;
        }
        queue_digital(kHwBankTool, static_cast<double>(request->pin), request->state ? 1.0 : 0.0);
      } else if (request->type == Req::TYPE_ANALOG_VOLTAGE) {
        if (!std::isfinite(request->value) || request->value < 0.0 || request->value > 255.0) {
          response->success = false;
          response->message = "tool analog voltage must be 0-255";
          return;
        }
        queue_analog(kHwBankTool, 1.0, request->value);
      } else {
        response->success = false;
        response->message = "type must be 0 (digital) or 1 (analog voltage)";
        return;
      }
      response->success = true;
      response->message = "accepted";
    });

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn IoController::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (state_interfaces_.size() != kNumIoStates || command_interfaces_.size() != 8) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected %zu state and 8 command interfaces, got %zu state / %zu command",
      kNumIoStates, state_interfaces_.size(), command_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  states_.assign(kNumIoStates, nullptr);
  for (size_t i = 0; i < kNumIoStates; ++i) {
    states_[i] = find_state(kStateNames[i]);
    if (!states_[i]) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Missing IO state interface %s", kStateNames[i]);
      clear_interface_cache();
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  digital_seq_cmd_ = find_command(kDigitalSeq);
  digital_bank_cmd_ = find_command(kDigitalBank);
  digital_pin_cmd_ = find_command(kDigitalPin);
  digital_value_cmd_ = find_command(kDigitalValue);
  analog_seq_cmd_ = find_command(kAnalogSeq);
  analog_bank_cmd_ = find_command(kAnalogBank);
  analog_channel_cmd_ = find_command(kAnalogChannel);
  analog_value_cmd_ = find_command(kAnalogValue);
  if (!digital_seq_cmd_ || !digital_bank_cmd_ || !digital_pin_cmd_ || !digital_value_cmd_ ||
      !analog_seq_cmd_ || !analog_bank_cmd_ || !analog_channel_cmd_ || !analog_value_cmd_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Missing IO command interface");
    clear_interface_cache();
    return controller_interface::CallbackReturn::ERROR;
  }

  active_.store(true);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn IoController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  active_.store(false);
  clear_interface_cache();
  return controller_interface::CallbackReturn::SUCCESS;
}

//----- helpers -----
void IoController::clear_interface_cache()
{
  states_.clear();
  digital_seq_cmd_ = nullptr;
  digital_bank_cmd_ = nullptr;
  digital_pin_cmd_ = nullptr;
  digital_value_cmd_ = nullptr;
  analog_seq_cmd_ = nullptr;
  analog_bank_cmd_ = nullptr;
  analog_channel_cmd_ = nullptr;
  analog_value_cmd_ = nullptr;
}

const hardware_interface::LoanedStateInterface *
IoController::find_state(const std::string & name) const
{
  for (const auto & iface : state_interfaces_) {
    if (iface.get_name() == name) {
      return &iface;
    }
  }
  return nullptr;
}

hardware_interface::LoanedCommandInterface *
IoController::find_command(const std::string & name)
{
  for (auto & iface : command_interfaces_) {
    if (iface.get_name() == name) {
      return &iface;
    }
  }
  return nullptr;
}

double IoController::state_value(size_t index) const
{
  return get_double(*states_[index]);
}

void IoController::queue_digital(double bank, double pin, double value)
{
  pending_digital_bank_.store(bank, std::memory_order_relaxed);
  pending_digital_pin_.store(pin, std::memory_order_relaxed);
  pending_digital_value_.store(value, std::memory_order_relaxed);
  pending_digital_seq_.fetch_add(1, std::memory_order_release);
}

void IoController::queue_analog(double bank, double channel, double value)
{
  pending_analog_bank_.store(bank, std::memory_order_relaxed);
  pending_analog_channel_.store(channel, std::memory_order_relaxed);
  pending_analog_value_.store(value, std::memory_order_relaxed);
  pending_analog_seq_.fetch_add(1, std::memory_order_release);
}

//----- update -----
controller_interface::return_type IoController::update(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (states_.size() != kNumIoStates || !digital_seq_cmd_ || !analog_seq_cmd_) {
    return controller_interface::return_type::ERROR;
  }

  const auto digital_seq = pending_digital_seq_.load(std::memory_order_acquire);
  set_command(*digital_bank_cmd_, pending_digital_bank_.load(std::memory_order_relaxed));
  set_command(*digital_pin_cmd_, pending_digital_pin_.load(std::memory_order_relaxed));
  set_command(*digital_value_cmd_, pending_digital_value_.load(std::memory_order_relaxed));
  set_command(*digital_seq_cmd_, static_cast<double>(digital_seq));

  const auto analog_seq = pending_analog_seq_.load(std::memory_order_acquire);
  set_command(*analog_bank_cmd_, pending_analog_bank_.load(std::memory_order_relaxed));
  set_command(*analog_channel_cmd_, pending_analog_channel_.load(std::memory_order_relaxed));
  set_command(*analog_value_cmd_, pending_analog_value_.load(std::memory_order_relaxed));
  set_command(*analog_seq_cmd_, static_cast<double>(analog_seq));

  symphony_msgs::msg::IOStates msg;
  msg.header.stamp = get_node()->now();
  msg.safety_digital_inputs = as_u32(state_value(kSafetyDi));
  msg.safety_digital_outputs = as_u32(state_value(kSafetyDo));
  msg.safety_analog_inputs[0] = state_value(kSafetyAi0);
  msg.safety_analog_inputs[1] = state_value(kSafetyAi1);
  msg.safety_analog_outputs[0] = state_value(kSafetyAo0);
  msg.safety_analog_outputs[1] = state_value(kSafetyAo1);
  msg.safety_digital_inputs_ex1 = as_u32(state_value(kEx1Di));
  msg.safety_digital_outputs_ex1 = as_u32(state_value(kEx1Do));
  msg.safety_analog_inputs_ex1[0] = state_value(kEx1Ai0);
  msg.safety_analog_inputs_ex1[1] = state_value(kEx1Ai1);
  msg.safety_analog_inputs_ex1[2] = state_value(kEx1Ai2);
  msg.safety_analog_inputs_ex1[3] = state_value(kEx1Ai3);
  msg.safety_analog_outputs_ex1[0] = state_value(kEx1Ao0);
  msg.safety_analog_outputs_ex1[1] = state_value(kEx1Ao1);
  msg.safety_analog_outputs_ex1[2] = state_value(kEx1Ao2);
  msg.safety_analog_outputs_ex1[3] = state_value(kEx1Ao3);
  msg.safety_digital_inputs_ex2 = as_u32(state_value(kEx2Di));
  msg.safety_digital_outputs_ex2 = as_u32(state_value(kEx2Do));
  msg.safety_analog_inputs_ex2[0] = state_value(kEx2Ai0);
  msg.safety_analog_inputs_ex2[1] = state_value(kEx2Ai1);
  msg.safety_analog_inputs_ex2[2] = state_value(kEx2Ai2);
  msg.safety_analog_inputs_ex2[3] = state_value(kEx2Ai3);
  msg.safety_analog_outputs_ex2[0] = state_value(kEx2Ao0);
  msg.safety_analog_outputs_ex2[1] = state_value(kEx2Ao1);
  msg.safety_analog_outputs_ex2[2] = state_value(kEx2Ao2);
  msg.safety_analog_outputs_ex2[3] = state_value(kEx2Ao3);
  get_io_pub_->publish(msg);

  symphony_msgs::msg::ToolIOStates tool_msg;
  tool_msg.header.stamp = msg.header.stamp;
  tool_msg.digital_inputs = as_u32(state_value(kToolDi));
  tool_msg.digital_outputs = as_u32(state_value(kToolDo));
  tool_msg.analog_inputs[0] = state_value(kToolAi0);
  tool_msg.analog_inputs[1] = state_value(kToolAi1);
  get_tool_io_pub_->publish(tool_msg);

  return controller_interface::return_type::OK;
}

}  // namespace symphony_controllers

PLUGINLIB_EXPORT_CLASS(
  symphony_controllers::IoController, controller_interface::ControllerInterface)
