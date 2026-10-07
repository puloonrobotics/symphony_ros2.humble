#include "symphony_controllers/force_controller.hpp"

#include <cmath>
#include <iterator>
#include <vector>

#include "symphony_controllers/loaned_iface.hpp"

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace symphony_controllers
{

//----- interface names -----
namespace
{
constexpr const char * kForceEnabled = "force/enabled";

constexpr const char * kCmdNames[] = {
  "force/payload_seq",
  "force/mass",
  "force/cog_x",
  "force/cog_y",
  "force/cog_z",
  "force/ixx",
  "force/iyy",
  "force/izz",
  "force/gravity_seq",
  "force/gx",
  "force/gy",
  "force/gz",
  "force/bias_seq",
  "force/force_seq",
  "force/enable",
  "force/fx",
  "force/fy",
  "force/fz",
  "force/tx",
  "force/ty",
  "force/tz",
  "force/param_seq",
  "force/set_imp_mass",
  "force/imp_mass_0",
  "force/imp_mass_1",
  "force/imp_mass_2",
  "force/imp_mass_3",
  "force/imp_mass_4",
  "force/imp_mass_5",
  "force/set_imp_stiffness",
  "force/imp_stiffness_0",
  "force/imp_stiffness_1",
  "force/imp_stiffness_2",
  "force/imp_stiffness_3",
  "force/imp_stiffness_4",
  "force/imp_stiffness_5",
  "force/set_imp_damping",
  "force/imp_damping_0",
  "force/imp_damping_1",
  "force/imp_damping_2",
  "force/imp_damping_3",
  "force/imp_damping_4",
  "force/imp_damping_5",
  "force/set_compliance",
  "force/compliance_0",
  "force/compliance_1",
  "force/compliance_2",
  "force/compliance_3",
  "force/compliance_4",
  "force/compliance_5",
};
constexpr size_t kNumCmds = sizeof(kCmdNames) / sizeof(kCmdNames[0]);
constexpr size_t kNumAxes = 6;

enum CmdIndex : size_t
{
  kPayloadSeq = 0,
  kMass,
  kCogX,
  kCogY,
  kCogZ,
  kIxx,
  kIyy,
  kIzz,
  kGravitySeq,
  kGx,
  kGy,
  kGz,
  kBiasSeq,
  kForceSeq,
  kEnable,
  kFx,
  kParamSeq = kFx + kNumAxes,
  kSetImpMass,
  kImpMass,
  kSetImpStiffness = kImpMass + kNumAxes,
  kImpStiffness,
  kSetImpDamping = kImpStiffness + kNumAxes,
  kImpDamping,
  kSetCompliance = kImpDamping + kNumAxes,
  kCompliance,
};
static_assert(kCompliance + kNumAxes == kNumCmds, "CmdIndex is out of step with kCmdNames");

std::string check_axis_array(
  const std::vector<double> & values, const char * name, bool strictly_positive)
{
  if (values.empty()) {
    return {};
  }
  if (values.size() != kNumAxes) {
    return std::string(name) + " must be empty or have exactly 6 elements";
  }
  for (const double value : values) {
    if (!std::isfinite(value)) {
      return std::string(name) + " must be finite";
    }
    if (strictly_positive ? value <= 0.0 : value < 0.0) {
      return std::string(name) + (strictly_positive ? " must be > 0" : " must be >= 0");
    }
  }
  return {};
}
}  // namespace

//----- lifecycle -----
controller_interface::InterfaceConfiguration
ForceController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.assign(std::begin(kCmdNames), std::end(kCmdNames));
  return config;
}

controller_interface::InterfaceConfiguration
ForceController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = {kForceEnabled};
  return config;
}

controller_interface::CallbackReturn ForceController::on_init()
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ForceController::on_configure(
  const rclcpp_lifecycle::State &)
{
  force_enabled_pub_ = get_node()->create_publisher<std_msgs::msg::Bool>(
    "~/force_control_enabled", rclcpp::SystemDefaultsQoS());

  set_tool_info_srv_ = get_node()->create_service<symphony_msgs::srv::SetToolInfo>(
    "~/set_tool_info",
    [this](
      const std::shared_ptr<symphony_msgs::srv::SetToolInfo::Request> request,
      std::shared_ptr<symphony_msgs::srv::SetToolInfo::Response> response)
    {
      if (!active_.load()) {
        response->success = false;
        response->message = "force_controller is not active";
        return;
      }
      if (request->mass < 0.0) {
        response->success = false;
        response->message = "mass must be >= 0";
        return;
      }
      payload_[0].store(request->mass, std::memory_order_relaxed);
      payload_[1].store(request->center_of_mass[0], std::memory_order_relaxed);
      payload_[2].store(request->center_of_mass[1], std::memory_order_relaxed);
      payload_[3].store(request->center_of_mass[2], std::memory_order_relaxed);
      payload_[4].store(request->inertia[0], std::memory_order_relaxed);
      payload_[5].store(request->inertia[1], std::memory_order_relaxed);
      payload_[6].store(request->inertia[2], std::memory_order_relaxed);
      payload_seq_.fetch_add(1, std::memory_order_release);
      response->success = true;
      response->message = "accepted";
    });

  set_gravity_vector_srv_ = get_node()->create_service<symphony_msgs::srv::SetGravity>(
    "~/set_gravity_vector",
    [this](
      const std::shared_ptr<symphony_msgs::srv::SetGravity::Request> request,
      std::shared_ptr<symphony_msgs::srv::SetGravity::Response> response)
    {
      if (!active_.load()) {
        response->success = false;
        response->message = "force_controller is not active";
        return;
      }
      gravity_[0].store(request->gravity[0], std::memory_order_relaxed);
      gravity_[1].store(request->gravity[1], std::memory_order_relaxed);
      gravity_[2].store(request->gravity[2], std::memory_order_relaxed);
      gravity_seq_.fetch_add(1, std::memory_order_release);
      response->success = true;
      response->message = "accepted";
    });

  set_force_bias_srv_ = get_node()->create_service<std_srvs::srv::Trigger>(
    "~/set_force_bias",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      if (!active_.load()) {
        response->success = false;
        response->message = "force_controller is not active";
        return;
      }
      bias_seq_.fetch_add(1, std::memory_order_release);
      response->success = true;
      response->message = "accepted";
    });

  start_force_control_srv_ = get_node()->create_service<symphony_msgs::srv::StartForceControl>(
    "~/start_force_control",
    [this](
      const std::shared_ptr<symphony_msgs::srv::StartForceControl::Request> request,
      std::shared_ptr<symphony_msgs::srv::StartForceControl::Response> response)
    {
      if (!active_.load()) {
        response->success = false;
        response->message = "force_controller is not active";
        return;
      }
      for (size_t i = 0; i < 6; ++i) {
        wrench_[i].store(request->wrench[i], std::memory_order_relaxed);
      }
      force_enable_.store(1.0, std::memory_order_relaxed);
      force_seq_.fetch_add(1, std::memory_order_release);
      response->success = true;
      response->message = "accepted";
    });

  set_force_parameters_srv_ = get_node()->create_service<symphony_msgs::srv::SetForceParameters>(
    "~/set_force_parameters",
    [this](
      const std::shared_ptr<symphony_msgs::srv::SetForceParameters::Request> request,
      std::shared_ptr<symphony_msgs::srv::SetForceParameters::Response> response)
    {
      if (!active_.load()) {
        response->success = false;
        response->message = "force_controller is not active";
        return;
      }

      std::string error =
        check_axis_array(request->impedance_mass, "impedance_mass", true);
      if (error.empty()) {
        error = check_axis_array(request->impedance_stiffness, "impedance_stiffness", false);
      }
      if (error.empty()) {
        error = check_axis_array(
          request->impedance_damping_ratio, "impedance_damping_ratio", false);
      }
      if (error.empty()) {
        error = check_axis_array(request->compliance, "compliance", false);
      }
      if (error.empty() && request->impedance_mass.empty() &&
        request->impedance_stiffness.empty() &&
        request->impedance_damping_ratio.empty() && request->compliance.empty())
      {
        error = "every array was empty, so there is nothing to set";
      }
      if (error.empty() && !request->impedance_damping_ratio.empty() &&
        !request->impedance_stiffness.empty())
      {
        for (size_t i = 0; i < kNumAxes; ++i) {
          if (request->impedance_damping_ratio[i] > 0.0 &&
            request->impedance_stiffness[i] <= 0.0)
          {
            error = "impedance_damping_ratio needs impedance_stiffness > 0 on the same axis";
            break;
          }
        }
      }
      if (!error.empty()) {
        response->success = false;
        response->message = error;
        return;
      }

      const auto store = [](
        const std::vector<double> & values, std::atomic<double> & flag,
        std::array<std::atomic<double>, kNumAxes> & slots)
        {
          for (size_t i = 0; i < values.size(); ++i) {
            slots[i].store(values[i], std::memory_order_relaxed);
          }
          flag.store(values.empty() ? 0.0 : 1.0, std::memory_order_relaxed);
        };
      store(request->impedance_mass, set_imp_mass_, imp_mass_);
      store(request->impedance_stiffness, set_imp_stiffness_, imp_stiffness_);
      store(request->impedance_damping_ratio, set_imp_damping_, imp_damping_);
      store(request->compliance, set_compliance_, compliance_);
      param_seq_.fetch_add(1, std::memory_order_release);

      response->success = true;
      response->message = "accepted";
    });

  stop_force_control_srv_ = get_node()->create_service<std_srvs::srv::Trigger>(
    "~/stop_force_control",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      if (!active_.load()) {
        response->success = false;
        response->message = "force_controller is not active";
        return;
      }
      force_enable_.store(0.0, std::memory_order_relaxed);
      force_seq_.fetch_add(1, std::memory_order_release);
      response->success = true;
      response->message = "accepted";
    });

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ForceController::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (state_interfaces_.size() != 1 || command_interfaces_.size() != kNumCmds) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected 1 state and %zu command interfaces, got %zu state / %zu command",
      kNumCmds, state_interfaces_.size(), command_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  force_enabled_state_ = find_state(kForceEnabled);
  if (!force_enabled_state_) {
    RCLCPP_ERROR(get_node()->get_logger(), "Missing %s", kForceEnabled);
    return controller_interface::CallbackReturn::ERROR;
  }

  for (size_t i = 0; i < kNumCmds; ++i) {
    cmds_[i] = find_command(kCmdNames[i]);
    if (!cmds_[i]) {
      RCLCPP_ERROR(get_node()->get_logger(), "Missing %s", kCmdNames[i]);
      clear_interface_cache();
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  active_.store(true);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ForceController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  active_.store(false);
  clear_interface_cache();
  return controller_interface::CallbackReturn::SUCCESS;
}

//----- helpers -----
void ForceController::clear_interface_cache()
{
  force_enabled_state_ = nullptr;
  cmds_.fill(nullptr);
}

const hardware_interface::LoanedStateInterface *
ForceController::find_state(const std::string & name) const
{
  for (const auto & iface : state_interfaces_) {
    if (iface.get_name() == name) {
      return &iface;
    }
  }
  return nullptr;
}

hardware_interface::LoanedCommandInterface *
ForceController::find_command(const std::string & name)
{
  for (auto & iface : command_interfaces_) {
    if (iface.get_name() == name) {
      return &iface;
    }
  }
  return nullptr;
}

//----- update -----
controller_interface::return_type ForceController::update(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!force_enabled_state_ || !cmds_[kPayloadSeq]) {
    return controller_interface::return_type::ERROR;
  }

  const auto payload_seq = payload_seq_.load(std::memory_order_acquire);
  set_command(*cmds_[kMass], payload_[0].load(std::memory_order_relaxed));
  set_command(*cmds_[kCogX], payload_[1].load(std::memory_order_relaxed));
  set_command(*cmds_[kCogY], payload_[2].load(std::memory_order_relaxed));
  set_command(*cmds_[kCogZ], payload_[3].load(std::memory_order_relaxed));
  set_command(*cmds_[kIxx], payload_[4].load(std::memory_order_relaxed));
  set_command(*cmds_[kIyy], payload_[5].load(std::memory_order_relaxed));
  set_command(*cmds_[kIzz], payload_[6].load(std::memory_order_relaxed));
  set_command(*cmds_[kPayloadSeq], static_cast<double>(payload_seq));

  const auto gravity_seq = gravity_seq_.load(std::memory_order_acquire);
  set_command(*cmds_[kGx], gravity_[0].load(std::memory_order_relaxed));
  set_command(*cmds_[kGy], gravity_[1].load(std::memory_order_relaxed));
  set_command(*cmds_[kGz], gravity_[2].load(std::memory_order_relaxed));
  set_command(*cmds_[kGravitySeq], static_cast<double>(gravity_seq));

  set_command(*cmds_[kBiasSeq], static_cast<double>(bias_seq_.load(std::memory_order_acquire)));

  const auto force_seq = force_seq_.load(std::memory_order_acquire);
  set_command(*cmds_[kEnable], force_enable_.load(std::memory_order_relaxed));
  for (size_t i = 0; i < 6; ++i) {
    set_command(*cmds_[kFx + i], wrench_[i].load(std::memory_order_relaxed));
  }
  set_command(*cmds_[kForceSeq], static_cast<double>(force_seq));

  const auto param_seq = param_seq_.load(std::memory_order_acquire);
  set_command(*cmds_[kSetImpMass], set_imp_mass_.load(std::memory_order_relaxed));
  set_command(*cmds_[kSetImpStiffness], set_imp_stiffness_.load(std::memory_order_relaxed));
  set_command(*cmds_[kSetImpDamping], set_imp_damping_.load(std::memory_order_relaxed));
  set_command(*cmds_[kSetCompliance], set_compliance_.load(std::memory_order_relaxed));
  for (size_t i = 0; i < kNumAxes; ++i) {
    set_command(*cmds_[kImpMass + i], imp_mass_[i].load(std::memory_order_relaxed));
    set_command(*cmds_[kImpStiffness + i], imp_stiffness_[i].load(std::memory_order_relaxed));
    set_command(*cmds_[kImpDamping + i], imp_damping_[i].load(std::memory_order_relaxed));
    set_command(*cmds_[kCompliance + i], compliance_[i].load(std::memory_order_relaxed));
  }
  set_command(*cmds_[kParamSeq], static_cast<double>(param_seq));

  std_msgs::msg::Bool enabled;
  enabled.data = get_double(*force_enabled_state_) > 0.5;
  force_enabled_pub_->publish(enabled);

  return controller_interface::return_type::OK;
}

}  // namespace symphony_controllers

PLUGINLIB_EXPORT_CLASS(
  symphony_controllers::ForceController,
  controller_interface::ControllerInterface)
