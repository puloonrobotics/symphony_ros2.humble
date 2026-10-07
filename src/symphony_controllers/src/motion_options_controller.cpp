#include "symphony_controllers/motion_options_controller.hpp"

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "symphony_controllers/loaned_iface.hpp"

namespace symphony_controllers
{

namespace
{
constexpr const char * kWorkCmd = "motion_options/work";
constexpr const char * kToolCmd = "motion_options/tool";
constexpr const char * kRelCmd = "motion_options/rel";
constexpr const char * kFixedspeedCmd = "motion_options/fixedspeed";
constexpr const char * kCfgCmd = "motion_options/cfg";
constexpr const char * kCoordCmd = "motion_options/coord";
constexpr const char * kWeavingCmd = "motion_options/weaving";
constexpr const char * kFixedorientCmd = "motion_options/fixedorient";
constexpr const char * kExtCmd = "motion_options/ext";
constexpr const char * kTolCmd = "motion_options/tol";
constexpr const char * kServoRelCmd = "motion_options/servo_relative";
}  // namespace

controller_interface::InterfaceConfiguration
MotionOptionsController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = {
    kWorkCmd, kToolCmd, kRelCmd, kFixedspeedCmd, kCfgCmd, kCoordCmd,
    kWeavingCmd, kFixedorientCmd, kExtCmd, kTolCmd, kServoRelCmd};
  return config;
}

controller_interface::InterfaceConfiguration
MotionOptionsController::state_interface_configuration() const
{
  return command_interface_configuration();
}

controller_interface::CallbackReturn MotionOptionsController::on_init()
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MotionOptionsController::on_configure(
  const rclcpp_lifecycle::State &)
{
  options_pub_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
    "~/options", rclcpp::SystemDefaultsQoS());

  set_srv_ = get_node()->create_service<symphony_msgs::srv::SetMotionOptions>(
    "~/set_motion_options",
    [this](
      const std::shared_ptr<symphony_msgs::srv::SetMotionOptions::Request> request,
      std::shared_ptr<symphony_msgs::srv::SetMotionOptions::Response> response)
    {
      if (!active_.load()) {
        response->success = false;
        response->message = "motion_options_controller is not active";
        return;
      }
      if (request->work < -1 || request->tool < -1) {
        response->success = false;
        response->message = "work/tool must be >= -1";
        return;
      }
      if (request->cfg < -1) {
        response->success = false;
        response->message = "cfg must be >= -1 (-1 omits pose cfg)";
        return;
      }
      if (request->coord < -1 || request->coord > 3) {
        response->success = false;
        response->message = "coord must be -1/0 (auto) or 1=JNT 2=XYZ 3=TOOL";
        return;
      }
      if (request->tol < -1) {
        response->success = false;
        response->message = "tol must be >= -1 (-1 = blend/default)";
        return;
      }
      if (request->ext < 0) {
        response->success = false;
        response->message = "ext must be >= 0";
        return;
      }
      pending_work_.store(request->work);
      pending_tool_.store(request->tool);
      pending_rel_.store(request->rel);
      pending_fixedspeed_.store(request->fixedspeed);
      pending_cfg_.store(request->cfg);
      pending_coord_.store(request->coord <= 0 ? -1 : request->coord);
      pending_weaving_.store(request->weaving);
      pending_fixedorient_.store(request->fixedorient);
      pending_ext_.store(request->ext);
      pending_tol_.store(request->tol);
      pending_servo_relative_.store(request->servo_relative);
      pending_set_.store(true);
      response->success = true;
      response->message = "accepted";
    });

  reset_srv_ = get_node()->create_service<std_srvs::srv::Trigger>(
    "~/reset_motion_options",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      if (!active_.load()) {
        response->success = false;
        response->message = "motion_options_controller is not active";
        return;
      }
      pending_reset_.store(true);
      response->success = true;
      response->message = "accepted";
    });

  return controller_interface::CallbackReturn::SUCCESS;
}

bool MotionOptionsController::bind_interfaces()
{
  work_cmd_ = find_command(kWorkCmd);
  tool_cmd_ = find_command(kToolCmd);
  rel_cmd_ = find_command(kRelCmd);
  fixedspeed_cmd_ = find_command(kFixedspeedCmd);
  cfg_cmd_ = find_command(kCfgCmd);
  coord_cmd_ = find_command(kCoordCmd);
  weaving_cmd_ = find_command(kWeavingCmd);
  fixedorient_cmd_ = find_command(kFixedorientCmd);
  ext_cmd_ = find_command(kExtCmd);
  tol_cmd_ = find_command(kTolCmd);
  servo_relative_cmd_ = find_command(kServoRelCmd);
  work_state_ = find_state(kWorkCmd);
  tool_state_ = find_state(kToolCmd);
  rel_state_ = find_state(kRelCmd);
  fixedspeed_state_ = find_state(kFixedspeedCmd);
  cfg_state_ = find_state(kCfgCmd);
  coord_state_ = find_state(kCoordCmd);
  weaving_state_ = find_state(kWeavingCmd);
  fixedorient_state_ = find_state(kFixedorientCmd);
  ext_state_ = find_state(kExtCmd);
  tol_state_ = find_state(kTolCmd);
  servo_relative_state_ = find_state(kServoRelCmd);
  return work_cmd_ && tool_cmd_ && rel_cmd_ && fixedspeed_cmd_ && cfg_cmd_ &&
         coord_cmd_ && weaving_cmd_ && fixedorient_cmd_ && ext_cmd_ && tol_cmd_ &&
         servo_relative_cmd_ && work_state_ && tool_state_ && rel_state_ &&
         fixedspeed_state_ && cfg_state_ && coord_state_ && weaving_state_ &&
         fixedorient_state_ && ext_state_ && tol_state_ && servo_relative_state_;
}

controller_interface::CallbackReturn MotionOptionsController::on_activate(
  const rclcpp_lifecycle::State &)
{
  clear_interface_cache();
  if (!bind_interfaces()) {
    RCLCPP_ERROR(get_node()->get_logger(), "motion_options interfaces missing");
    clear_interface_cache();
    return controller_interface::CallbackReturn::ERROR;
  }
  write_defaults();
  active_.store(true);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MotionOptionsController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  active_.store(false);
  clear_interface_cache();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type MotionOptionsController::update(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_.load() || !work_cmd_) {
    return controller_interface::return_type::OK;
  }

  if (pending_reset_.exchange(false)) {
    write_defaults();
  } else if (pending_set_.exchange(false)) {
    set_command(*work_cmd_, static_cast<double>(pending_work_.load()));
    set_command(*tool_cmd_, static_cast<double>(pending_tool_.load()));
    set_command(*rel_cmd_, pending_rel_.load() ? 1.0 : 0.0);
    set_command(*fixedspeed_cmd_, pending_fixedspeed_.load() ? 1.0 : 0.0);
    set_command(*cfg_cmd_, static_cast<double>(pending_cfg_.load()));
    set_command(*coord_cmd_, static_cast<double>(pending_coord_.load()));
    set_command(*weaving_cmd_, pending_weaving_.load() ? 1.0 : 0.0);
    set_command(*fixedorient_cmd_, pending_fixedorient_.load() ? 1.0 : 0.0);
    set_command(*ext_cmd_, static_cast<double>(pending_ext_.load()));
    set_command(*tol_cmd_, static_cast<double>(pending_tol_.load()));
    set_command(*servo_relative_cmd_, pending_servo_relative_.load() ? 1.0 : 0.0);
  }

  std_msgs::msg::Float64MultiArray msg;
  msg.data = {
    get_double(*work_state_),
    get_double(*tool_state_),
    get_double(*rel_state_),
    get_double(*fixedspeed_state_),
    get_double(*cfg_state_),
    get_double(*coord_state_),
    get_double(*weaving_state_),
    get_double(*fixedorient_state_),
    get_double(*ext_state_),
    get_double(*tol_state_),
    get_double(*servo_relative_state_),
  };
  options_pub_->publish(msg);
  return controller_interface::return_type::OK;
}

hardware_interface::LoanedCommandInterface *
MotionOptionsController::find_command(const std::string & name)
{
  for (auto & iface : command_interfaces_) {
    if (iface.get_name() == name) {
      return &iface;
    }
  }
  return nullptr;
}

const hardware_interface::LoanedStateInterface *
MotionOptionsController::find_state(const std::string & name)
{
  for (const auto & iface : state_interfaces_) {
    if (iface.get_name() == name) {
      return &iface;
    }
  }
  return nullptr;
}

void MotionOptionsController::clear_interface_cache()
{
  work_cmd_ = tool_cmd_ = rel_cmd_ = fixedspeed_cmd_ = cfg_cmd_ = nullptr;
  coord_cmd_ = weaving_cmd_ = fixedorient_cmd_ = ext_cmd_ = tol_cmd_ = nullptr;
  servo_relative_cmd_ = nullptr;
  work_state_ = tool_state_ = rel_state_ = fixedspeed_state_ = cfg_state_ = nullptr;
  coord_state_ = weaving_state_ = fixedorient_state_ = ext_state_ = tol_state_ = nullptr;
  servo_relative_state_ = nullptr;
}

void MotionOptionsController::write_defaults()
{
  if (!work_cmd_) {
    return;
  }
  set_command(*work_cmd_, -1.0);
  set_command(*tool_cmd_, -1.0);
  set_command(*rel_cmd_, 0.0);
  set_command(*fixedspeed_cmd_, 0.0);
  set_command(*cfg_cmd_, -1.0);
  set_command(*coord_cmd_, -1.0);
  set_command(*weaving_cmd_, 0.0);
  set_command(*fixedorient_cmd_, 0.0);
  set_command(*ext_cmd_, 0.0);
  set_command(*tol_cmd_, -1.0);
  set_command(*servo_relative_cmd_, 0.0);
}

}  // namespace symphony_controllers

PLUGINLIB_EXPORT_CLASS(
  symphony_controllers::MotionOptionsController,
  controller_interface::ControllerInterface)
