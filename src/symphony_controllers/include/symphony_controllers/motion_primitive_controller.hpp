#ifndef SYMPHONY_CONTROLLERS__MOTION_PRIMITIVE_CONTROLLER_HPP_
#define SYMPHONY_CONTROLLERS__MOTION_PRIMITIVE_CONTROLLER_HPP_

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "symphony_controllers/visibility_control.h"
#include "symphony_msgs/action/execute_motion_primitive_sequence.hpp"

namespace symphony_controllers
{

class MotionPrimitiveController : public controller_interface::ControllerInterface
{
public:
  using ExecuteMotion = symphony_msgs::action::ExecuteMotionPrimitiveSequence;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteMotion>;

  SYMPHONY_CONTROLLERS_PUBLIC
  MotionPrimitiveController() = default;

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
  enum class Kind : uint8_t
  {
    None,
    Ptp,
    Lin,
    Arc,
    Stop
  };

  enum class Phase : uint8_t
  {
    Idle,
    WaitStart,
    WaitDone,
    WaitReset
  };

  struct Job
  {
    Kind kind{Kind::None};
    std::array<double, 6> joints{};
    geometry_msgs::msg::Pose pose;
    geometry_msgs::msg::Pose via;
    uint32_t pose_cfg{0};
    double velocity{0.0};
    double acceleration{0.0};
    bool has_acceleration{false};
    double blend_radius{0.0};
    bool fixed_speed{false};
  };

  hardware_interface::LoanedCommandInterface * find_command(const std::string & name);
  const hardware_interface::LoanedStateInterface * find_state(const std::string & name) const;
  void clear_interface_cache();
  bool cache_interfaces();
  void write_nan_commands();
  void write_job(const Job & job);
  bool decode_goal(
    const ExecuteMotion::Goal & goal, std::vector<Job> & jobs, std::string & error) const;
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ExecuteMotion::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
  void begin_job_locked(const rclcpp::Time & time, const Job & job);
  void publish_feedback_locked();
  void succeed_locked(const std::string & message);
  void abort_locked(int32_t error_code, const std::string & message);
  void cancel_locked(const std::string & message);
  void clear_motion_locked();

  double handshake_timeout_{2.0};
  double execution_timeout_{60.0};

  hardware_interface::LoanedCommandInterface * type_cmd_{nullptr};
  std::array<hardware_interface::LoanedCommandInterface *, 6> q_cmds_{};
  std::array<hardware_interface::LoanedCommandInterface *, 7> pose_cmds_{};
  std::array<hardware_interface::LoanedCommandInterface *, 7> via_cmds_{};
  hardware_interface::LoanedCommandInterface * blend_cmd_{nullptr};
  hardware_interface::LoanedCommandInterface * velocity_cmd_{nullptr};
  hardware_interface::LoanedCommandInterface * acceleration_cmd_{nullptr};
  hardware_interface::LoanedCommandInterface * move_time_cmd_{nullptr};
  hardware_interface::LoanedCommandInterface * pose_cfg_cmd_{nullptr};
  hardware_interface::LoanedCommandInterface * pause_cmd_{nullptr};
  hardware_interface::LoanedCommandInterface * resume_cmd_{nullptr};
  const hardware_interface::LoanedStateInterface * status_state_{nullptr};
  const hardware_interface::LoanedStateInterface * ready_state_{nullptr};

  rclcpp_action::Server<ExecuteMotion>::SharedPtr action_server_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr move_pause_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr move_resume_srv_;

  std::mutex mutex_;
  std::shared_ptr<GoalHandle> goal_handle_;
  std::vector<Job> jobs_;
  size_t job_index_{0};
  Phase phase_{Phase::Idle};
  bool cancel_requested_{false};
  bool stopping_{false};
  bool motion_started_{false};
  rclcpp::Time deadline_;
  bool deadline_active_{false};

  std::atomic<bool> active_{false};
  std::atomic<bool> pause_pulse_{false};
  std::atomic<bool> resume_pulse_{false};
};

}  // namespace symphony_controllers

#endif  // SYMPHONY_CONTROLLERS__MOTION_PRIMITIVE_CONTROLLER_HPP_
