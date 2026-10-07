#ifndef SYMPHONY_CONTROLLERS__NATIVE_JOINT_TRAJECTORY_CONTROLLER_HPP_
#define SYMPHONY_CONTROLLERS__NATIVE_JOINT_TRAJECTORY_CONTROLLER_HPP_

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "symphony_controllers/visibility_control.h"
#include "symphony_msgs/hw_constants.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace symphony_controllers
{
class NativeJointTrajectoryController : public controller_interface::ControllerInterface
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

  SYMPHONY_CONTROLLERS_PUBLIC
  NativeJointTrajectoryController() = default;

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
  enum class Phase : uint8_t
  {
    Idle,
    AnnounceSize,
    WaitReady,
    SendPoint,
    WaitPointAck,
    SendGo,
    WaitExec,
    WaitDone
  };

  struct TrajectoryPoint
  {
    std::array<double, 6> positions{};
    std::array<double, 6> velocities{};
    std::array<double, 6> accelerations{};
    double time_from_start{0.0};
  };

  hardware_interface::LoanedCommandInterface * find_command(const std::string & name);
  const hardware_interface::LoanedStateInterface * find_state(const std::string & name) const;
  void clear_interface_cache();
  bool cache_interfaces();
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
  bool map_joint_order(
    const std::vector<std::string> & names, std::array<int, 6> & map) const;
  bool remap_goal(
    const FollowJointTrajectory::Goal & goal, std::vector<TrajectoryPoint> & points) const;
  void simplify(std::vector<TrajectoryPoint> & points) const;
  void finish_goal(bool success, int32_t error_code, const std::string & message);
  void publish_feedback();
  void set_deadline(const rclcpp::Time & now, double timeout);
  bool deadline_expired(const rclcpp::Time & now) const;

  std::vector<std::string> joints_;
  size_t max_points_{symphony_msgs::kNativeJtMaxPoints};
  double path_tolerance_{0.01};
  double handshake_timeout_{2.0};
  double execution_timeout_margin_{10.0};

  std::array<hardware_interface::LoanedCommandInterface *, 6> pos_cmds_{};
  std::array<hardware_interface::LoanedCommandInterface *, 6> vel_cmds_{};
  std::array<hardware_interface::LoanedCommandInterface *, 6> acc_cmds_{};
  hardware_interface::LoanedCommandInterface * transfer_cmd_{nullptr};
  hardware_interface::LoanedCommandInterface * time_cmd_{nullptr};
  hardware_interface::LoanedCommandInterface * abort_cmd_{nullptr};
  hardware_interface::LoanedCommandInterface * size_cmd_{nullptr};
  std::array<const hardware_interface::LoanedStateInterface *, 6> pos_states_{};
  std::array<const hardware_interface::LoanedStateInterface *, 6> vel_states_{};
  const hardware_interface::LoanedStateInterface * speed_rate_state_{nullptr};

  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;

  std::mutex mutex_;
  std::shared_ptr<GoalHandle> goal_handle_;
  std::vector<TrajectoryPoint> points_;
  size_t send_index_{0};
  Phase phase_{Phase::Idle};
  bool abort_requested_{false};
  std::string abort_reason_{"canceled"};
  rclcpp::Time deadline_;
  bool deadline_active_{false};

  std::atomic<bool> active_{false};
};

}  // namespace symphony_controllers

#endif  // SYMPHONY_CONTROLLERS__NATIVE_JOINT_TRAJECTORY_CONTROLLER_HPP_
