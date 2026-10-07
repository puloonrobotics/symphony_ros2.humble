#include "symphony_controllers/native_joint_trajectory_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/create_server.hpp"
#include "symphony_controllers/loaned_iface.hpp"
#include "symphony_msgs/hw_constants.hpp"

namespace symphony_controllers
{

//----- constants -----
namespace
{
constexpr size_t kNumJoints = 6;
using symphony_msgs::kPtDone;
using symphony_msgs::kPtExec;
using symphony_msgs::kPtGo;
using symphony_msgs::kPtPoint;
using symphony_msgs::kPtReady;
using symphony_msgs::kPtSize;
}  // namespace

//----- lifecycle -----
controller_interface::InterfaceConfiguration
NativeJointTrajectoryController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (size_t i = 0; i < kNumJoints; ++i) {
    const auto idx = std::to_string(i);
    config.names.push_back("native_joint_trajectory/setpoint_positions_" + idx);
    config.names.push_back("native_joint_trajectory/setpoint_velocities_" + idx);
    config.names.push_back("native_joint_trajectory/setpoint_accelerations_" + idx);
  }
  config.names.push_back("native_joint_trajectory/transfer_state");
  config.names.push_back("native_joint_trajectory/time_from_start");
  config.names.push_back("native_joint_trajectory/abort");
  config.names.push_back("native_joint_trajectory/size");
  return config;
}

controller_interface::InterfaceConfiguration
NativeJointTrajectoryController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joints_) {
    config.names.push_back(joint + "/position");
    config.names.push_back(joint + "/velocity");
  }
  config.names.push_back("speed_rate/value");
  return config;
}

controller_interface::CallbackReturn NativeJointTrajectoryController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>{});
    auto_declare<int>("max_points", static_cast<int>(symphony_msgs::kNativeJtMaxPoints));
    auto_declare<double>("path_tolerance", 0.01);
    auto_declare<double>("handshake_timeout", 2.0);
    auto_declare<double>("execution_timeout_margin", 10.0);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn NativeJointTrajectoryController::on_configure(
  const rclcpp_lifecycle::State &)
{
  joints_ = get_node()->get_parameter("joints").as_string_array();
  if (joints_.size() != kNumJoints) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "parameter 'joints' must have %zu names, got %zu",
      kNumJoints, joints_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  const int max_points = get_node()->get_parameter("max_points").as_int();
  if (max_points < 2) {
    RCLCPP_ERROR(get_node()->get_logger(), "parameter 'max_points' must be >= 2");
    return controller_interface::CallbackReturn::ERROR;
  }
  max_points_ = static_cast<size_t>(max_points);
  path_tolerance_ = get_node()->get_parameter("path_tolerance").as_double();
  handshake_timeout_ = get_node()->get_parameter("handshake_timeout").as_double();
  execution_timeout_margin_ =
    get_node()->get_parameter("execution_timeout_margin").as_double();

  using namespace std::placeholders;
  const auto action_name = std::string(get_node()->get_name()) + "/follow_joint_trajectory";
  action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
    get_node()->get_node_base_interface(),
    get_node()->get_node_clock_interface(),
    get_node()->get_node_logging_interface(),
    get_node()->get_node_waitables_interface(),
    action_name,
    std::bind(&NativeJointTrajectoryController::handle_goal, this, _1, _2),
    std::bind(&NativeJointTrajectoryController::handle_cancel, this, _1),
    std::bind(&NativeJointTrajectoryController::handle_accepted, this, _1));

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn NativeJointTrajectoryController::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!cache_interfaces()) {
    clear_interface_cache();
    return controller_interface::CallbackReturn::ERROR;
  }
  set_command(*abort_cmd_, 0.0);
  set_command(*transfer_cmd_, 0.0);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    phase_ = Phase::Idle;
    abort_requested_ = false;
    deadline_active_ = false;
    goal_handle_.reset();
    points_.clear();
    send_index_ = 0;
  }
  active_.store(true);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn NativeJointTrajectoryController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  active_.store(false);
  if (abort_cmd_) {
    set_command(*abort_cmd_, 1.0);
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (goal_handle_) {
      auto result = std::make_shared<FollowJointTrajectory::Result>();
      result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
      result->error_string = "native_joint_trajectory_controller deactivated";
      goal_handle_->abort(result);
      goal_handle_.reset();
    }
    phase_ = Phase::Idle;
    abort_requested_ = false;
    deadline_active_ = false;
    points_.clear();
  }
  clear_interface_cache();
  return controller_interface::CallbackReturn::SUCCESS;
}

//----- helpers -----
bool NativeJointTrajectoryController::cache_interfaces()
{
  for (size_t i = 0; i < kNumJoints; ++i) {
    const auto idx = std::to_string(i);
    pos_cmds_[i] = find_command("native_joint_trajectory/setpoint_positions_" + idx);
    vel_cmds_[i] = find_command("native_joint_trajectory/setpoint_velocities_" + idx);
    acc_cmds_[i] = find_command("native_joint_trajectory/setpoint_accelerations_" + idx);
    pos_states_[i] = find_state(joints_[i] + "/position");
    vel_states_[i] = find_state(joints_[i] + "/velocity");
    if (!pos_cmds_[i] || !vel_cmds_[i] || !acc_cmds_[i] || !pos_states_[i] || !vel_states_[i]) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "missing native JTC interface for joint index %zu", i);
      return false;
    }
  }
  speed_rate_state_ = find_state("speed_rate/value");
  transfer_cmd_ = find_command("native_joint_trajectory/transfer_state");
  time_cmd_ = find_command("native_joint_trajectory/time_from_start");
  abort_cmd_ = find_command("native_joint_trajectory/abort");
  size_cmd_ = find_command("native_joint_trajectory/size");
  if (!transfer_cmd_ || !time_cmd_ || !abort_cmd_ || !size_cmd_) {
    RCLCPP_ERROR(get_node()->get_logger(), "missing native JTC handshake command interface");
    return false;
  }
  return true;
}

void NativeJointTrajectoryController::clear_interface_cache()
{
  pos_cmds_.fill(nullptr);
  vel_cmds_.fill(nullptr);
  acc_cmds_.fill(nullptr);
  pos_states_.fill(nullptr);
  vel_states_.fill(nullptr);
  speed_rate_state_ = nullptr;
  transfer_cmd_ = nullptr;
  time_cmd_ = nullptr;
  abort_cmd_ = nullptr;
  size_cmd_ = nullptr;
}

const hardware_interface::LoanedStateInterface *
NativeJointTrajectoryController::find_state(const std::string & name) const
{
  for (const auto & iface : state_interfaces_) {
    if (iface.get_name() == name) {
      return &iface;
    }
  }
  return nullptr;
}

hardware_interface::LoanedCommandInterface *
NativeJointTrajectoryController::find_command(const std::string & name)
{
  for (auto & iface : command_interfaces_) {
    if (iface.get_name() == name) {
      return &iface;
    }
  }
  return nullptr;
}

rclcpp_action::GoalResponse NativeJointTrajectoryController::handle_goal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const FollowJointTrajectory::Goal> goal)
{
  if (!active_.load()) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!goal || goal->trajectory.points.empty()) {
    RCLCPP_WARN(get_node()->get_logger(), "rejected empty FollowJointTrajectory goal");
    return rclcpp_action::GoalResponse::REJECT;
  }
  std::array<int, 6> map{};
  if (!map_joint_order(goal->trajectory.joint_names, map)) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (path_tolerance_ <= 0.0 && goal->trajectory.points.size() > max_points_) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "rejected FollowJointTrajectory: %zu points exceed max_points=%zu and "
      "path_tolerance is disabled",
      goal->trajectory.points.size(), max_points_);
    return rclcpp_action::GoalResponse::REJECT;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (phase_ != Phase::Idle || goal_handle_) {
    RCLCPP_WARN(get_node()->get_logger(), "rejected FollowJointTrajectory: already executing");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse NativeJointTrajectoryController::handle_cancel(
  const std::shared_ptr<GoalHandle> goal_handle)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (goal_handle_ && goal_handle == goal_handle_) {
    abort_requested_ = true;
    abort_reason_ = "canceled";
    deadline_active_ = false;  // re-arm handshake timeout in update(), not execution deadline
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  return rclcpp_action::CancelResponse::REJECT;
}

void NativeJointTrajectoryController::handle_accepted(
  const std::shared_ptr<GoalHandle> goal_handle)
{
  std::vector<TrajectoryPoint> points;
  if (!remap_goal(*goal_handle->get_goal(), points)) {
    auto result = std::make_shared<FollowJointTrajectory::Result>();
    result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
    result->error_string = "invalid trajectory";
    goal_handle->abort(result);
    return;
  }

  simplify(points);
  if (points.size() > max_points_) {
    auto result = std::make_shared<FollowJointTrajectory::Result>();
    result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
    result->error_string = "trajectory does not fit the hardware waypoint budget";
    goal_handle->abort(result);
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (phase_ != Phase::Idle || goal_handle_) {
    auto result = std::make_shared<FollowJointTrajectory::Result>();
    result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
    result->error_string = "already executing";
    goal_handle->abort(result);
    return;
  }
  points_ = std::move(points);
  send_index_ = 0;
  abort_requested_ = false;
  abort_reason_ = "canceled";
  deadline_active_ = false;
  goal_handle_ = goal_handle;
  phase_ = Phase::AnnounceSize;
}

bool NativeJointTrajectoryController::map_joint_order(
  const std::vector<std::string> & names, std::array<int, 6> & map) const
{
  map.fill(-1);
  for (size_t i = 0; i < joints_.size(); ++i) {
    const auto it = std::find(names.begin(), names.end(), joints_[i]);
    if (it == names.end()) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "trajectory missing joint '%s'", joints_[i].c_str());
      return false;
    }
    map[i] = static_cast<int>(std::distance(names.begin(), it));
  }
  return true;
}

bool NativeJointTrajectoryController::remap_goal(
  const FollowJointTrajectory::Goal & goal, std::vector<TrajectoryPoint> & points) const
{
  std::array<int, 6> map{};
  if (!map_joint_order(goal.trajectory.joint_names, map)) {
    return false;
  }

  points.clear();
  points.reserve(goal.trajectory.points.size());
  double previous_time = -1.0;
  for (const auto & pt : goal.trajectory.points) {
    TrajectoryPoint point;
    point.time_from_start = static_cast<double>(pt.time_from_start.sec) +
      static_cast<double>(pt.time_from_start.nanosec) * 1e-9;
    if (point.time_from_start < previous_time) {
      RCLCPP_ERROR(get_node()->get_logger(), "trajectory time_from_start decreases");
      return false;
    }
    previous_time = point.time_from_start;

    for (size_t i = 0; i < kNumJoints; ++i) {
      const auto source = static_cast<size_t>(map[i]);
      if (source >= pt.positions.size()) {
        RCLCPP_ERROR(get_node()->get_logger(), "trajectory point has too few positions");
        return false;
      }
      point.positions[i] = pt.positions[source];
      if (!std::isfinite(point.positions[i])) {
        RCLCPP_ERROR(get_node()->get_logger(), "trajectory point has NaN/Inf");
        return false;
      }
      if (source < pt.velocities.size() && std::isfinite(pt.velocities[source])) {
        point.velocities[i] = pt.velocities[source];
      }
      if (source < pt.accelerations.size() && std::isfinite(pt.accelerations[source])) {
        point.accelerations[i] = pt.accelerations[source];
      }
    }
    points.push_back(point);
  }
  return !points.empty();
}

namespace
{
double joint_distance_to_segment(
  const std::array<double, 6> & point,
  const std::array<double, 6> & from,
  const std::array<double, 6> & to)
{
  double segment_sq = 0.0;
  double projection = 0.0;
  for (size_t i = 0; i < kNumJoints; ++i) {
    const double d = to[i] - from[i];
    segment_sq += d * d;
    projection += (point[i] - from[i]) * d;
  }
  const double t = (segment_sq > 0.0) ? std::clamp(projection / segment_sq, 0.0, 1.0) : 0.0;
  double error_sq = 0.0;
  for (size_t i = 0; i < kNumJoints; ++i) {
    const double e = point[i] - (from[i] + t * (to[i] - from[i]));
    error_sq += e * e;
  }
  return std::sqrt(error_sq);
}

template<typename PointT>
void mark_kept(
  const std::vector<PointT> & points, double tolerance, std::vector<uint8_t> & keep)
{
  keep.assign(points.size(), 0);
  if (points.size() < 2) {
    keep.assign(points.size(), 1);
    return;
  }
  keep.front() = 1;
  keep.back() = 1;

  std::vector<std::pair<size_t, size_t>> stack{{0, points.size() - 1}};
  while (!stack.empty()) {
    const auto [first, last] = stack.back();
    stack.pop_back();
    if (last <= first + 1) {
      continue;
    }
    double worst = 0.0;
    size_t worst_index = first;
    for (size_t i = first + 1; i < last; ++i) {
      const double distance = joint_distance_to_segment(
        points[i].positions, points[first].positions, points[last].positions);
      if (distance > worst) {
        worst = distance;
        worst_index = i;
      }
    }
    if (worst > tolerance) {
      keep[worst_index] = 1;
      stack.emplace_back(first, worst_index);
      stack.emplace_back(worst_index, last);
    }
  }
}
}  // namespace

void NativeJointTrajectoryController::simplify(std::vector<TrajectoryPoint> & points) const
{
  if (path_tolerance_ <= 0.0 || points.size() <= 2) {
    return;
  }

  double tolerance = path_tolerance_;
  std::vector<uint8_t> keep;
  for (int attempt = 0; attempt < 24; ++attempt) {
    mark_kept(points, tolerance, keep);
    const size_t kept = static_cast<size_t>(std::count(keep.begin(), keep.end(), 1));
    if (kept <= max_points_) {
      if (kept == points.size()) {
        return;
      }
      std::vector<TrajectoryPoint> reduced;
      reduced.reserve(kept);
      for (size_t i = 0; i < points.size(); ++i) {
        if (keep[i]) {
          reduced.push_back(points[i]);
        }
      }
      if (tolerance > path_tolerance_) {
        RCLCPP_WARN(
          get_node()->get_logger(),
          "relaxed path_tolerance to %.4f rad to fit max_points=%zu",
          tolerance, max_points_);
      }
      points = std::move(reduced);
      return;
    }
    tolerance *= 2.0;
  }
}

void NativeJointTrajectoryController::finish_goal(
  bool success, int32_t error_code, const std::string & message)
{
  if (!goal_handle_) {
    phase_ = Phase::Idle;
    abort_requested_ = false;
    deadline_active_ = false;
    points_.clear();
    send_index_ = 0;
    return;
  }
  auto result = std::make_shared<FollowJointTrajectory::Result>();
  result->error_code = error_code;
  result->error_string = message;
  if (success) {
    goal_handle_->succeed(result);
  } else if (abort_requested_ && goal_handle_->is_canceling()) {
    goal_handle_->canceled(result);
  } else {
    goal_handle_->abort(result);
  }
  goal_handle_.reset();
  phase_ = Phase::Idle;
  abort_requested_ = false;
  deadline_active_ = false;
  points_.clear();
  send_index_ = 0;
  if (abort_cmd_) {
    set_command(*abort_cmd_, 0.0);
  }
}

void NativeJointTrajectoryController::set_deadline(const rclcpp::Time & now, double timeout)
{
  deadline_active_ = timeout > 0.0;
  if (deadline_active_) {
    deadline_ = now + rclcpp::Duration::from_seconds(timeout);
  }
}

bool NativeJointTrajectoryController::deadline_expired(const rclcpp::Time & now) const
{
  return deadline_active_ && now > deadline_;
}

void NativeJointTrajectoryController::publish_feedback()
{
  if (!goal_handle_) {
    return;
  }
  auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
  feedback->joint_names = joints_;
  feedback->actual.positions.resize(kNumJoints);
  feedback->actual.velocities.resize(kNumJoints);
  for (size_t i = 0; i < kNumJoints; ++i) {
    feedback->actual.positions[i] = get_double(*pos_states_[i]);
    feedback->actual.velocities[i] = get_double(*vel_states_[i]);
  }
  goal_handle_->publish_feedback(feedback);
}

//----- update -----
controller_interface::return_type NativeJointTrajectoryController::update(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  if (!transfer_cmd_ || !abort_cmd_) {
    return controller_interface::return_type::ERROR;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (phase_ == Phase::Idle) {
    return controller_interface::return_type::OK;
  }

  const int state = static_cast<int>(std::lround(get_double(*transfer_cmd_)));
  publish_feedback();

  if (abort_requested_) {
    set_command(*abort_cmd_, 1.0);
    if (!deadline_active_) {
      set_deadline(time, handshake_timeout_);
    }
    if (state == static_cast<int>(kPtDone)) {
      finish_goal(false, FollowJointTrajectory::Result::INVALID_GOAL, abort_reason_);
    } else if (deadline_expired(time)) {
      finish_goal(
        false, FollowJointTrajectory::Result::INVALID_GOAL,
        abort_reason_ + " (hardware did not acknowledge)");
    }
    return controller_interface::return_type::OK;
  }

  switch (phase_) {
    case Phase::WaitReady:
      if (state == static_cast<int>(kPtDone)) {
        finish_goal(false, FollowJointTrajectory::Result::INVALID_GOAL, "hw rejected size");
        return controller_interface::return_type::OK;
      }
      if (state == static_cast<int>(kPtReady)) {
        phase_ = Phase::SendPoint;
        deadline_active_ = false;
      }
      break;
    case Phase::WaitPointAck:
      if (state == static_cast<int>(kPtDone)) {
        finish_goal(false, FollowJointTrajectory::Result::INVALID_GOAL, "hw rejected point");
        return controller_interface::return_type::OK;
      }
      if (state == static_cast<int>(kPtReady)) {
        ++send_index_;
        phase_ = (send_index_ < points_.size()) ? Phase::SendPoint : Phase::SendGo;
        deadline_active_ = false;
      }
      break;
    case Phase::WaitExec:
    case Phase::WaitDone: {
      if (state == static_cast<int>(kPtExec)) {
        phase_ = Phase::WaitDone;
        break;
      }
      if (state == static_cast<int>(kPtDone)) {
        const bool ok = get_double(*abort_cmd_) <= 0.5;
        finish_goal(
          ok,
          ok ? FollowJointTrajectory::Result::SUCCESSFUL
             : FollowJointTrajectory::Result::INVALID_GOAL,
          ok ? "succeeded" : "moveCMP failed");
        return controller_interface::return_type::OK;
      }
      break;
    }
    default:
      break;
  }

  if (deadline_expired(time)) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "native JTC timed out waiting for the hardware; aborting");
    abort_requested_ = true;
    abort_reason_ = "hardware timeout";
    deadline_active_ = false;
    set_command(*abort_cmd_, 1.0);
    return controller_interface::return_type::OK;
  }

  switch (phase_) {
    case Phase::AnnounceSize:
      set_command(*abort_cmd_, 0.0);
      set_command(*size_cmd_, static_cast<double>(points_.size()));
      set_command(*transfer_cmd_, kPtSize);
      phase_ = Phase::WaitReady;
      set_deadline(time, handshake_timeout_);
      break;
    case Phase::SendPoint: {
      const auto & point = points_[send_index_];
      for (size_t i = 0; i < kNumJoints; ++i) {
        set_command(*pos_cmds_[i], point.positions[i]);
        set_command(*vel_cmds_[i], point.velocities[i]);
        set_command(*acc_cmds_[i], point.accelerations[i]);
      }
      set_command(*time_cmd_, point.time_from_start);
      set_command(*transfer_cmd_, kPtPoint);
      phase_ = Phase::WaitPointAck;
      set_deadline(time, handshake_timeout_);
      break;
    }
    case Phase::SendGo: {
      set_command(*transfer_cmd_, kPtGo);
      phase_ = Phase::WaitExec;
      double timeout = 0.0;
      if (execution_timeout_margin_ > 0.0) {
        const double scaling = speed_rate_state_
          ? std::clamp(get_double(*speed_rate_state_), 0.05, 1.0)
          : 1.0;
        timeout = points_.back().time_from_start / scaling + execution_timeout_margin_;
      }
      set_deadline(time, timeout);
      break;
    }
    default:
      break;
  }

  return controller_interface::return_type::OK;
}

}  // namespace symphony_controllers

PLUGINLIB_EXPORT_CLASS(
  symphony_controllers::NativeJointTrajectoryController,
  controller_interface::ControllerInterface)
