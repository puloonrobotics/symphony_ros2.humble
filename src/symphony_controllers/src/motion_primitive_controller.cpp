#include "symphony_controllers/motion_primitive_controller.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/create_server.hpp"
#include "symphony_msgs/msg/motion_primitive.hpp"

namespace symphony_controllers
{

namespace
{
constexpr size_t kNumJoints = 6;
constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

constexpr double kTypePtp = 0.0;
constexpr double kTypeLin = 50.0;
constexpr double kTypeArc = 51.0;
constexpr double kTypeStop = 66.0;
constexpr double kTypeReset = 67.0;

constexpr int kExecIdle = 0;
constexpr int kExecExecuting = 1;
constexpr int kExecSuccess = 2;
constexpr int kExecError = 3;
constexpr int kExecStopping = 4;
constexpr int kExecStopped = 5;

constexpr const char * kTypeName = "motion_primitive/motion_type";
constexpr const char * kQNames[] = {
  "motion_primitive/q1", "motion_primitive/q2", "motion_primitive/q3",
  "motion_primitive/q4", "motion_primitive/q5", "motion_primitive/q6",
};
constexpr const char * kPoseNames[] = {
  "motion_primitive/pos_x", "motion_primitive/pos_y", "motion_primitive/pos_z",
  "motion_primitive/pos_qx", "motion_primitive/pos_qy", "motion_primitive/pos_qz",
  "motion_primitive/pos_qw",
};
constexpr const char * kViaNames[] = {
  "motion_primitive/pos_via_x", "motion_primitive/pos_via_y", "motion_primitive/pos_via_z",
  "motion_primitive/pos_via_qx", "motion_primitive/pos_via_qy", "motion_primitive/pos_via_qz",
  "motion_primitive/pos_via_qw",
};
constexpr const char * kBlend = "motion_primitive/blend_radius";
constexpr const char * kVelocity = "motion_primitive/velocity";
constexpr const char * kAcceleration = "motion_primitive/acceleration";
constexpr const char * kMoveTime = "motion_primitive/move_time";
constexpr const char * kPoseCfg = "motion_primitive/pose_cfg";
constexpr const char * kPause = "motion_primitive/pause";
constexpr const char * kResume = "motion_primitive/resume";
constexpr const char * kStatus = "motion_primitive/execution_status";
constexpr const char * kReady = "motion_primitive/ready_for_new_primitive";

int exec_status(const hardware_interface::LoanedStateInterface * state)
{
  if (!state) {
    return kExecError;
  }
  return static_cast<int>(std::lround(state->get_value()));
}

bool is_ready(const hardware_interface::LoanedStateInterface * state)
{
  return state && state->get_value() > 0.5;
}

void write_pose(
  const std::array<hardware_interface::LoanedCommandInterface *, 7> & cmds,
  const geometry_msgs::msg::Pose & pose)
{
  cmds[0]->set_value(pose.position.x);
  cmds[1]->set_value(pose.position.y);
  cmds[2]->set_value(pose.position.z);
  cmds[3]->set_value(pose.orientation.x);
  cmds[4]->set_value(pose.orientation.y);
  cmds[5]->set_value(pose.orientation.z);
  cmds[6]->set_value(pose.orientation.w);
}

void write_nan_pose(const std::array<hardware_interface::LoanedCommandInterface *, 7> & cmds)
{
  for (auto * cmd : cmds) {
    cmd->set_value(kNan);
  }
}

bool finite_pose(const geometry_msgs::msg::Pose & pose)
{
  if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
      !std::isfinite(pose.position.z) || !std::isfinite(pose.orientation.x) ||
      !std::isfinite(pose.orientation.y) || !std::isfinite(pose.orientation.z) ||
      !std::isfinite(pose.orientation.w))
  {
    return false;
  }
  const double n =
    pose.orientation.x * pose.orientation.x + pose.orientation.y * pose.orientation.y +
    pose.orientation.z * pose.orientation.z + pose.orientation.w * pose.orientation.w;
  return n > 1.0e-6;
}

bool find_argument(
  const symphony_msgs::msg::MotionPrimitive & primitive, const std::string & name, double & value)
{
  for (const auto & arg : primitive.additional_arguments) {
    if (arg.name == name) {
      value = arg.value;
      return true;
    }
  }
  return false;
}
}  // namespace

controller_interface::InterfaceConfiguration
MotionPrimitiveController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.push_back(kTypeName);
  for (const char * name : kQNames) {
    config.names.push_back(name);
  }
  for (const char * name : kPoseNames) {
    config.names.push_back(name);
  }
  for (const char * name : kViaNames) {
    config.names.push_back(name);
  }
  config.names.push_back(kBlend);
  config.names.push_back(kVelocity);
  config.names.push_back(kAcceleration);
  config.names.push_back(kMoveTime);
  config.names.push_back(kPoseCfg);
  config.names.push_back(kPause);
  config.names.push_back(kResume);
  return config;
}

controller_interface::InterfaceConfiguration
MotionPrimitiveController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = {kStatus, kReady};
  return config;
}

controller_interface::CallbackReturn MotionPrimitiveController::on_init()
{
  try {
    auto_declare<double>("handshake_timeout", 2.0);
    auto_declare<double>("execution_timeout", 60.0);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MotionPrimitiveController::on_configure(
  const rclcpp_lifecycle::State &)
{
  handshake_timeout_ = get_node()->get_parameter("handshake_timeout").as_double();
  execution_timeout_ = get_node()->get_parameter("execution_timeout").as_double();
  if (handshake_timeout_ <= 0.0 || execution_timeout_ <= 0.0) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "handshake_timeout and execution_timeout must be > 0");
    return controller_interface::CallbackReturn::ERROR;
  }

  using namespace std::placeholders;
  const auto action_name = std::string(get_node()->get_name()) + "/motion_primitive";
  action_server_ = rclcpp_action::create_server<ExecuteMotion>(
    get_node()->get_node_base_interface(),
    get_node()->get_node_clock_interface(),
    get_node()->get_node_logging_interface(),
    get_node()->get_node_waitables_interface(),
    action_name,
    std::bind(&MotionPrimitiveController::handle_goal, this, _1, _2),
    std::bind(&MotionPrimitiveController::handle_cancel, this, _1),
    std::bind(&MotionPrimitiveController::handle_accepted, this, _1));

  move_pause_srv_ = get_node()->create_service<std_srvs::srv::Trigger>(
    "~/move_pause",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      if (!active_.load()) {
        response->success = false;
        response->message = "motion_primitive_controller is not active";
        return;
      }
      pause_pulse_.store(true, std::memory_order_release);
      response->success = true;
      response->message = "accepted";
    });

  move_resume_srv_ = get_node()->create_service<std_srvs::srv::Trigger>(
    "~/move_resume",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      if (!active_.load()) {
        response->success = false;
        response->message = "motion_primitive_controller is not active";
        return;
      }
      resume_pulse_.store(true, std::memory_order_release);
      response->success = true;
      response->message = "accepted";
    });

  RCLCPP_INFO(
    get_node()->get_logger(),
    "MotionPrimitiveController configured (~/motion_primitive). "
    "Cancel stops the motion. Do not activate next to joint_trajectory_controller.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MotionPrimitiveController::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!cache_interfaces()) {
    clear_interface_cache();
    return controller_interface::CallbackReturn::ERROR;
  }
  write_nan_commands();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    clear_motion_locked();
  }
  pause_pulse_.store(false);
  resume_pulse_.store(false);
  active_.store(true);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MotionPrimitiveController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  active_.store(false);
  if (type_cmd_) {
    type_cmd_->set_value(kTypeStop);
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    abort_locked(
      ExecuteMotion::Result::INVALID_GOAL, "motion_primitive_controller deactivated");
  }
  clear_interface_cache();
  return controller_interface::CallbackReturn::SUCCESS;
}

bool MotionPrimitiveController::cache_interfaces()
{
  type_cmd_ = find_command(kTypeName);
  for (size_t i = 0; i < kNumJoints; ++i) {
    q_cmds_[i] = find_command(kQNames[i]);
  }
  for (size_t i = 0; i < 7; ++i) {
    pose_cmds_[i] = find_command(kPoseNames[i]);
    via_cmds_[i] = find_command(kViaNames[i]);
  }
  blend_cmd_ = find_command(kBlend);
  velocity_cmd_ = find_command(kVelocity);
  acceleration_cmd_ = find_command(kAcceleration);
  move_time_cmd_ = find_command(kMoveTime);
  pose_cfg_cmd_ = find_command(kPoseCfg);
  pause_cmd_ = find_command(kPause);
  resume_cmd_ = find_command(kResume);
  status_state_ = find_state(kStatus);
  ready_state_ = find_state(kReady);
  if (!type_cmd_ || !blend_cmd_ || !velocity_cmd_ || !acceleration_cmd_ || !move_time_cmd_ ||
      !pose_cfg_cmd_ || !pause_cmd_ || !resume_cmd_ || !status_state_ || !ready_state_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "missing motion_primitive interface");
    return false;
  }
  for (size_t i = 0; i < kNumJoints; ++i) {
    if (!q_cmds_[i]) {
      RCLCPP_ERROR(get_node()->get_logger(), "missing %s", kQNames[i]);
      return false;
    }
  }
  for (size_t i = 0; i < 7; ++i) {
    if (!pose_cmds_[i] || !via_cmds_[i]) {
      RCLCPP_ERROR(get_node()->get_logger(), "missing pose/via command interface");
      return false;
    }
  }
  return true;
}

void MotionPrimitiveController::clear_interface_cache()
{
  type_cmd_ = nullptr;
  q_cmds_.fill(nullptr);
  pose_cmds_.fill(nullptr);
  via_cmds_.fill(nullptr);
  blend_cmd_ = nullptr;
  velocity_cmd_ = nullptr;
  acceleration_cmd_ = nullptr;
  move_time_cmd_ = nullptr;
  pose_cfg_cmd_ = nullptr;
  pause_cmd_ = nullptr;
  resume_cmd_ = nullptr;
  status_state_ = nullptr;
  ready_state_ = nullptr;
}

hardware_interface::LoanedCommandInterface *
MotionPrimitiveController::find_command(const std::string & name)
{
  for (auto & iface : command_interfaces_) {
    if (iface.get_name() == name) {
      return &iface;
    }
  }
  return nullptr;
}

const hardware_interface::LoanedStateInterface *
MotionPrimitiveController::find_state(const std::string & name) const
{
  for (const auto & iface : state_interfaces_) {
    if (iface.get_name() == name) {
      return &iface;
    }
  }
  return nullptr;
}

void MotionPrimitiveController::write_nan_commands()
{
  if (!type_cmd_) {
    return;
  }
  type_cmd_->set_value(kNan);
  for (auto * cmd : q_cmds_) {
    cmd->set_value(kNan);
  }
  write_nan_pose(pose_cmds_);
  write_nan_pose(via_cmds_);
  blend_cmd_->set_value(kNan);
  velocity_cmd_->set_value(kNan);
  acceleration_cmd_->set_value(kNan);
  move_time_cmd_->set_value(kNan);
  pose_cfg_cmd_->set_value(kNan);
}

void MotionPrimitiveController::write_job(const Job & job)
{
  write_nan_commands();
  velocity_cmd_->set_value(job.velocity);
  blend_cmd_->set_value(job.blend_radius);
  if (job.has_acceleration) {
    acceleration_cmd_->set_value(job.acceleration);
  }
  move_time_cmd_->set_value(job.fixed_speed ? 1.0 : 0.0);
  switch (job.kind) {
    case Kind::Ptp:
      for (size_t i = 0; i < kNumJoints; ++i) {
        q_cmds_[i]->set_value(job.joints[i]);
      }
      type_cmd_->set_value(kTypePtp);
      break;
    case Kind::Lin:
      write_pose(pose_cmds_, job.pose);
      if (job.pose_cfg != 0) {
        pose_cfg_cmd_->set_value(static_cast<double>(job.pose_cfg));
      }
      type_cmd_->set_value(kTypeLin);
      break;
    case Kind::Arc:
      write_pose(via_cmds_, job.via);
      write_pose(pose_cmds_, job.pose);
      if (job.pose_cfg != 0) {
        pose_cfg_cmd_->set_value(static_cast<double>(job.pose_cfg));
      }
      type_cmd_->set_value(kTypeArc);
      break;
    case Kind::Stop:
      type_cmd_->set_value(kTypeStop);
      break;
    case Kind::None:
      break;
  }
}

bool MotionPrimitiveController::decode_goal(
  const ExecuteMotion::Goal & goal, std::vector<Job> & jobs, std::string & error) const
{
  using Primitive = symphony_msgs::msg::MotionPrimitive;
  const auto & motions = goal.trajectory.motions;
  if (motions.empty()) {
    error = "trajectory.motions is empty";
    return false;
  }
  if (motions.size() > 255) {
    error = "trajectory.motions exceeds feedback index range";
    return false;
  }
  jobs.clear();
  jobs.reserve(motions.size());
  for (size_t i = 0; i < motions.size(); ++i) {
    const auto & primitive = motions[i];
    Job job;
    job.blend_radius = primitive.blend_radius;
    if (!std::isfinite(job.blend_radius)) {
      error = "primitive " + std::to_string(i) + " blend_radius is not finite";
      return false;
    }
    double velocity = 0.0;
    if (find_argument(primitive, "velocity", velocity)) {
      if (!std::isfinite(velocity)) {
        error = "primitive " + std::to_string(i) + " velocity is not finite";
        return false;
      }
      job.velocity = velocity;
    }
    double acceleration = 0.0;
    if (find_argument(primitive, "acceleration", acceleration)) {
      if (!std::isfinite(acceleration)) {
        error = "primitive " + std::to_string(i) + " acceleration is not finite";
        return false;
      }
      job.acceleration = acceleration;
      job.has_acceleration = true;
    }
    double pose_cfg = 0.0;
    if (find_argument(primitive, "pose_cfg", pose_cfg)) {
      if (!std::isfinite(pose_cfg)) {
        error = "primitive " + std::to_string(i) + " pose_cfg is not finite";
        return false;
      }
      job.pose_cfg = static_cast<uint32_t>(std::lround(std::max(0.0, pose_cfg)));
    }
    double fixed_speed = 0.0;
    if (find_argument(primitive, "fixed_speed", fixed_speed) ||
        find_argument(primitive, "fixedspeed", fixed_speed))
    {
      job.fixed_speed = fixed_speed > 0.5;
    }
    for (const auto & arg : primitive.additional_arguments) {
      if (arg.name != "velocity" && arg.name != "acceleration" && arg.name != "pose_cfg" &&
          arg.name != "fixed_speed" && arg.name != "fixedspeed")
      {
        RCLCPP_WARN(
          get_node()->get_logger(),
          "primitive %zu ignores additional argument '%s'", i, arg.name.c_str());
      }
    }

    if (primitive.type == Primitive::LINEAR_JOINT) {
      if (primitive.joint_positions.size() != kNumJoints) {
        error = "primitive " + std::to_string(i) + " LINEAR_JOINT needs 6 joint_positions";
        return false;
      }
      for (size_t j = 0; j < kNumJoints; ++j) {
        if (!std::isfinite(primitive.joint_positions[j])) {
          error = "primitive " + std::to_string(i) + " joint_positions must be finite";
          return false;
        }
        job.joints[j] = primitive.joint_positions[j];
      }
      job.kind = Kind::Ptp;
    } else if (primitive.type == Primitive::LINEAR_CARTESIAN) {
      if (primitive.poses.size() != 1 || !finite_pose(primitive.poses[0].pose)) {
        error = "primitive " + std::to_string(i) +
          " LINEAR_CARTESIAN needs one finite pose";
        return false;
      }
      job.pose = primitive.poses[0].pose;
      job.kind = Kind::Lin;
    } else if (primitive.type == Primitive::CIRCULAR_CARTESIAN) {
      if (primitive.poses.size() != 2 || !finite_pose(primitive.poses[0].pose) ||
          !finite_pose(primitive.poses[1].pose))
      {
        error = "primitive " + std::to_string(i) +
          " CIRCULAR_CARTESIAN needs goal pose then via pose";
        return false;
      }
      job.pose = primitive.poses[0].pose;
      job.via = primitive.poses[1].pose;
      job.kind = Kind::Arc;
    } else {
      error = "primitive " + std::to_string(i) + " has unsupported type";
      return false;
    }
    jobs.push_back(job);
  }
  return true;
}

rclcpp_action::GoalResponse MotionPrimitiveController::handle_goal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const ExecuteMotion::Goal> goal)
{
  if (!active_.load() || !goal) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  std::vector<Job> jobs;
  std::string error;
  if (!decode_goal(*goal, jobs, error)) {
    RCLCPP_WARN(get_node()->get_logger(), "rejected motion_primitive: %s", error.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (goal_handle_ || phase_ != Phase::Idle) {
    RCLCPP_WARN(get_node()->get_logger(), "rejected motion_primitive: already executing");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MotionPrimitiveController::handle_cancel(
  const std::shared_ptr<GoalHandle> goal_handle)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (goal_handle_ && goal_handle == goal_handle_) {
    cancel_requested_ = true;
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  return rclcpp_action::CancelResponse::REJECT;
}

void MotionPrimitiveController::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
  std::vector<Job> jobs;
  std::string error;
  if (!decode_goal(*goal_handle->get_goal(), jobs, error)) {
    auto result = std::make_shared<ExecuteMotion::Result>();
    result->error_code = ExecuteMotion::Result::INVALID_GOAL;
    result->error_string = error;
    goal_handle->abort(result);
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (goal_handle_ || phase_ != Phase::Idle) {
    auto result = std::make_shared<ExecuteMotion::Result>();
    result->error_code = ExecuteMotion::Result::INVALID_GOAL;
    result->error_string = "motion_primitive_controller is busy";
    goal_handle->abort(result);
    return;
  }
  goal_handle_ = goal_handle;
  jobs_ = std::move(jobs);
  job_index_ = 0;
  phase_ = Phase::Idle;
  cancel_requested_ = false;
  stopping_ = false;
  motion_started_ = false;
  deadline_active_ = false;
}

void MotionPrimitiveController::begin_job_locked(const rclcpp::Time & time, const Job & job)
{
  motion_started_ = false;
  write_job(job);
  phase_ = Phase::WaitStart;
  deadline_ = time + rclcpp::Duration::from_seconds(handshake_timeout_);
  deadline_active_ = true;
  if (!stopping_) {
    publish_feedback_locked();
  }
}

void MotionPrimitiveController::publish_feedback_locked()
{
  if (!goal_handle_) {
    return;
  }
  auto feedback = std::make_shared<ExecuteMotion::Feedback>();
  feedback->current_primitive_index = static_cast<uint8_t>(job_index_);
  goal_handle_->publish_feedback(feedback);
}

void MotionPrimitiveController::clear_motion_locked()
{
  goal_handle_.reset();
  jobs_.clear();
  job_index_ = 0;
  phase_ = Phase::Idle;
  cancel_requested_ = false;
  stopping_ = false;
  motion_started_ = false;
  deadline_active_ = false;
}

void MotionPrimitiveController::succeed_locked(const std::string & message)
{
  if (!goal_handle_) {
    clear_motion_locked();
    return;
  }
  auto result = std::make_shared<ExecuteMotion::Result>();
  result->error_code = ExecuteMotion::Result::SUCCESSFUL;
  result->error_string = message;
  if (goal_handle_->is_canceling()) {
    goal_handle_->canceled(result);
  } else {
    goal_handle_->succeed(result);
  }
  clear_motion_locked();
}

void MotionPrimitiveController::abort_locked(int32_t error_code, const std::string & message)
{
  if (!goal_handle_) {
    clear_motion_locked();
    return;
  }
  auto result = std::make_shared<ExecuteMotion::Result>();
  result->error_code = error_code;
  result->error_string = message;
  goal_handle_->abort(result);
  clear_motion_locked();
}

void MotionPrimitiveController::cancel_locked(const std::string & message)
{
  if (!goal_handle_) {
    clear_motion_locked();
    return;
  }
  auto result = std::make_shared<ExecuteMotion::Result>();
  result->error_code = ExecuteMotion::Result::SUCCESSFUL;
  result->error_string = message;
  goal_handle_->canceled(result);
  clear_motion_locked();
}

controller_interface::return_type MotionPrimitiveController::update(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  if (!type_cmd_ || !status_state_ || !ready_state_) {
    return controller_interface::return_type::ERROR;
  }

  const bool pause = pause_pulse_.exchange(false, std::memory_order_acq_rel);
  const bool resume = resume_pulse_.exchange(false, std::memory_order_acq_rel);
  pause_cmd_->set_value(pause ? 1.0 : 0.0);
  resume_cmd_->set_value(resume ? 1.0 : 0.0);

  std::lock_guard<std::mutex> lock(mutex_);
  const int status = exec_status(status_state_);
  const bool ready = is_ready(ready_state_);
  if (status == kExecExecuting || status == kExecStopping) {
    motion_started_ = true;
  }

  if (goal_handle_ && (cancel_requested_ || goal_handle_->is_canceling()) && !stopping_) {
    if (phase_ == Phase::Idle) {
      cancel_locked("canceled");
      write_nan_commands();
      return controller_interface::return_type::OK;
    }
    Job stop;
    stop.kind = Kind::Stop;
    stopping_ = true;
    cancel_requested_ = false;
    begin_job_locked(time, stop);
    return controller_interface::return_type::OK;
  }

  if (phase_ == Phase::Idle) {
    if (goal_handle_ && job_index_ < jobs_.size()) {
      begin_job_locked(time, jobs_[job_index_]);
    } else {
      write_nan_commands();
    }
    return controller_interface::return_type::OK;
  }

  write_nan_commands();

  if (deadline_active_ && time > deadline_) {
    const char * why =
      (phase_ == Phase::WaitStart) ? "HW did not start the motion" : "motion timed out";
    abort_locked(ExecuteMotion::Result::INVALID_GOAL, why);
    return controller_interface::return_type::OK;
  }

  switch (phase_) {
    case Phase::WaitStart:
      if (status == kExecError) {
        abort_locked(ExecuteMotion::Result::INVALID_GOAL, "HW reported error");
        break;
      }
      if (motion_started_ || (stopping_ && status == kExecStopped)) {
        phase_ = Phase::WaitDone;
        deadline_ = time + rclcpp::Duration::from_seconds(execution_timeout_);
        deadline_active_ = true;
      }
      break;
    case Phase::WaitDone:
      if (status == kExecError) {
        abort_locked(ExecuteMotion::Result::INVALID_GOAL, "HW reported error");
        break;
      }
      if (stopping_) {
        if (status == kExecStopped || status == kExecIdle) {
          type_cmd_->set_value(kTypeReset);
          phase_ = Phase::WaitReset;
          deadline_ = time + rclcpp::Duration::from_seconds(handshake_timeout_);
          deadline_active_ = true;
        }
        break;
      }
      if (!motion_started_) {
        break;
      }
      if (status == kExecStopped) {
        cancel_locked("stopped");
        break;
      }
      if (status == kExecSuccess || (ready && status == kExecIdle)) {
        ++job_index_;
        if (job_index_ < jobs_.size()) {
          begin_job_locked(time, jobs_[job_index_]);
        } else {
          succeed_locked("done");
        }
      }
      break;
    case Phase::WaitReset:
      if (status == kExecIdle || ready) {
        cancel_locked("stopped");
      }
      break;
    case Phase::Idle:
      break;
  }
  return controller_interface::return_type::OK;
}

}  // namespace symphony_controllers

PLUGINLIB_EXPORT_CLASS(
  symphony_controllers::MotionPrimitiveController, controller_interface::ControllerInterface)
