#include "hw_internal.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <exception>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_set>
#include <utility>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "puloon_rtpri/armDefine.h"
#include "puloon_rtpri/rtpriUdpStatus.h"
#include "rclcpp/rclcpp.hpp"
#include "symphony_msgs/msg/safety_status.hpp"

namespace symphony_driver
{

//----- ctor / units / connection / status -----

SymphonyHardwareInterface::SymphonyHardwareInterface() = default;

SymphonyHardwareInterface::~SymphonyHardwareInterface()
{
  stop_async_worker();
  disconnect_robot();
}

double SymphonyHardwareInterface::robot_deg_to_ros_rad(double robot_deg)
{
  // [deg] -> [rad]
  return robot_deg * kDegToRad;
}

double SymphonyHardwareInterface::ros_rad_to_robot_deg(double ros_rad)
{
  // [rad] -> [deg]
  return ros_rad * kRadToDeg;
}

void SymphonyHardwareInterface::rpy_deg_to_quat(
  double roll_deg, double pitch_deg, double yaw_deg, double q[4])
{
  const double roll = robot_deg_to_ros_rad(roll_deg);
  const double pitch = robot_deg_to_ros_rad(pitch_deg);
  const double yaw = robot_deg_to_ros_rad(yaw_deg);
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  q[0] = sr * cp * cy - cr * sp * sy;
  q[1] = cr * sp * cy + sr * cp * sy;
  q[2] = cr * cp * sy - sr * sp * cy;
  q[3] = cr * cp * cy + sr * sp * sy;
}

void SymphonyHardwareInterface::quat_to_rpy_deg(
  double qx, double qy, double qz, double qw,
  double & roll_deg, double & pitch_deg, double & yaw_deg)
{
  const double sinr_cosp = 2.0 * (qw * qx + qy * qz);
  const double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
  const double roll = std::atan2(sinr_cosp, cosr_cosp);

  const double sinp = 2.0 * (qw * qy - qz * qx);
  const double pitch = (std::abs(sinp) >= 1.0)
    ? std::copysign(M_PI / 2.0, sinp)
    : std::asin(sinp);

  const double siny_cosp = 2.0 * (qw * qz + qx * qy);
  const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  const double yaw = std::atan2(siny_cosp, cosy_cosp);

  roll_deg = ros_rad_to_robot_deg(roll);
  pitch_deg = ros_rad_to_robot_deg(pitch);
  yaw_deg = ros_rad_to_robot_deg(yaw);
}

bool SymphonyHardwareInterface::connect_robot()
{
  if (!control_ || !command_ || !status_) {
    return false;
  }

  if (!control_->connect()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("SymphonyHardwareInterface"),
      "RTPRIControl connect failed (%s:4500)", robot_ip_.c_str());
    return false;
  }
  if (!status_->connect()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("SymphonyHardwareInterface"),
      "RTPRIStatus connect failed (%s:4300)", robot_ip_.c_str());
    control_->disconnect();
    return false;
  }
  if (!command_->connect()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("SymphonyHardwareInterface"),
      "RTPRICommand connect failed (%s:4200)", robot_ip_.c_str());
    status_->disconnect();
    control_->disconnect();
    return false;
  }

  control_->stopRtSpeedControl();
  control_->stopRtServoControl();
  constexpr uint8_t kMotionIdle = 0;
  for (int i = 0; i < 50; ++i) {
    if (status_ && status_->getMotionState() == kMotionIdle) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (status_) {
    const auto actual_joint_angles_deg = status_->getActualJointAngles();
    if (actual_joint_angles_deg.size() >= kNumJoints) {
      for (size_t i = 0; i < kNumJoints; ++i) {
        joint_positions_[i] = robot_deg_to_ros_rad(actual_joint_angles_deg[i]);
      }
      joint_position_commands_ = joint_positions_;
      joint_velocity_commands_.assign(kNumJoints, 0.0);
    }
  }

  connected_ = true;
  rt_servo_active_ = false;
  rt_speed_active_ = false;
  rt_servox_active_ = false;
  return true;
}

void SymphonyHardwareInterface::disconnect_robot()
{
  status_link_ok_ = false;
  stop_async_worker();
  stop_rt_streams();

  if (command_) {
    command_->disconnect();
  }
  if (status_) {
    status_->disconnect();
  }
  if (control_) {
    control_->disconnect();
  }

  connected_ = false;
}

bool SymphonyHardwareInterface::robot_is_safe_to_command() const
{
  if (!status_) {
    return false;
  }

  const uint32_t state = status_->getArmState();
  const uint32_t unsafe =
    PRM_STATE_FAULT | PRM_STATE_STO | PRM_STATE_SS1 | PRM_STATE_SS2 |
    PRM_STATE_EMERGENCY | PRM_STATE_COLLISION;
  return (state & unsafe) == 0;
}

bool SymphonyHardwareInterface::status_sample_plausible(
  unsigned int dof, uint8_t op_mode, uint8_t motion_state, double speed,
  const std::vector<double> & joint_angles_deg) const
{
  if (dof != kNumJoints) {
    return false;
  }
  if (op_mode > 3) {
    return false;
  }
  if (motion_state > 12) {
    return false;
  }
  if (!std::isfinite(speed) || speed < 0.0 || speed > 1.05) {
    return false;
  }
  if (joint_angles_deg.size() < kNumJoints) {
    return false;
  }
  constexpr double kMaxAbsJointDeg = 720.0;
  for (size_t i = 0; i < kNumJoints; ++i) {
    if (!std::isfinite(joint_angles_deg[i]) ||
      std::abs(joint_angles_deg[i]) > kMaxAbsJointDeg)
    {
      return false;
    }
  }
  return true;
}

bool SymphonyHardwareInterface::sync_state_from_robot()
{
  if (!status_) {
    status_link_ok_ = false;
    return false;
  }

  const unsigned int dof = status_->getDof();
  const uint8_t op_mode = status_->getOpMode();
  const uint8_t motion_state = status_->getMotionState();
  const double speed = status_->getSpeed();
  const auto actual_joint_angles_deg = status_->getActualJointAngles();
  if (!status_sample_plausible(
      dof, op_mode, motion_state, speed, actual_joint_angles_deg))
  {
    if (status_link_ok_.exchange(false)) {
      RCLCPP_WARN(
        rclcpp::get_logger("SymphonyHardwareInterface"),
        "RTPRI status sample rejected (dof=%u opmode=%u motion=%u speed=%.3f); "
        "holding last state",
        dof, static_cast<unsigned>(op_mode),
        static_cast<unsigned>(motion_state), speed);
    }
    return false;
  }

  const auto actual_joint_velocities_deg_s = status_->getActualJointVelocities();
  const auto joint_torques = status_->getJointTorques();

  for (size_t i = 0; i < kNumJoints; ++i) {
    joint_positions_[i] = robot_deg_to_ros_rad(actual_joint_angles_deg[i]);
    joint_velocities_[i] = (actual_joint_velocities_deg_s.size() > i)
      ? robot_deg_to_ros_rad(actual_joint_velocities_deg_s[i])
      : 0.0;
    joint_efforts_[i] = (joint_torques.size() > i) ? joint_torques[i] : 0.0;
  }

  const uint32_t arm_state = status_->getArmState();

  speed_rate_ = speed;
  op_mode_ = static_cast<double>(op_mode);
  safety_status_ = derive_safety_status(arm_state);
  safety_zone_ = static_cast<double>(status_->getUserSafeState());
  arm_state_ = static_cast<double>(arm_state);
  tool_io_status_ = static_cast<double>(status_->getToolIoStatus());
  tm_command_ = static_cast<double>(status_->getToolIoModbusCommand());
  tm_echo_address_ = static_cast<double>(status_->getToolIoModbusAddress());
  tm_number_ = static_cast<double>(status_->getToolIoModbusNumber());
  servo_enable_ = status_->getServoEnable(0) ? 1.0 : 0.0;
  force_enabled_ = (arm_state & PRM_STATE_FORCECTL) ? 1.0 : 0.0;
  update_arm_errors();
  sync_io_state_from_robot();
  sync_tcp_sensors_from_robot();

  status_link_ok_.store(true);
  return true;
}

double SymphonyHardwareInterface::derive_safety_status(uint32_t arm_state)
{
  using symphony_msgs::msg::SafetyStatus;
  if (arm_state & PRM_STATE_FAULT) {
    return SafetyStatus::FAULT;
  }
  if (arm_state & PRM_STATE_EMERGENCY) {
    return SafetyStatus::EMERGENCY_STOP;
  }
  if (arm_state & PRM_STATE_STO) {
    return SafetyStatus::SAFE_TORQUE_OFF;
  }
  if (arm_state & PRM_STATE_SS1) {
    return SafetyStatus::SAFE_STOP_1;
  }
  if (arm_state & PRM_STATE_SS2) {
    return SafetyStatus::SAFE_STOP_2;
  }
  if (arm_state & PRM_STATE_COLLISION) {
    return SafetyStatus::COLLISION;
  }
  if (arm_state & PRM_STATE_RECOVERY) {
    return SafetyStatus::RECOVERY;
  }
  return SafetyStatus::NORMAL;
}

void SymphonyHardwareInterface::update_arm_errors()
{
  std::vector<uint32_t> codes = status_->getArmErrorList();
  codes.erase(std::find(codes.begin(), codes.end(), 0u), codes.end());

  for (size_t i = 0; i < kArmErrorSlots; ++i) {
    arm_errors_[i] = (i < codes.size()) ? static_cast<double>(codes[i]) : 0.0;
  }

  size_t first_new = 0;
  for (size_t shift = 0; shift <= last_arm_errors_.size(); ++shift) {
    const size_t overlap = last_arm_errors_.size() - shift;
    if (overlap > codes.size()) {
      continue;
    }
    const auto overlap_begin =
      last_arm_errors_.begin() + static_cast<std::ptrdiff_t>(shift);
    if (std::equal(overlap_begin, last_arm_errors_.end(), codes.begin())) {
      first_new = overlap;
      break;
    }
  }

  for (size_t i = first_new; i < codes.size(); ++i) {
    const auto code = static_cast<unsigned int>(codes[i]);
    if (code & 0x80000000u) {
      RCLCPP_ERROR(
        rclcpp::get_logger("SymphonyHardwareInterface"),
        "Robot reported error 0x%08X", code);
    } else {
      RCLCPP_WARN(
        rclcpp::get_logger("SymphonyHardwareInterface"),
        "Robot reported warning 0x%08X", code);
    }
  }

  last_arm_errors_ = std::move(codes);
}

//----- cyclic read / write / gpio / async -----

hardware_interface::return_type SymphonyHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!activated_) {
    return hardware_interface::return_type::OK;
  }

  if (connected_ && status_) {
    if (sync_state_from_robot()) {
      update_moprim_state();
      update_native_jt_state();
    }
    return hardware_interface::return_type::OK;
  }

  for (size_t i = 0; i < joint_positions_.size(); ++i) {
    joint_positions_[i] = joint_position_commands_[i];
    joint_velocities_[i] = 0.0;
    joint_efforts_[i] = 0.0;
  }
  speed_rate_ = initial_speed_;
  op_mode_ = 0.0;
  safety_status_ = 0.0;
  safety_zone_ = 0.0;
  arm_state_ = 0.0;
  tool_io_status_ = 0.0;
  servo_enable_ = 0.0;
  force_enabled_ = force_mode_active_ ? 1.0 : 0.0;
  for (double & code : arm_errors_) {
    code = 0.0;
  }
  last_arm_errors_.clear();

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SymphonyHardwareInterface::write(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  if (!activated_) {
    return hardware_interface::return_type::OK;
  }

  apply_gpio_commands();
  apply_io_commands();
  apply_force_commands();
  apply_tool_modbus_commands();
  handle_moprim_commands();
  handle_native_jt_commands();

  update_command_watchdog(time);

  const bool streaming = rt_servo_active_ || rt_speed_active_ || rt_servox_active_;
  if (connected_ && streaming && command_) {
    if (!robot_is_safe_to_command()) {
      request_stop_rt_streams();
      RCLCPP_ERROR(
        rclcpp::get_logger("SymphonyHardwareInterface"),
        "Robot not safe to command (emergency/STO/fault). Stopped RT stream.");
      return hardware_interface::return_type::OK;
    }

    const auto now = std::chrono::steady_clock::now();
    if (last_rtpri_cmd_send_valid_) {
      const double elapsed =
        std::chrono::duration<double>(now - last_rtpri_cmd_send_).count();
      if (elapsed < rt_period_s_) {
        return hardware_interface::return_type::OK;
      }
    }

    std::vector<double> joint_commands_deg(kNumJoints);
    if (rt_speed_active_) {
      for (size_t i = 0; i < kNumJoints; ++i) {
        joint_commands_deg[i] = ros_rad_to_robot_deg(joint_velocity_commands_[i]);
      }
      command_->speedQ(joint_commands_deg);
      last_rtpri_cmd_send_ = now;
      last_rtpri_cmd_send_valid_ = true;
    } else if (rt_servox_active_) {
      if (now < rt_servox_send_after_) {
        return hardware_interface::return_type::OK;
      }
      const bool relative = moprim_opt_servo_relative_ > 0.5;
      if (!relative) {
        servox_rel_primed_ = false;
      }
      if (relative) {
        const int coord = static_cast<int>(std::lround(moprim_opt_coord_));
        uint8_t mode = 2;
        bool mode_ok = coord == 2 || coord < 0;
        if (coord == 3) {
          mode = 3;
          mode_ok = true;
        }
        const std::vector<double> zero_pose(6, 0.0);
        const bool new_cmd = tcp_pose_seq_cmd_ != last_servox_rel_seq_;
        auto send_zero = [&]() {
            command_->servoX(zero_pose, true, mode);
            last_rtpri_cmd_send_ = now;
            last_rtpri_cmd_send_valid_ = true;
          };
        if (!servox_rel_primed_) {
          last_servox_rel_seq_ = tcp_pose_seq_cmd_;
          tcp_pose_cmd_[0] = tcp_pose_cmd_[1] = tcp_pose_cmd_[2] = 0.0;
          tcp_pose_cmd_[3] = tcp_pose_cmd_[4] = tcp_pose_cmd_[5] = 0.0;
          tcp_pose_cmd_[6] = 1.0;
          servox_rel_primed_ = true;
          send_zero();
        } else if (!mode_ok) {
          if (new_cmd) {
            static rclcpp::Clock mode_clock(RCL_STEADY_TIME);
            RCLCPP_ERROR_THROTTLE(
              rclcpp::get_logger("SymphonyHardwareInterface"),
              mode_clock,
              2000,
              "servoX relative coord %d is not base (2) or tool (3); skip this tick",
              coord);
            last_servox_rel_seq_ = tcp_pose_seq_cmd_;
          }
          send_zero();
        } else if (!new_cmd) {
          send_zero();
        } else {
          std::vector<double> pose_mm_deg;
          const double dx = tcp_pose_cmd_[0];
          const double dy = tcp_pose_cmd_[1];
          const double dz = tcp_pose_cmd_[2];
          const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
          const double qdot = std::clamp(std::abs(tcp_pose_cmd_[6]), 0.0, 1.0);
          const double ang = 2.0 * std::acos(qdot);
          const bool finite = cartesian_to_rtpri_pose(
            tcp_pose_cmd_[0], tcp_pose_cmd_[1], tcp_pose_cmd_[2],
            tcp_pose_cmd_[3], tcp_pose_cmd_[4], tcp_pose_cmd_[5], tcp_pose_cmd_[6],
            pose_mm_deg);
          const bool over = (rt_max_tcp_jump_m_ > 0.0 && dist > rt_max_tcp_jump_m_) ||
            (rt_max_joint_jump_rad_ > 0.0 && ang > rt_max_joint_jump_rad_);
          if (!finite || over) {
            static rclcpp::Clock jump_clock(RCL_STEADY_TIME);
            RCLCPP_ERROR_THROTTLE(
              rclcpp::get_logger("SymphonyHardwareInterface"),
              jump_clock,
              2000,
              "servoX relative jump %.4f m / %.4f rad > limit %.4f m / %.4f rad; "
              "skip this tick",
              dist, ang, rt_max_tcp_jump_m_, rt_max_joint_jump_rad_);
            last_servox_rel_seq_ = tcp_pose_seq_cmd_;
            send_zero();
          } else {
            command_->servoX(pose_mm_deg, true, mode);
            last_servox_rel_seq_ = tcp_pose_seq_cmd_;
            last_rtpri_cmd_send_ = now;
            last_rtpri_cmd_send_valid_ = true;
          }
        }
      } else {
        std::vector<double> pose_mm_deg;
        if (!cartesian_to_rtpri_pose(
              tcp_pose_cmd_[0], tcp_pose_cmd_[1], tcp_pose_cmd_[2],
              tcp_pose_cmd_[3], tcp_pose_cmd_[4], tcp_pose_cmd_[5], tcp_pose_cmd_[6],
              pose_mm_deg))
        {
          static rclcpp::Clock pose_clock(RCL_STEADY_TIME);
          if (!snap_tcp_pose_cmd_to_measured() ||
            !cartesian_to_rtpri_pose(
              tcp_pose_cmd_[0], tcp_pose_cmd_[1], tcp_pose_cmd_[2],
              tcp_pose_cmd_[3], tcp_pose_cmd_[4], tcp_pose_cmd_[5], tcp_pose_cmd_[6],
              pose_mm_deg))
          {
            RCLCPP_ERROR_THROTTLE(
              rclcpp::get_logger("SymphonyHardwareInterface"),
              pose_clock,
              2000,
              "servoX pose is not finite and current TCP is not usable; not sending");
            return hardware_interface::return_type::OK;
          }
        }
        const double dx = tcp_pose_cmd_[0] - tcp_position_[0];
        const double dy = tcp_pose_cmd_[1] - tcp_position_[1];
        const double dz = tcp_pose_cmd_[2] - tcp_position_[2];
        const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
        const double qdot = std::clamp(
          std::abs(
            tcp_pose_cmd_[3] * tcp_orientation_[0] +
            tcp_pose_cmd_[4] * tcp_orientation_[1] +
            tcp_pose_cmd_[5] * tcp_orientation_[2] +
            tcp_pose_cmd_[6] * tcp_orientation_[3]),
          0.0, 1.0);
        const double ang = 2.0 * std::acos(qdot);
        if ((rt_max_tcp_jump_m_ > 0.0 && dist > rt_max_tcp_jump_m_) ||
          (rt_max_joint_jump_rad_ > 0.0 && ang > rt_max_joint_jump_rad_))
        {
          static rclcpp::Clock jump_clock(RCL_STEADY_TIME);
          snap_tcp_pose_cmd_to_measured();
          RCLCPP_ERROR_THROTTLE(
            rclcpp::get_logger("SymphonyHardwareInterface"),
            jump_clock,
            2000,
            "servoX jump %.4f m / %.4f rad > limit %.4f m / %.4f rad; skip this tick",
            dist, ang, rt_max_tcp_jump_m_, rt_max_joint_jump_rad_);
          return hardware_interface::return_type::OK;
        }
        command_->servoX(pose_mm_deg, false);
        last_rtpri_cmd_send_ = now;
        last_rtpri_cmd_send_valid_ = true;
      }
    } else {
      if (now < rt_servo_send_after_) {
        return hardware_interface::return_type::OK;
      }
      const bool relative = moprim_opt_servo_relative_ > 0.5;
      if (!relative) {
        servoq_rel_primed_ = false;
      }
      double max_jump = 0.0;
      for (size_t i = 0; i < kNumJoints; ++i) {
        const double rad = std::isfinite(joint_position_commands_[i])
          ? joint_position_commands_[i]
          : (relative ? 0.0 : joint_positions_[i]);
        max_jump = std::max(
          max_jump, std::abs(relative ? rad : (rad - joint_positions_[i])));
        joint_commands_deg[i] = ros_rad_to_robot_deg(rad);
      }
      if (relative) {
        const bool new_cmd = servoq_cmd_seq_ != last_servoq_rel_seq_;
        if (!servoq_rel_primed_) {
          last_servoq_rel_seq_ = servoq_cmd_seq_;
          servoq_rel_primed_ = true;
          command_->servoQ(std::vector<double>(kNumJoints, 0.0), true);
          last_rtpri_cmd_send_ = now;
          last_rtpri_cmd_send_valid_ = true;
          return hardware_interface::return_type::OK;
        }
        if (!new_cmd || max_jump <= 1e-12) {
          if (new_cmd) {
            last_servoq_rel_seq_ = servoq_cmd_seq_;
          }
          command_->servoQ(std::vector<double>(kNumJoints, 0.0), true);
          last_rtpri_cmd_send_ = now;
          last_rtpri_cmd_send_valid_ = true;
          return hardware_interface::return_type::OK;
        }
      }
      if (rt_max_joint_jump_rad_ > 0.0 && max_jump > rt_max_joint_jump_rad_) {
        static rclcpp::Clock jump_clock(RCL_STEADY_TIME);
        RCLCPP_WARN_THROTTLE(
          rclcpp::get_logger("SymphonyHardwareInterface"),
          jump_clock,
          2000,
          "servoQ jump %.4f rad > rt_max_joint_jump %.4f; skip this tick",
          max_jump, rt_max_joint_jump_rad_);
        if (relative) {
          last_servoq_rel_seq_ = servoq_cmd_seq_;
          command_->servoQ(std::vector<double>(kNumJoints, 0.0), true);
          last_rtpri_cmd_send_ = now;
          last_rtpri_cmd_send_valid_ = true;
        }
        return hardware_interface::return_type::OK;
      }
      if (relative) {
        command_->servoQ(joint_commands_deg, true);
        last_servoq_rel_seq_ = servoq_cmd_seq_;
      } else {
        command_->servoQ(joint_commands_deg, false);
      }
      last_rtpri_cmd_send_ = now;
      last_rtpri_cmd_send_valid_ = true;
    }
  }

  return hardware_interface::return_type::OK;
}

void SymphonyHardwareInterface::apply_gpio_commands()
{
  if (!connected_ || !control_) {
    return;
  }

  if (speed_rate_cmd_ >= 0.0 && speed_rate_cmd_ != last_speed_cmd_) {
    const double rate = std::clamp(speed_rate_cmd_, 0.0, 1.0);
    if ((rt_servo_active_ || rt_speed_active_ || rt_servox_active_) && rate < 0.99) {
      RCLCPP_WARN(
        rclcpp::get_logger("SymphonyHardwareInterface"),
        "setSpeed(%.3f) while RT active: does not time-scale the stream "
        "(overspeed limit only). Prefer 1.0; slow via Δq / velocity cmds",
        rate);
    }
    enqueue_control_job([this, rate]() {control_->setSpeed(rate);});
    last_speed_cmd_ = speed_rate_cmd_;
  }

  if (servo_enable_cmd_ >= 0.0 && servo_enable_cmd_ != last_servo_enable_cmd_) {
    const bool enable = servo_enable_cmd_ > 0.5;
    if (!enable) {
      request_stop_rt_streams();
    }
    const bool restart_speed = enable && activated_ &&
      active_command_modes_.count(kModeVelocity) != 0;
    const bool restart_servo = enable && activated_ &&
      active_command_modes_.count(kModePosition) != 0;
    const bool restart_servox = enable && activated_ &&
      active_command_modes_.count(kModeCartesianPose) != 0;

    enqueue_control_job([this, enable, restart_speed, restart_servo, restart_servox]() {
        control_->setServoEnable(0, enable);
        if (restart_speed && !start_rt_speed_locked(false)) {
          RCLCPP_ERROR(
            rclcpp::get_logger("SymphonyHardwareInterface"),
            "startRtSpeedQControl failed after set_servo_enable");
        } else if (restart_servo && !start_rt_servo_locked(false)) {
          RCLCPP_ERROR(
            rclcpp::get_logger("SymphonyHardwareInterface"),
            "startRtServoQControl failed after set_servo_enable");
        } else if (restart_servox && !start_rt_servox_locked(false)) {
          RCLCPP_ERROR(
            rclcpp::get_logger("SymphonyHardwareInterface"),
            "startRtServoXControl failed after set_servo_enable");
        }
      });
    last_servo_enable_cmd_ = servo_enable_cmd_;
  }

  if (clear_fault_cmd_ != last_clear_fault_cmd_ && clear_fault_cmd_ > 0.0) {
    const bool warning = clear_fault_warning_ > 0.5;
    enqueue_control_job([this, warning]() {control_->clearFault(0, warning);});
    last_clear_fault_cmd_ = clear_fault_cmd_;
  }

  if (collision_enable_cmd_ >= 0.0 &&
    collision_enable_cmd_ != last_collision_enable_cmd_)
  {
    const bool enable = collision_enable_cmd_ > 0.5;
    enqueue_control_job([this, enable]() {
        control_->setEnableCollisionDetection(enable);
      });
    last_collision_enable_cmd_ = collision_enable_cmd_;
  }

  if (rt_period_cmd_ > 0.0 && rt_period_cmd_ != last_rt_period_cmd_) {
    rt_period_s_ = rt_period_cmd_;
    last_rt_period_cmd_ = rt_period_cmd_;
  }
  if (rt_filter_cmd_ >= 0.0 && rt_filter_cmd_ != last_rt_filter_cmd_) {
    const auto shared = static_cast<uint32_t>(std::lround(rt_filter_cmd_));
    rt_servo_filter_ = shared;
    rt_speed_filter_ = shared;
    last_rt_filter_cmd_ = rt_filter_cmd_;
  }

  if (pause_rt_cmd_ != last_pause_rt_cmd_ && pause_rt_cmd_ > 0.0) {
#if defined(SYMPHONY_RTPRI_HAS_1_3)
    enqueue_control_job([this]() {control_->pauseRtControl();});
#else
    RCLCPP_WARN(
      rclcpp::get_logger("SymphonyHardwareInterface"),
      "pause_rt requires puloon_rtpri >= 1.3.0");
#endif
    last_pause_rt_cmd_ = pause_rt_cmd_;
  }

  if (resume_rt_cmd_ != last_resume_rt_cmd_ && resume_rt_cmd_ > 0.0) {
#if defined(SYMPHONY_RTPRI_HAS_1_3)
    enqueue_control_job([this]() {
        if (!control_->resumeRtControl()) {
          RCLCPP_WARN(
            rclcpp::get_logger("SymphonyHardwareInterface"),
            "resumeRtControl returned false");
        }
      });
#else
    RCLCPP_WARN(
      rclcpp::get_logger("SymphonyHardwareInterface"),
      "resume_rt requires puloon_rtpri >= 1.3.0");
#endif
    last_resume_rt_cmd_ = resume_rt_cmd_;
  }
}

void SymphonyHardwareInterface::sync_tcp_sensors_from_robot()
{
  if (!status_) {
    return;
  }

  const std::vector<double> pose = status_->getActualPose();
  if (pose.size() >= 6) {
    // [mm] -> [m]; RPY stays [deg] until quat
    tcp_position_[0] = pose[0] * 0.001;
    tcp_position_[1] = pose[1] * 0.001;
    tcp_position_[2] = pose[2] * 0.001;
    rpy_deg_to_quat(pose[3], pose[4], pose[5], tcp_orientation_);
  }

  const std::vector<double> wrench = status_->getTcpForce();
  if (wrench.size() >= 6) {
    tcp_force_[0] = wrench[0];
    tcp_force_[1] = wrench[1];
    tcp_force_[2] = wrench[2];
    tcp_torque_[0] = wrench[3];
    tcp_torque_[1] = wrench[4];
    tcp_torque_[2] = wrench[5];
  }
}

void SymphonyHardwareInterface::start_async_worker()
{
  if (async_worker_running_) {
    return;
  }
  async_worker_running_ = true;
  async_worker_ = std::thread(&SymphonyHardwareInterface::async_worker_loop, this);
}

void SymphonyHardwareInterface::stop_async_worker()
{
  if (async_worker_running_) {
    async_worker_running_ = false;
    async_cv_.notify_all();
  }
  if (async_worker_.joinable()) {
    async_worker_.join();
  }
}

void SymphonyHardwareInterface::enqueue_control_job(std::function<void()> fn)
{
  if (!fn) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(async_queue_mutex_);
    AsyncJob job;
    job.kind = AsyncJobKind::Control;
    job.fn = std::move(fn);
    async_queue_.push(std::move(job));
  }
  async_cv_.notify_one();
}

void SymphonyHardwareInterface::async_worker_loop()
{
  while (true) {
    AsyncJob job;
    {
      std::unique_lock<std::mutex> lock(async_queue_mutex_);
      async_cv_.wait(lock, [this] {
        return !async_queue_.empty() || !async_worker_running_;
      });
      if (async_queue_.empty()) {
        break;
      }
      job = std::move(async_queue_.front());
      async_queue_.pop();
    }

    if (!connected_ || !control_) {
      if (job.kind == AsyncJobKind::Control) {
        continue;
      }
      moprim_status_code_ = kExecError;
      moprim_ready_flag_ = true;
      if (job.kind == AsyncJobKind::NativeJointTrajectory) {
        native_jt_failed_ = true;
        native_jt_executing_ = false;
        native_jt_wait_start_ = false;
      }
      continue;
    }

    std::lock_guard<std::mutex> tcp_lock(tcp_mutex_);
    switch (job.kind) {
      case AsyncJobKind::Stop:
        control_->moveStop();
        moprim_status_code_ = kExecStopping;
        moprim_wait_start_ = false;
        break;
      case AsyncJobKind::Pause:
        control_->movePause();
        break;
      case AsyncJobKind::Resume:
        control_->moveResume();
        break;
      case AsyncJobKind::Primitive:
        execute_moprim_primitive(job.cmd);
        break;
      case AsyncJobKind::NativeJointTrajectory:
        execute_native_jt(job.waypoints, job.speeds, job.blends);
        break;
      case AsyncJobKind::Control:
        job.fn();
        break;
    }
  }
}

//----- lifecycle / export / mode switch -----

hardware_interface::CallbackReturn SymphonyHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & hardware_info)
{
  if (hardware_interface::SystemInterface::on_init(hardware_info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != kNumJoints) {
    RCLCPP_ERROR(
      rclcpp::get_logger("SymphonyHardwareInterface"),
      "Expected %zu joints, got %zu", kNumJoints, info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  joint_positions_.assign(kNumJoints, 0.0);
  joint_velocities_.assign(kNumJoints, 0.0);
  joint_efforts_.assign(kNumJoints, 0.0);
  joint_position_commands_.assign(kNumJoints, 0.0);
  joint_velocity_commands_.assign(kNumJoints, 0.0);
  fill_nan(moprim_commands_);
  moprim_execution_status_ = static_cast<double>(kExecIdle);
  moprim_ready_ = 1.0;
  moprim_status_code_ = kExecIdle;
  moprim_ready_flag_ = true;

  if (!parse_hardware_parameters()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  motion_mode_switch_ = create_motion_mode_switch_table();

  for (const auto & joint : info_.joints) {
    bool has_position = false;
    bool has_velocity = false;
    for (const auto & ci : joint.command_interfaces) {
      if (ci.name == hardware_interface::HW_IF_POSITION) {
        has_position = true;
      } else if (ci.name == hardware_interface::HW_IF_VELOCITY) {
        has_velocity = true;
      } else {
        RCLCPP_ERROR(
          rclcpp::get_logger("SymphonyHardwareInterface"),
          "Joint '%s' has unsupported command interface '%s'",
          joint.name.c_str(), ci.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    if (!has_position || !has_velocity) {
      RCLCPP_ERROR(
        rclcpp::get_logger("SymphonyHardwareInterface"),
        "Joint '%s' must expose position and velocity command interfaces",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() < 2 ||
        joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
        joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("SymphonyHardwareInterface"),
        "Joint '%s' must expose position and velocity state interfaces",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SymphonyHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  try {
    disconnect_robot();
    control_.reset();
    command_.reset();
    status_.reset();

    control_ = std::make_shared<rtpri::RTPRIControl>(robot_ip_);
    command_ = std::make_shared<rtpri::RTPRICommand>(robot_ip_);
    status_ = std::make_shared<rtpri::RTPRIStatus>(robot_ip_);

    if (!connect_robot()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("SymphonyHardwareInterface"),
        "Failed to connect RTPRI clients at %s "
        "(Control TCP:4500 / Command UDP:4200 / Status UDP:4300)",
        robot_ip_.c_str());
      disconnect_robot();
      control_.reset();
      command_.reset();
      status_.reset();
      return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("SymphonyHardwareInterface"),
      "on_configure exception: %s", e.what());
    disconnect_robot();
    control_.reset();
    command_.reset();
    status_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }
}

hardware_interface::CallbackReturn SymphonyHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  disconnect_robot();
  control_.reset();
  command_.reset();
  status_.reset();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SymphonyHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  try {
    if (!connected_ || !control_ || !command_ || !status_) {
      RCLCPP_ERROR(
        rclcpp::get_logger("SymphonyHardwareInterface"),
        "Cannot activate: RTPRI clients are not connected. "
        "Did on_configure succeed?");
      return hardware_interface::CallbackReturn::ERROR;
    }

    try {
      if (!sync_state_from_robot()) {
        RCLCPP_WARN(
          rclcpp::get_logger("SymphonyHardwareInterface"),
          "Connected, but failed to read initial joint state from RTPRI. "
          "Keeping zero-initialized state until read() succeeds.");
      }
    } catch (...) {
      RCLCPP_WARN(
        rclcpp::get_logger("SymphonyHardwareInterface"),
        "sync_state_from_robot threw during activate; continuing");
    }
    joint_position_commands_ = joint_positions_;

#if defined(SYMPHONY_RTPRI_HAS_1_3)
    if (!control_->requestControlAuthority()) {
      RCLCPP_WARN(
        rclcpp::get_logger("SymphonyHardwareInterface"),
        "requestControlAuthority returned false; continuing activate");
    }
#endif

    if (!enable_servo()) {
      RCLCPP_WARN(
        rclcpp::get_logger("SymphonyHardwareInterface"),
        "setServoEnable did not confirm; activating anyway");
    }

    start_async_worker();

    last_speed_cmd_ = initial_speed_;
    last_servo_enable_cmd_ = 1.0;
    last_tm_seq_ = tm_seq_;
    last_clear_fault_cmd_ = clear_fault_cmd_;
    last_pause_rt_cmd_ = pause_rt_cmd_;
    last_resume_rt_cmd_ = resume_rt_cmd_;

    activated_ = true;
    active_command_modes_.clear();
    pending_command_modes_.clear();
    command_watchdog_stamp_valid_ = false;
    command_watchdog_tripped_ = false;

    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("SymphonyHardwareInterface"),
      "on_activate exception: %s — returning SUCCESS so bringup does not abort",
      e.what());
    activated_ = true;
    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (...) {
    RCLCPP_ERROR(
      rclcpp::get_logger("SymphonyHardwareInterface"),
      "on_activate non-std exception — returning SUCCESS so bringup does not abort");
    activated_ = true;
    return hardware_interface::CallbackReturn::SUCCESS;
  }
}

hardware_interface::CallbackReturn SymphonyHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (control_) {
    request_moprim_stop();
  }
  stop_async_worker();
  activated_ = false;

  if (force_mode_active_ && control_) {
    control_->setEnableForceControl(false);
    control_->setEnableCollisionDetection(true);
    force_mode_active_ = false;
    force_enabled_ = 0.0;
  }

  if (control_) {
    std::lock_guard<std::mutex> lock(tcp_mutex_);
    control_->moveStop();
#if defined(SYMPHONY_RTPRI_HAS_1_3)
    control_->releaseControlAuthority();
#endif
  }
  stop_rt_streams();
  active_command_modes_.clear();
  pending_command_modes_.clear();
  command_watchdog_stamp_valid_ = false;
  command_watchdog_tripped_ = false;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SymphonyHardwareInterface::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_ERROR(
    rclcpp::get_logger("SymphonyHardwareInterface"),
    "on_error: leaving hardware recoverable (UNCONFIGURED)");
  activated_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SymphonyHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_positions_[i]);
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_velocities_[i]);
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joint_efforts_[i]);
  }
  state_interfaces.emplace_back(
    "speed_rate", "value", &speed_rate_);
  state_interfaces.emplace_back("robot_status", "op_mode", &op_mode_);
  state_interfaces.emplace_back("robot_status", "safety_status", &safety_status_);
  state_interfaces.emplace_back("robot_status", "safety_zone", &safety_zone_);
  state_interfaces.emplace_back("robot_status", "arm_state", &arm_state_);
  state_interfaces.emplace_back("robot_status", "tool_io_status", &tool_io_status_);
  state_interfaces.emplace_back("robot_status", "servo_enable", &servo_enable_);
  for (size_t i = 0; i < kArmErrorSlots; ++i) {
    state_interfaces.emplace_back(
      "robot_status", "arm_error_" + std::to_string(i), &arm_errors_[i]);
  }
  state_interfaces.emplace_back("io", "safety_di", &io_safety_di_);
  state_interfaces.emplace_back("io", "safety_do", &io_safety_do_);
  state_interfaces.emplace_back("io", "safety_ai0", &io_safety_ai_[0]);
  state_interfaces.emplace_back("io", "safety_ai1", &io_safety_ai_[1]);
  state_interfaces.emplace_back("io", "safety_ao0", &io_safety_ao_[0]);
  state_interfaces.emplace_back("io", "safety_ao1", &io_safety_ao_[1]);
  state_interfaces.emplace_back("io", "tool_di", &io_tool_di_);
  state_interfaces.emplace_back("io", "tool_do", &io_tool_do_);
  state_interfaces.emplace_back("io", "tool_ai0", &io_tool_ai_[0]);
  state_interfaces.emplace_back("io", "tool_ai1", &io_tool_ai_[1]);
  state_interfaces.emplace_back("io", "ex1_di", &io_ex1_di_);
  state_interfaces.emplace_back("io", "ex1_do", &io_ex1_do_);
  state_interfaces.emplace_back("io", "ex1_ai0", &io_ex1_ai_[0]);
  state_interfaces.emplace_back("io", "ex1_ai1", &io_ex1_ai_[1]);
  state_interfaces.emplace_back("io", "ex1_ai2", &io_ex1_ai_[2]);
  state_interfaces.emplace_back("io", "ex1_ai3", &io_ex1_ai_[3]);
  state_interfaces.emplace_back("io", "ex1_ao0", &io_ex1_ao_[0]);
  state_interfaces.emplace_back("io", "ex1_ao1", &io_ex1_ao_[1]);
  state_interfaces.emplace_back("io", "ex1_ao2", &io_ex1_ao_[2]);
  state_interfaces.emplace_back("io", "ex1_ao3", &io_ex1_ao_[3]);
  state_interfaces.emplace_back("io", "ex2_di", &io_ex2_di_);
  state_interfaces.emplace_back("io", "ex2_do", &io_ex2_do_);
  state_interfaces.emplace_back("io", "ex2_ai0", &io_ex2_ai_[0]);
  state_interfaces.emplace_back("io", "ex2_ai1", &io_ex2_ai_[1]);
  state_interfaces.emplace_back("io", "ex2_ai2", &io_ex2_ai_[2]);
  state_interfaces.emplace_back("io", "ex2_ai3", &io_ex2_ai_[3]);
  state_interfaces.emplace_back("io", "ex2_ao0", &io_ex2_ao_[0]);
  state_interfaces.emplace_back("io", "ex2_ao1", &io_ex2_ao_[1]);
  state_interfaces.emplace_back("io", "ex2_ao2", &io_ex2_ao_[2]);
  state_interfaces.emplace_back("io", "ex2_ao3", &io_ex2_ao_[3]);
  state_interfaces.emplace_back("force", "enabled", &force_enabled_);
  state_interfaces.emplace_back("tool_modbus", "done_seq", &tm_done_seq_);
  state_interfaces.emplace_back("tool_modbus", "ok", &tm_ok_);
  state_interfaces.emplace_back("tool_modbus", "command", &tm_command_);
  state_interfaces.emplace_back("tool_modbus", "address", &tm_echo_address_);
  state_interfaces.emplace_back("tool_modbus", "number", &tm_number_);
  for (size_t i = 0; i < 8; ++i) {
    state_interfaces.emplace_back(
      "tool_modbus", "result_" + std::to_string(i), &tm_result_[i]);
  }
  state_interfaces.emplace_back("tcp_pose", "position.x", &tcp_position_[0]);
  state_interfaces.emplace_back("tcp_pose", "position.y", &tcp_position_[1]);
  state_interfaces.emplace_back("tcp_pose", "position.z", &tcp_position_[2]);
  state_interfaces.emplace_back("tcp_pose", "orientation.x", &tcp_orientation_[0]);
  state_interfaces.emplace_back("tcp_pose", "orientation.y", &tcp_orientation_[1]);
  state_interfaces.emplace_back("tcp_pose", "orientation.z", &tcp_orientation_[2]);
  state_interfaces.emplace_back("tcp_pose", "orientation.w", &tcp_orientation_[3]);
  state_interfaces.emplace_back("tcp_fts_sensor", "force.x", &tcp_force_[0]);
  state_interfaces.emplace_back("tcp_fts_sensor", "force.y", &tcp_force_[1]);
  state_interfaces.emplace_back("tcp_fts_sensor", "force.z", &tcp_force_[2]);
  state_interfaces.emplace_back("tcp_fts_sensor", "torque.x", &tcp_torque_[0]);
  state_interfaces.emplace_back("tcp_fts_sensor", "torque.y", &tcp_torque_[1]);
  state_interfaces.emplace_back("tcp_fts_sensor", "torque.z", &tcp_torque_[2]);
  state_interfaces.emplace_back(
    "motion_primitive", "execution_status", &moprim_execution_status_);
  state_interfaces.emplace_back(
    "motion_primitive", "ready_for_new_primitive", &moprim_ready_);
  state_interfaces.emplace_back("motion_options", "work", &moprim_opt_work_);
  state_interfaces.emplace_back("motion_options", "tool", &moprim_opt_tool_);
  state_interfaces.emplace_back("motion_options", "rel", &moprim_opt_rel_);
  state_interfaces.emplace_back(
    "motion_options", "fixedspeed", &moprim_opt_fixedspeed_);
  state_interfaces.emplace_back("motion_options", "cfg", &moprim_opt_cfg_);
  state_interfaces.emplace_back("motion_options", "coord", &moprim_opt_coord_);
  state_interfaces.emplace_back("motion_options", "weaving", &moprim_opt_weaving_);
  state_interfaces.emplace_back(
    "motion_options", "fixedorient", &moprim_opt_fixedorient_);
  state_interfaces.emplace_back("motion_options", "ext", &moprim_opt_ext_);
  state_interfaces.emplace_back("motion_options", "tol", &moprim_opt_tol_);
  state_interfaces.emplace_back(
    "motion_options", "servo_relative", &moprim_opt_servo_relative_);
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> SymphonyHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_position_commands_[i]);
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_velocity_commands_[i]);
  }
  command_interfaces.emplace_back("tcp_pose", "position.x", &tcp_pose_cmd_[0]);
  command_interfaces.emplace_back("tcp_pose", "position.y", &tcp_pose_cmd_[1]);
  command_interfaces.emplace_back("tcp_pose", "position.z", &tcp_pose_cmd_[2]);
  command_interfaces.emplace_back("tcp_pose", "orientation.x", &tcp_pose_cmd_[3]);
  command_interfaces.emplace_back("tcp_pose", "orientation.y", &tcp_pose_cmd_[4]);
  command_interfaces.emplace_back("tcp_pose", "orientation.z", &tcp_pose_cmd_[5]);
  command_interfaces.emplace_back("tcp_pose", "orientation.w", &tcp_pose_cmd_[6]);
  command_interfaces.emplace_back("tcp_pose", "seq", &tcp_pose_seq_cmd_);
  command_interfaces.emplace_back("servo_q", "seq", &servoq_cmd_seq_);
  command_interfaces.emplace_back(
    "speed_rate", "command", &speed_rate_cmd_);
  command_interfaces.emplace_back(
    "robot_status", "servo_enable_cmd", &servo_enable_cmd_);
  command_interfaces.emplace_back(
    "robot_status", "clear_fault_cmd", &clear_fault_cmd_);
  command_interfaces.emplace_back(
    "robot_status", "clear_fault_warning", &clear_fault_warning_);
  command_interfaces.emplace_back(
    "robot_status", "collision_enable_cmd", &collision_enable_cmd_);
  command_interfaces.emplace_back(
    "robot_status", "rt_period_cmd", &rt_period_cmd_);
  command_interfaces.emplace_back(
    "robot_status", "rt_filter_cmd", &rt_filter_cmd_);
  command_interfaces.emplace_back(
    "robot_status", "pause_rt_cmd", &pause_rt_cmd_);
  command_interfaces.emplace_back(
    "robot_status", "resume_rt_cmd", &resume_rt_cmd_);
  command_interfaces.emplace_back("io", "digital_seq", &io_digital_seq_);
  command_interfaces.emplace_back("io", "digital_bank", &io_digital_bank_);
  command_interfaces.emplace_back("io", "digital_pin", &io_digital_pin_);
  command_interfaces.emplace_back("io", "digital_value", &io_digital_value_);
  command_interfaces.emplace_back("io", "analog_seq", &io_analog_seq_);
  command_interfaces.emplace_back("io", "analog_bank", &io_analog_bank_);
  command_interfaces.emplace_back("io", "analog_channel", &io_analog_channel_);
  command_interfaces.emplace_back("io", "analog_value", &io_analog_value_);
  command_interfaces.emplace_back("tool_modbus", "seq", &tm_seq_);
  command_interfaces.emplace_back("tool_modbus", "op", &tm_op_);
  command_interfaces.emplace_back("tool_modbus", "device", &tm_device_);
  command_interfaces.emplace_back("tool_modbus", "address", &tm_address_);
  command_interfaces.emplace_back("tool_modbus", "size", &tm_size_);
  for (size_t i = 0; i < 8; ++i) {
    command_interfaces.emplace_back(
      "tool_modbus", "value_" + std::to_string(i), &tm_value_[i]);
  }
  command_interfaces.emplace_back("force", "payload_seq", &fp_payload_seq_);
  command_interfaces.emplace_back("force", "mass", &fp_payload_[0]);
  command_interfaces.emplace_back("force", "cog_x", &fp_payload_[1]);
  command_interfaces.emplace_back("force", "cog_y", &fp_payload_[2]);
  command_interfaces.emplace_back("force", "cog_z", &fp_payload_[3]);
  command_interfaces.emplace_back("force", "ixx", &fp_payload_[4]);
  command_interfaces.emplace_back("force", "iyy", &fp_payload_[5]);
  command_interfaces.emplace_back("force", "izz", &fp_payload_[6]);
  command_interfaces.emplace_back("force", "gravity_seq", &fp_gravity_seq_);
  command_interfaces.emplace_back("force", "gx", &fp_gravity_[0]);
  command_interfaces.emplace_back("force", "gy", &fp_gravity_[1]);
  command_interfaces.emplace_back("force", "gz", &fp_gravity_[2]);
  command_interfaces.emplace_back("force", "bias_seq", &fp_bias_seq_);
  command_interfaces.emplace_back("force", "force_seq", &fp_force_seq_);
  command_interfaces.emplace_back("force", "enable", &fp_force_enable_);
  command_interfaces.emplace_back("force", "fx", &fp_wrench_[0]);
  command_interfaces.emplace_back("force", "fy", &fp_wrench_[1]);
  command_interfaces.emplace_back("force", "fz", &fp_wrench_[2]);
  command_interfaces.emplace_back("force", "tx", &fp_wrench_[3]);
  command_interfaces.emplace_back("force", "ty", &fp_wrench_[4]);
  command_interfaces.emplace_back("force", "tz", &fp_wrench_[5]);
  command_interfaces.emplace_back("force", "param_seq", &fp_param_seq_);
  command_interfaces.emplace_back("force", "set_imp_mass", &fp_set_imp_mass_);
  command_interfaces.emplace_back(
    "force", "set_imp_stiffness", &fp_set_imp_stiffness_);
  command_interfaces.emplace_back("force", "set_imp_damping", &fp_set_imp_damping_);
  command_interfaces.emplace_back("force", "set_compliance", &fp_set_compliance_);
  for (size_t i = 0; i < 6; ++i) {
    const auto idx = std::to_string(i);
    command_interfaces.emplace_back("force", "imp_mass_" + idx, &fp_imp_mass_[i]);
    command_interfaces.emplace_back(
      "force", "imp_stiffness_" + idx, &fp_imp_stiffness_[i]);
    command_interfaces.emplace_back(
      "force", "imp_damping_" + idx, &fp_imp_damping_[i]);
    command_interfaces.emplace_back("force", "compliance_" + idx, &fp_compliance_[i]);
  }
  for (size_t i = 0; i < kMoprimCmdCount; ++i) {
    command_interfaces.emplace_back(
      "motion_primitive", kMoprimCmdNames[i], &moprim_commands_[i]);
  }
  command_interfaces.emplace_back("motion_primitive", "pause", &moprim_pause_cmd_);
  command_interfaces.emplace_back("motion_primitive", "resume", &moprim_resume_cmd_);
  command_interfaces.emplace_back("motion_options", "work", &moprim_opt_work_);
  command_interfaces.emplace_back("motion_options", "tool", &moprim_opt_tool_);
  command_interfaces.emplace_back("motion_options", "rel", &moprim_opt_rel_);
  command_interfaces.emplace_back(
    "motion_options", "fixedspeed", &moprim_opt_fixedspeed_);
  command_interfaces.emplace_back("motion_options", "cfg", &moprim_opt_cfg_);
  command_interfaces.emplace_back("motion_options", "coord", &moprim_opt_coord_);
  command_interfaces.emplace_back("motion_options", "weaving", &moprim_opt_weaving_);
  command_interfaces.emplace_back(
    "motion_options", "fixedorient", &moprim_opt_fixedorient_);
  command_interfaces.emplace_back("motion_options", "ext", &moprim_opt_ext_);
  command_interfaces.emplace_back("motion_options", "tol", &moprim_opt_tol_);
  command_interfaces.emplace_back(
    "motion_options", "servo_relative", &moprim_opt_servo_relative_);
  for (size_t i = 0; i < kNumJoints; ++i) {
    const auto idx = std::to_string(i);
    command_interfaces.emplace_back(
      "native_joint_trajectory", "setpoint_positions_" + idx, &native_jt_positions_[i]);
    command_interfaces.emplace_back(
      "native_joint_trajectory", "setpoint_velocities_" + idx, &native_jt_velocities_[i]);
    command_interfaces.emplace_back(
      "native_joint_trajectory", "setpoint_accelerations_" + idx, &native_jt_accelerations_[i]);
  }
  command_interfaces.emplace_back("native_joint_trajectory", "transfer_state", &native_jt_transfer_state_);
  command_interfaces.emplace_back("native_joint_trajectory", "time_from_start", &native_jt_time_from_start_);
  command_interfaces.emplace_back("native_joint_trajectory", "abort", &native_jt_abort_);
  command_interfaces.emplace_back("native_joint_trajectory", "size", &native_jt_size_);
  return command_interfaces;
}

bool SymphonyHardwareInterface::parse_hardware_parameters()
{
  auto get_param = [this](const std::string & key) -> std::string {
    if (info_.hardware_parameters.count(key) == 0) {
      return "";
    }
    return info_.hardware_parameters.at(key);
  };

  const auto robot_ip = get_param("robot_ip");
  if (!robot_ip.empty()) {
    robot_ip_ = robot_ip;
  }

  try {
    const auto rt_period = get_param("rt_period");
    if (!rt_period.empty()) {
      rt_period_s_ = std::stod(rt_period);
    }
    const auto rt_filter = get_param("rt_filter");
    if (!rt_filter.empty()) {
      const auto shared = static_cast<uint32_t>(std::stoul(rt_filter));
      rt_servo_filter_ = shared;
      rt_speed_filter_ = shared;
    }
    const auto rt_servo_filter = get_param("rt_servo_filter");
    if (!rt_servo_filter.empty()) {
      rt_servo_filter_ = static_cast<uint32_t>(std::stoul(rt_servo_filter));
    }
    const auto rt_speed_filter = get_param("rt_speed_filter");
    if (!rt_speed_filter.empty()) {
      rt_speed_filter_ = static_cast<uint32_t>(std::stoul(rt_speed_filter));
    }
    const auto initial_speed = get_param("initial_speed");
    if (!initial_speed.empty()) {
      initial_speed_ = std::stod(initial_speed);
    }
    const auto rt_client_id = get_param("rt_client_id");
    if (!rt_client_id.empty()) {
      rt_client_id_ = static_cast<uint16_t>(std::stoul(rt_client_id));
    }
    const auto rt_settle = get_param("rt_settle");
    if (!rt_settle.empty()) {
      rt_settle_s_ = std::stod(rt_settle);
    }
    const auto rt_jump = get_param("rt_max_joint_jump");
    if (!rt_jump.empty()) {
      rt_max_joint_jump_rad_ = std::stod(rt_jump);
    }
    const auto rt_tcp_jump = get_param("rt_max_tcp_jump");
    if (!rt_tcp_jump.empty()) {
      rt_max_tcp_jump_m_ = std::stod(rt_tcp_jump);
    }
    const auto watchdog = get_param("command_watchdog_timeout");
    if (!watchdog.empty()) {
      command_watchdog_timeout_s_ = std::stod(watchdog);
    }
    const auto max_points = get_param("native_joint_trajectory_max_points");
    if (!max_points.empty()) {
      native_jt_max_points_ = static_cast<size_t>(std::stoul(max_points));
    }
    const auto blend = get_param("native_joint_trajectory_blend");
    if (!blend.empty()) {
      native_jt_blend_ = std::stod(blend);
    }
    const auto velocities = get_param("max_joint_velocities");
    if (!velocities.empty()) {
      max_joint_velocities_.clear();
      std::stringstream stream(velocities);
      std::string token;
      while (std::getline(stream, token, ',')) {
        max_joint_velocities_.push_back(std::stod(token));
      }
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      rclcpp::get_logger("SymphonyHardwareInterface"),
      "Invalid hardware parameter: %s", ex.what());
    return false;
  }

  if (rt_period_s_ <= 0.0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("SymphonyHardwareInterface"),
      "rt_period must be > 0 (got %.4f)", rt_period_s_);
    return false;
  }

  if (rt_settle_s_ < 0.0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("SymphonyHardwareInterface"),
      "rt_settle must be >= 0 (got %.4f)", rt_settle_s_);
    return false;
  }
  if (rt_max_joint_jump_rad_ < 0.0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("SymphonyHardwareInterface"),
      "rt_max_joint_jump must be >= 0 (got %.4f)", rt_max_joint_jump_rad_);
    return false;
  }
  if (rt_max_tcp_jump_m_ < 0.0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("SymphonyHardwareInterface"),
      "rt_max_tcp_jump must be >= 0 (got %.4f)", rt_max_tcp_jump_m_);
    return false;
  }

  if (command_watchdog_timeout_s_ < 0.0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("SymphonyHardwareInterface"),
      "command_watchdog_timeout must be >= 0 (got %.4f)", command_watchdog_timeout_s_);
    return false;
  }

  if (native_jt_max_points_ == 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("SymphonyHardwareInterface"),
      "native_joint_trajectory_max_points must be > 0");
    return false;
  }

  native_jt_blend_ = std::clamp(native_jt_blend_, 0.0, 100.0);

  if (!max_joint_velocities_.empty()) {
    if (max_joint_velocities_.size() != kNumJoints) {
      RCLCPP_ERROR(
        rclcpp::get_logger("SymphonyHardwareInterface"),
        "max_joint_velocities must list %zu values, got %zu",
        kNumJoints, max_joint_velocities_.size());
      return false;
    }
    for (const double velocity : max_joint_velocities_) {
      if (!std::isfinite(velocity) || velocity <= 0.0) {
        RCLCPP_ERROR(
          rclcpp::get_logger("SymphonyHardwareInterface"),
          "max_joint_velocities must be finite and > 0");
        return false;
      }
    }
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger("SymphonyHardwareInterface"),
      "max_joint_velocities not set: native JTC ignores trajectory timing and "
      "runs every segment at %.0f%% speed", kNativeJtDefaultSpeed);
  }

  return true;
}

std::unordered_set<std::string> SymphonyHardwareInterface::collect_command_modes(
  const std::vector<std::string> & interface_names) const
{
  std::unordered_set<std::string> modes;
  for (const auto & name : interface_names) {
    if (const auto key = command_mode_key(name)) {
      modes.insert(*key);
      continue;
    }
    if (name.find("velocity") != std::string::npos) {
      modes.insert(kModeVelocity);
      continue;
    }
    for (const auto & joint : info_.joints) {
      if (name == joint.name ||
          name.find(joint.name + "/") != std::string::npos ||
          name.find("/" + joint.name) != std::string::npos)
      {
        modes.insert(kModePosition);
        break;
      }
    }
  }
  return modes;
}

hardware_interface::return_type SymphonyHardwareInterface::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  auto next = active_command_modes_;
  for (const auto & mode : collect_command_modes(stop_interfaces)) {
    next.erase(mode);
  }
  for (const auto & mode : collect_command_modes(start_interfaces)) {
    next.insert(mode);
  }

  if (!mode_set_compatible(motion_mode_switch_, next)) {
    std::string listed;
    for (const auto & mode : next) {
      if (!listed.empty()) {
        listed += ", ";
      }
      listed += mode;
    }
    RCLCPP_ERROR(
      rclcpp::get_logger("SymphonyHardwareInterface"),
      "Incompatible command modes in switch [%s] (see motion_mode_switch table)",
      listed.c_str());
    pending_command_modes_.clear();
    return hardware_interface::return_type::ERROR;
  }

  pending_command_modes_ = next;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SymphonyHardwareInterface::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  auto next = active_command_modes_;
  for (const auto & mode : collect_command_modes(stop_interfaces)) {
    next.erase(mode);
  }
  for (const auto & mode : collect_command_modes(start_interfaces)) {
    next.insert(mode);
  }
  if (start_interfaces.empty() && stop_interfaces.empty() &&
      !pending_command_modes_.empty())
  {
    next = pending_command_modes_;
  } else {
    pending_command_modes_ = next;
  }

  if (!apply_rt_mode(next)) {
    pending_command_modes_.clear();
    return hardware_interface::return_type::ERROR;
  }
  active_command_modes_ = next;
  pending_command_modes_.clear();
  command_watchdog_stamp_valid_ = false;
  command_watchdog_tripped_ = false;
  return hardware_interface::return_type::OK;
}

bool SymphonyHardwareInterface::enable_servo()
{
  try {
    control_->setSpeed(initial_speed_);
    control_->setServoEnable(0, true);
  } catch (...) {
    RCLCPP_ERROR(
      rclcpp::get_logger("SymphonyHardwareInterface"),
      "setSpeed/setServoEnable threw");
    return false;
  }

  if (!status_->getServoEnable(0)) {
    RCLCPP_WARN(
      rclcpp::get_logger("SymphonyHardwareInterface"),
      "getServoEnable(0) false right after setServoEnable; not waiting");
  }
  return true;
}

//----- RT streams (servoQ / speedQ) -----

bool SymphonyHardwareInterface::apply_rt_mode(const std::unordered_set<std::string> & modes)
{
  const bool want_position = modes.count(kModePosition) != 0;
  const bool want_velocity = modes.count(kModeVelocity) != 0;
  const bool want_cartesian = modes.count(kModeCartesianPose) != 0;
  const bool want_primitive = modes.count(kModeMotionPrimitive) != 0;
  const bool want_native_jt = modes.count(kModeNativeJointTrajectory) != 0;
  const bool drop_primitive = !want_primitive &&
    active_command_modes_.count(kModeMotionPrimitive) != 0 && connected_ && control_;
  const bool drop_native_jt = !want_native_jt &&
    active_command_modes_.count(kModeNativeJointTrajectory) != 0 && connected_ && control_;

  if (drop_primitive || drop_native_jt) {
    request_moprim_stop();
  }

  std::lock_guard<std::mutex> lock(tcp_mutex_);

  if (drop_primitive) {
    control_->moveStop();
    moprim_status_code_ = kExecIdle;
    moprim_ready_flag_ = true;
    moprim_wait_start_ = false;
    fill_nan(moprim_commands_);
  }

  if (drop_native_jt) {
    control_->moveStop();
    reset_native_jt();
  }

  if (!want_velocity) {
    stop_rt_speed_locked();
  }
  if (!want_position) {
    stop_rt_servo_locked();
  }
  if (!want_cartesian) {
    stop_rt_servox_locked();
  }
  if (want_position && !start_rt_servo_locked(true)) {
    return false;
  }
  if (want_velocity && !start_rt_speed_locked(true)) {
    return false;
  }
  if (want_cartesian && !start_rt_servox_locked(true)) {
    return false;
  }
  if (want_primitive) {
    moprim_status_code_ = kExecIdle;
    moprim_ready_flag_ = true;
    moprim_wait_start_ = false;
  }
  if (want_native_jt) {
    reset_native_jt();
  }
  return true;
}

bool SymphonyHardwareInterface::start_rt_servo_locked(bool resync_commands)
{
  if (!control_) {
    return false;
  }
  constexpr uint8_t kMotionIdle = 0;
  constexpr uint8_t kMotionMove = 1;
  const uint8_t motion_before = status_ ? status_->getMotionState() : 0xFF;
  if (rt_servo_active_ && motion_before == kMotionMove && !resync_commands) {
    return true;
  }
  rt_servo_active_ = false;
  rt_servox_active_ = false;

  if (status_) {
    const auto actual_joint_angles_deg = status_->getActualJointAngles();
    if (actual_joint_angles_deg.size() >= kNumJoints) {
      for (size_t i = 0; i < kNumJoints; ++i) {
        joint_positions_[i] = robot_deg_to_ros_rad(actual_joint_angles_deg[i]);
      }
    }
  }
  if (resync_commands) {
    if (moprim_opt_servo_relative_ > 0.5) {
      joint_position_commands_.assign(kNumJoints, 0.0);
    } else {
      joint_position_commands_ = joint_positions_;
    }
  }
  if (moprim_opt_servo_relative_ > 0.5) {
    last_servoq_rel_seq_ = servoq_cmd_seq_;
    servoq_rel_primed_ = true;
  } else {
    servoq_rel_primed_ = false;
  }
  const uint8_t opmode = status_ ? status_->getOpMode() : 0;
  if (opmode != 3) {
    RCLCPP_ERROR(
      rclcpp::get_logger("SymphonyHardwareInterface"),
      "startRtServoQControl skipped: robot opmode=%u (need 3=External)",
      opmode);
    return false;
  }

  control_->setSpeed(1.0);
  speed_rate_ = 1.0;
  last_speed_cmd_ = 1.0;

  control_->stopRtSpeedControl();
  control_->stopRtServoControl();
  for (int i = 0; i < 50; ++i) {
    if (status_ && status_->getMotionState() == kMotionIdle) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  const int result = control_->startRtServoQControl(rt_period_s_, rt_servo_filter_);
  if (result != PS_OK) {
    RCLCPP_ERROR(
      rclcpp::get_logger("SymphonyHardwareInterface"),
      "startRtServoQControl failed (code=%d, period=%.4f, filter=%u)",
      result, rt_period_s_, rt_servo_filter_);
    return false;
  }

  std::vector<double> hold_deg(kNumJoints);
  for (size_t i = 0; i < kNumJoints; ++i) {
    hold_deg[i] = ros_rad_to_robot_deg(joint_positions_[i]);
  }
  uint8_t motion = status_ ? status_->getMotionState() : 0xFF;
  for (int i = 0; i < 50; ++i) {
    if (command_) {
      command_->servoQ(hold_deg, false);
    }
    motion = status_ ? status_->getMotionState() : 0xFF;
    if (motion == kMotionMove) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (motion != kMotionMove) {
    RCLCPP_WARN(
      rclcpp::get_logger("SymphonyHardwareInterface"),
      "startRtServoQControl PS_OK but motion_state=%u (MOVE=1, previous=%u, "
      "opmode=%u, period=%.4f, filter=%u). Streaming anyway — some firmware "
      "does not report MOVE while RT_SVOQ is active.",
      motion, motion_before, opmode, rt_period_s_, rt_servo_filter_);
  } else {
    RCLCPP_INFO(
      rclcpp::get_logger("SymphonyHardwareInterface"),
      "startRtServoQControl ok (period=%.4f, filter=%u, opmode=%u, motion_state=%u)",
      rt_period_s_, rt_servo_filter_, opmode, motion);
  }

  const auto t0 = std::chrono::steady_clock::now();
  rt_servo_send_after_ = t0 + std::chrono::duration_cast<
    std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(std::max(0.0, rt_settle_s_)));
  last_rtpri_cmd_send_ = t0;
  last_rtpri_cmd_send_valid_ = true;
  rt_servo_active_ = true;
  return true;
}

bool SymphonyHardwareInterface::start_rt_speed_locked(bool resync_commands)
{
  if (!control_) {
    return false;
  }
  constexpr uint8_t kMotionIdle = 0;
  constexpr uint8_t kMotionMove = 1;
  const uint8_t motion_before = status_ ? status_->getMotionState() : 0xFF;
  if (rt_speed_active_ && motion_before == kMotionMove && !resync_commands) {
    return true;
  }
  rt_speed_active_ = false;
  rt_servo_active_ = false;
  rt_servox_active_ = false;

  const uint8_t opmode = status_ ? status_->getOpMode() : 0;
  if (opmode != 3) {
    RCLCPP_ERROR(
      rclcpp::get_logger("SymphonyHardwareInterface"),
      "startRtSpeedQControl skipped: robot opmode=%u (need 3=External)",
      opmode);
    return false;
  }

  if (resync_commands) {
    joint_velocity_commands_.assign(kNumJoints, 0.0);
  }

  control_->setSpeed(1.0);
  speed_rate_ = 1.0;
  last_speed_cmd_ = 1.0;

  control_->stopRtSpeedControl();
  control_->stopRtServoControl();
  for (int i = 0; i < 50; ++i) {
    if (status_ && status_->getMotionState() == kMotionIdle) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  const int result = control_->startRtSpeedQControl(rt_period_s_, rt_speed_filter_);
  if (result != PS_OK) {
    RCLCPP_ERROR(
      rclcpp::get_logger("SymphonyHardwareInterface"),
      "startRtSpeedQControl failed (code=%d, period=%.4f, filter=%u)",
      result, rt_period_s_, rt_speed_filter_);
    return false;
  }

  uint8_t motion = status_ ? status_->getMotionState() : 0xFF;
  for (int i = 0; i < 50; ++i) {
    if (command_) {
      command_->speedQ(std::vector<double>(kNumJoints, 0.0));
    }
    motion = status_ ? status_->getMotionState() : 0xFF;
    if (motion == kMotionMove) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (motion != kMotionMove) {
    RCLCPP_WARN(
      rclcpp::get_logger("SymphonyHardwareInterface"),
      "startRtSpeedQControl PS_OK but motion_state=%u (MOVE=1, previous=%u, "
      "opmode=%u, period=%.4f, filter=%u). Streaming anyway — some firmware "
      "does not report MOVE while RT_SPDQ is active.",
      motion, motion_before, opmode, rt_period_s_, rt_speed_filter_);
  } else {
    RCLCPP_INFO(
      rclcpp::get_logger("SymphonyHardwareInterface"),
      "startRtSpeedQControl ok (period=%.4f, filter=%u, opmode=%u, motion_state=%u)",
      rt_period_s_, rt_speed_filter_, opmode, motion);
  }

  last_rtpri_cmd_send_ = std::chrono::steady_clock::now();
  last_rtpri_cmd_send_valid_ = true;
  rt_speed_active_ = true;
  return true;
}

bool SymphonyHardwareInterface::start_rt_servox_locked(bool resync_commands)
{
  if (!control_) {
    return false;
  }
  constexpr uint8_t kMotionIdle = 0;
  constexpr uint8_t kMotionMove = 1;
  const uint8_t motion_before = status_ ? status_->getMotionState() : 0xFF;
  if (rt_servox_active_ && motion_before == kMotionMove && !resync_commands) {
    return true;
  }
  rt_servox_active_ = false;
  rt_servo_active_ = false;
  rt_speed_active_ = false;

  if (status_) {
    sync_tcp_sensors_from_robot();
  }
  if (resync_commands) {
    if (moprim_opt_servo_relative_ > 0.5) {
      tcp_pose_cmd_[0] = tcp_pose_cmd_[1] = tcp_pose_cmd_[2] = 0.0;
      tcp_pose_cmd_[3] = tcp_pose_cmd_[4] = tcp_pose_cmd_[5] = 0.0;
      tcp_pose_cmd_[6] = 1.0;
      tcp_pose_seq_cmd_ = 0.0;
    } else {
      for (size_t i = 0; i < 3; ++i) {
        tcp_pose_cmd_[i] = tcp_position_[i];
      }
      for (size_t i = 0; i < 4; ++i) {
        tcp_pose_cmd_[3 + i] = tcp_orientation_[i];
      }
    }
  }
  last_servox_rel_seq_ = tcp_pose_seq_cmd_;
  servox_rel_primed_ = false;
  const uint8_t opmode = status_ ? status_->getOpMode() : 0;
  if (opmode != 3) {
    RCLCPP_ERROR(
      rclcpp::get_logger("SymphonyHardwareInterface"),
      "startRtServoXControl skipped: robot opmode=%u (need 3=External)",
      opmode);
    return false;
  }

  control_->setSpeed(1.0);
  speed_rate_ = 1.0;
  last_speed_cmd_ = 1.0;

  control_->stopRtSpeedControl();
  control_->stopRtServoControl();
  for (int i = 0; i < 50; ++i) {
    if (status_ && status_->getMotionState() == kMotionIdle) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  const int result = control_->startRtServoXControl(rt_period_s_, rt_servo_filter_);
  if (result != PS_OK) {
    RCLCPP_ERROR(
      rclcpp::get_logger("SymphonyHardwareInterface"),
      "startRtServoXControl failed (code=%d, period=%.4f, filter=%u)",
      result, rt_period_s_, rt_servo_filter_);
    return false;
  }

  std::vector<double> hold_pose;
  if (!cartesian_to_rtpri_pose(
        tcp_position_[0], tcp_position_[1], tcp_position_[2],
        tcp_orientation_[0], tcp_orientation_[1], tcp_orientation_[2], tcp_orientation_[3],
        hold_pose))
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("SymphonyHardwareInterface"),
      "startRtServoXControl aborted: current TCP pose is not finite");
    control_->stopRtServoControl();
    return false;
  }
  uint8_t motion = status_ ? status_->getMotionState() : 0xFF;
  for (int i = 0; i < 50; ++i) {
    if (command_) {
      command_->servoX(hold_pose, false);
    }
    motion = status_ ? status_->getMotionState() : 0xFF;
    if (motion == kMotionMove) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (motion != kMotionMove) {
    RCLCPP_WARN(
      rclcpp::get_logger("SymphonyHardwareInterface"),
      "startRtServoXControl PS_OK but motion_state=%u (MOVE=1, previous=%u, "
      "opmode=%u, period=%.4f, filter=%u). Streaming anyway — some firmware "
      "does not report MOVE while RT_SVOX is active.",
      motion, motion_before, opmode, rt_period_s_, rt_servo_filter_);
  } else {
    RCLCPP_INFO(
      rclcpp::get_logger("SymphonyHardwareInterface"),
      "startRtServoXControl ok (period=%.4f, filter=%u, opmode=%u, motion_state=%u)",
      rt_period_s_, rt_servo_filter_, opmode, motion);
  }

  const auto t0 = std::chrono::steady_clock::now();
  rt_servox_send_after_ = t0 + std::chrono::duration_cast<
    std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(std::max(0.0, rt_settle_s_)));
  last_rtpri_cmd_send_ = t0;
  last_rtpri_cmd_send_valid_ = true;
  rt_servox_active_ = true;
  return true;
}

void SymphonyHardwareInterface::stop_rt_servo_locked()
{
  if (!rt_servo_active_.exchange(false)) {
    return;
  }
  if (control_) {
    control_->stopRtServoControl();
  }
}

void SymphonyHardwareInterface::stop_rt_speed_locked()
{
  if (!rt_speed_active_.exchange(false)) {
    return;
  }
  if (command_) {
    command_->speedQ(std::vector<double>(kNumJoints, 0.0));
  }
  if (control_) {
    control_->stopRtSpeedControl();
  }
}

void SymphonyHardwareInterface::stop_rt_servox_locked()
{
  if (!rt_servox_active_.exchange(false)) {
    return;
  }
  if (control_) {
    control_->stopRtServoControl();
  }
}

void SymphonyHardwareInterface::stop_rt_streams()
{
  std::lock_guard<std::mutex> lock(tcp_mutex_);
  stop_rt_speed_locked();
  stop_rt_servo_locked();
  stop_rt_servox_locked();
}

void SymphonyHardwareInterface::request_stop_rt_streams()
{
  const bool was_speed = rt_speed_active_.exchange(false);
  const bool was_servo = rt_servo_active_.exchange(false);
  const bool was_servox = rt_servox_active_.exchange(false);
  if (!was_speed && !was_servo && !was_servox) {
    return;
  }

  if (was_speed && command_) {
    command_->speedQ(std::vector<double>(kNumJoints, 0.0));
  }

  enqueue_control_job([this, was_speed, was_servo, was_servox]() {
      if (!control_) {
        return;
      }
      if (was_speed) {
        control_->stopRtSpeedControl();
      }
      if (was_servo || was_servox) {
        control_->stopRtServoControl();
      }
    });
}

void SymphonyHardwareInterface::trip_command_watchdog()
{
  if (command_watchdog_tripped_) {
    return;
  }
  command_watchdog_tripped_ = true;
  request_stop_rt_streams();
  for (double & v : joint_velocity_commands_) {
    v = 0.0;
  }
  RCLCPP_WARN(
    rclcpp::get_logger("SymphonyHardwareInterface"),
    "command watchdog: RT commands stale for %.3fs — stopped RT stream "
    "(reactivate motion controller or send new setpoints to resume)",
    command_watchdog_timeout_s_);
}

void SymphonyHardwareInterface::update_command_watchdog(const rclcpp::Time & time)
{
  if (command_watchdog_timeout_s_ <= 0.0 || !connected_) {
    return;
  }

  const bool want_position = active_command_modes_.count(kModePosition) != 0;
  const bool want_velocity = active_command_modes_.count(kModeVelocity) != 0;
  const bool want_cartesian = active_command_modes_.count(kModeCartesianPose) != 0;
  if (!want_position && !want_velocity && !want_cartesian) {
    command_watchdog_stamp_valid_ = false;
    command_watchdog_tripped_ = false;
    return;
  }

  if (watchdog_last_position_cmds_.size() != joint_position_commands_.size() ||
    watchdog_last_pose_cmds_.size() != 7)
  {
    watchdog_last_position_cmds_ = joint_position_commands_;
    watchdog_last_velocity_cmds_ = joint_velocity_commands_;
    watchdog_last_pose_cmds_.assign(tcp_pose_cmd_, tcp_pose_cmd_ + 7);
    command_watchdog_stamp_ = time;
    command_watchdog_stamp_valid_ = true;
    return;
  }

  constexpr double kCmdEps = 1e-6;
  constexpr double kHoldEps = 0.01;
  constexpr double kVelEps = 1e-4;
  const bool relative = moprim_opt_servo_relative_ > 0.5;

  bool position_changed = false;
  bool velocity_changed = false;
  bool pose_changed = false;
  double max_abs_vel = 0.0;
  double max_abs_pos_cmd = 0.0;
  double max_tracking_err = 0.0;
  double pose_err = 0.0;
  for (size_t i = 0; i < joint_position_commands_.size(); ++i) {
    if (std::abs(joint_position_commands_[i] - watchdog_last_position_cmds_[i]) > kCmdEps) {
      position_changed = true;
    }
    if (i < joint_velocity_commands_.size() &&
      i < watchdog_last_velocity_cmds_.size() &&
      std::abs(joint_velocity_commands_[i] - watchdog_last_velocity_cmds_[i]) > kCmdEps)
    {
      velocity_changed = true;
    }
    if (i < joint_velocity_commands_.size()) {
      max_abs_vel = std::max(max_abs_vel, std::abs(joint_velocity_commands_[i]));
    }
    max_abs_pos_cmd = std::max(max_abs_pos_cmd, std::abs(joint_position_commands_[i]));
    if (i < joint_positions_.size()) {
      max_tracking_err = std::max(
        max_tracking_err, std::abs(joint_position_commands_[i] - joint_positions_[i]));
    }
  }
  for (size_t i = 0; i < 7; ++i) {
    if (std::abs(tcp_pose_cmd_[i] - watchdog_last_pose_cmds_[i]) > kCmdEps) {
      pose_changed = true;
    }
  }
  {
    const double dx = tcp_pose_cmd_[0] - tcp_position_[0];
    const double dy = tcp_pose_cmd_[1] - tcp_position_[1];
    const double dz = tcp_pose_cmd_[2] - tcp_position_[2];
    pose_err = std::sqrt(dx * dx + dy * dy + dz * dz);
  }
  watchdog_last_position_cmds_ = joint_position_commands_;
  watchdog_last_velocity_cmds_ = joint_velocity_commands_;
  watchdog_last_pose_cmds_.assign(tcp_pose_cmd_, tcp_pose_cmd_ + 7);

  const bool activity = position_changed || velocity_changed || pose_changed;
  const bool at_hold = want_position && !want_velocity && !want_cartesian &&
    (relative ? (max_abs_pos_cmd < kHoldEps) : (max_tracking_err < kHoldEps));
  const double rel_dist = std::sqrt(
    tcp_pose_cmd_[0] * tcp_pose_cmd_[0] +
    tcp_pose_cmd_[1] * tcp_pose_cmd_[1] +
    tcp_pose_cmd_[2] * tcp_pose_cmd_[2]);
  const bool cartesian_hold = want_cartesian && !want_position && !want_velocity &&
    (relative ? (rel_dist < 0.001) : (pose_err < 0.001));
  const bool velocity_idle = want_velocity && max_abs_vel < kVelEps;
  const bool velocity_streaming = want_velocity && max_abs_vel >= kVelEps;
  const bool safe_idle = at_hold || cartesian_hold || velocity_idle || velocity_streaming;

  if (activity || safe_idle) {
    command_watchdog_stamp_ = time;
    command_watchdog_stamp_valid_ = true;
    if (command_watchdog_tripped_ && activity) {
      command_watchdog_tripped_ = false;
      const bool resume_pos = want_position;
      const bool resume_vel = want_velocity;
      const bool resume_cart = want_cartesian;
      enqueue_control_job([this, resume_pos, resume_vel, resume_cart]() {
          if (resume_pos && !start_rt_servo_locked(false)) {
            RCLCPP_WARN(
              rclcpp::get_logger("SymphonyHardwareInterface"),
              "command watchdog: failed to resume RT servoQ after new setpoints");
          }
          if (resume_vel && !start_rt_speed_locked(false)) {
            RCLCPP_WARN(
              rclcpp::get_logger("SymphonyHardwareInterface"),
              "command watchdog: failed to resume RT speedQ after new setpoints");
          }
          if (resume_cart && !start_rt_servox_locked(false)) {
            RCLCPP_WARN(
              rclcpp::get_logger("SymphonyHardwareInterface"),
              "command watchdog: failed to resume RT servoX after new setpoints");
          }
        });
    }
    return;
  }

  if (!command_watchdog_stamp_valid_) {
    command_watchdog_stamp_ = time;
    command_watchdog_stamp_valid_ = true;
    return;
  }

  if (time.get_clock_type() != command_watchdog_stamp_.get_clock_type()) {
    command_watchdog_stamp_ = time;
    return;
  }

  const double age = (time - command_watchdog_stamp_).seconds();
  if (age >= command_watchdog_timeout_s_) {
    trip_command_watchdog();
  }
}

//----- IO / tool modbus -----

void SymphonyHardwareInterface::sync_io_state_from_robot()
{
  if (!status_) {
    return;
  }

  io_safety_di_ = static_cast<double>(status_->getSafetyIoDigitalInputs());
  io_safety_do_ = static_cast<double>(status_->getSafetyIoDigitalOutputs());
  copy_analog(status_->getSafetyIoAnalogInputs(), io_safety_ai_, 2);
  copy_analog(status_->getSafetyIoAnalogOutputs(), io_safety_ao_, 2);

  io_tool_di_ = static_cast<double>(status_->getToolIoInputs());
  io_tool_do_ = static_cast<double>(status_->getToolIoOutputs());
  copy_analog(status_->getToolIoAnalogInputs(), io_tool_ai_, 2);

  io_ex1_di_ = static_cast<double>(status_->getSafetyIoDigitalInputsEx1());
  io_ex1_do_ = static_cast<double>(status_->getSafetyIoDigitalOutputsEx1());
  copy_analog(status_->getSafetyIoAnalogInputsEx1(), io_ex1_ai_, 4);
  copy_analog(status_->getSafetyIoAnalogOutputsEx1(), io_ex1_ao_, 4);

  io_ex2_di_ = static_cast<double>(status_->getSafetyIoDigitalInputsEx2());
  io_ex2_do_ = static_cast<double>(status_->getSafetyIoDigitalOutputsEx2());
  copy_analog(status_->getSafetyIoAnalogInputsEx2(), io_ex2_ai_, 4);
  copy_analog(status_->getSafetyIoAnalogOutputsEx2(), io_ex2_ao_, 4);
}

void SymphonyHardwareInterface::apply_io_commands()
{
  if (!connected_ || !control_) {
    return;
  }

  if (io_digital_seq_ != last_io_digital_seq_ && io_digital_seq_ > 0.0) {
    const int bank = static_cast<int>(io_digital_bank_);
    const unsigned int pin = static_cast<unsigned int>(io_digital_pin_);
    const bool value = io_digital_value_ > 0.5;
    if (pin >= 1 && pin <= 32) {
      const uint32_t mask = 1u << (pin - 1);
      const uint32_t output = value ? mask : 0u;
      switch (bank) {
        case kIoBankSafety:
          enqueue_control_job(
            [this, output, mask]() {control_->setSafetyIoDigitalOutputs(output, mask);});
          break;
        case kIoBankSafetyEx1:
          enqueue_control_job(
            [this, pin, value]() {control_->setSafetyIoDigitalOutputEx1(pin, value);});
          break;
        case kIoBankSafetyEx2:
          enqueue_control_job(
            [this, pin, value]() {control_->setSafetyIoDigitalOutputEx2(pin, value);});
          break;
        case kIoBankTool:
          enqueue_control_job([this, pin, value]() {control_->setToolIoOutput(pin, value);});
          break;
        default:
          RCLCPP_WARN(
            rclcpp::get_logger("SymphonyHardwareInterface"),
            "Unknown IO digital bank %d", bank);
          break;
      }
    }
    last_io_digital_seq_ = io_digital_seq_;
  }

  if (io_analog_seq_ != last_io_analog_seq_ && io_analog_seq_ > 0.0) {
    const int bank = static_cast<int>(io_analog_bank_);
    const unsigned int channel = static_cast<unsigned int>(io_analog_channel_);
    switch (bank) {
      case kIoBankSafety:
        if (channel >= 1 && channel <= 2) {
          auto values = analog_vector(io_safety_ao_, 2);
          values[channel - 1] = io_analog_value_;
          enqueue_control_job(
            [this, values]() {control_->setSafetyIoAnalogOutputs(values);});
        }
        break;
      case kIoBankSafetyEx1:
        if (channel >= 1 && channel <= 4) {
          auto values = analog_vector(io_ex1_ao_, 4);
          values[channel - 1] = io_analog_value_;
          enqueue_control_job(
            [this, values]() {control_->setSafetyIoAnalogOutputsEx1(values);});
        }
        break;
      case kIoBankSafetyEx2:
        if (channel >= 1 && channel <= 4) {
          auto values = analog_vector(io_ex2_ao_, 4);
          values[channel - 1] = io_analog_value_;
          enqueue_control_job(
            [this, values]() {control_->setSafetyIoAnalogOutputsEx2(values);});
        }
        break;
      case kIoBankTool:
        {
          const auto voltage = static_cast<uint8_t>(std::clamp(io_analog_value_, 0.0, 255.0));
          enqueue_control_job(
            [this, voltage]() {control_->setToolIoOutputVoltage(voltage);});
        }
        break;
      default:
        RCLCPP_WARN(
          rclcpp::get_logger("SymphonyHardwareInterface"),
          "Unknown IO analog bank %d", bank);
        break;
    }
    last_io_analog_seq_ = io_analog_seq_;
  }
}

void SymphonyHardwareInterface::apply_tool_modbus_commands()
{
  if (tm_seq_ == last_tm_seq_ || tm_seq_ <= 0.0) {
    return;
  }

  const int op = static_cast<int>(std::lround(tm_op_));
  const auto device = static_cast<uint16_t>(std::clamp(tm_device_, 0.0, 65535.0));
  const auto address = static_cast<uint16_t>(std::clamp(tm_address_, 0.0, 65535.0));
  int size = static_cast<int>(std::lround(tm_size_));
  if (size < 1) {
    size = 1;
  }
  if (size > 8) {
    size = 8;
  }
  std::vector<uint16_t> values(static_cast<size_t>(size), 0);
  for (int i = 0; i < size; ++i) {
    values[static_cast<size_t>(i)] = static_cast<uint16_t>(
      std::clamp(tm_value_[i], 0.0, 65535.0));
  }
  const double seq = tm_seq_;
  last_tm_seq_ = tm_seq_;

  if (!connected_ || !control_) {
    tm_ok_ = 0.0;
    tm_done_seq_ = seq;
    return;
  }

  enqueue_control_job([this, op, device, address, size, values, seq]() {
    bool ok = false;
    std::vector<uint16_t> result;
    switch (op) {
      case kTmAnalogOn:
        control_->setToolIoEnableAnalogInput(true);
        ok = true;
        break;
      case kTmAnalogOff:
        control_->setToolIoEnableAnalogInput(false);
        ok = true;
        break;
      case kTmOpen:
        ok = control_->openToolIoModbus(device);
        break;
      case kTmClose:
        control_->closeToolIoModbus();
        ok = true;
        break;
      case kTmReadBit:
        {
          auto packed = control_->readToolIoModbusBit(address, size);
          ok = std::get<0>(packed);
          result = std::move(std::get<1>(packed));
        }
        break;
      case kTmReadInputBit:
        {
          auto packed = control_->readToolIoModbusInputBit(address, size);
          ok = std::get<0>(packed);
          result = std::move(std::get<1>(packed));
        }
        break;
      case kTmReadReg:
        {
          auto packed = control_->readToolIoModbusRegister(address, size);
          ok = std::get<0>(packed);
          result = std::move(std::get<1>(packed));
        }
        break;
      case kTmWriteBit:
        ok = control_->writeToolIoModbusBit(address, size, values);
        break;
      case kTmWriteReg:
        ok = control_->writeToolIoModbusRegister(address, size, values);
        break;
      case kTmClear:
        {
          const bool sdk_ok = control_->clearToolIoModbusCommand();
          if (sdk_ok) {
            ok = true;
            break;
          }
          ok = false;
          if (status_) {
            for (int i = 0; i < 25; ++i) {
              if (status_->getToolIoModbusCommand() == 0) {
                ok = true;
                break;
              }
              std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
          }
          if (!ok) {
            RCLCPP_WARN(
              rclcpp::get_logger("SymphonyHardwareInterface"),
              "clearToolIoModbusCommand returned false and tool Modbus "
              "command register is not 0");
          }
        }
        break;
      default:
        RCLCPP_WARN(
          rclcpp::get_logger("SymphonyHardwareInterface"),
          "Unknown tool Modbus op %d", op);
        break;
    }
    tm_ok_ = ok ? 1.0 : 0.0;
    for (size_t i = 0; i < 8; ++i) {
      tm_result_[i] = (i < result.size()) ? static_cast<double>(result[i]) : 0.0;
    }
    tm_done_seq_ = seq;
  });
}

//----- force -----

bool SymphonyHardwareInterface::force_mode_allowed() const
{
  auto next = active_command_modes_;
  next.insert(kModeForce);
  return mode_set_compatible(motion_mode_switch_, next);
}

void SymphonyHardwareInterface::apply_force_commands()
{
  if (!connected_ || !control_) {
    return;
  }

  const bool allowed = force_mode_allowed();

  if (force_mode_active_ && !allowed) {
    force_mode_active_ = false;
    enqueue_control_job([this]() {
        control_->setEnableForceControl(false);
      });
    RCLCPP_WARN(
      rclcpp::get_logger("SymphonyHardwareInterface"),
      "Force mode stopped: incompatible with active motion mode");
  }

  if (fp_payload_seq_ != last_fp_payload_seq_ && fp_payload_seq_ > 0.0) {
    // CoG [m] -> [mm]
    std::vector<double> info(7);
    info[0] = fp_payload_[0];
    info[1] = fp_payload_[1] * 1000.0;
    info[2] = fp_payload_[2] * 1000.0;
    info[3] = fp_payload_[3] * 1000.0;
    info[4] = fp_payload_[4];
    info[5] = fp_payload_[5];
    info[6] = fp_payload_[6];
    enqueue_control_job([this, info]() {control_->setToolInfo(info);});
    last_fp_payload_seq_ = fp_payload_seq_;
  }

  if (fp_gravity_seq_ != last_fp_gravity_seq_ && fp_gravity_seq_ > 0.0) {
    const auto gravity = analog_vector(fp_gravity_, 3);
    enqueue_control_job([this, gravity]() {control_->setGravity(gravity);});
    last_fp_gravity_seq_ = fp_gravity_seq_;
  }

  if (fp_bias_seq_ != last_fp_bias_seq_ && fp_bias_seq_ > 0.0) {
    enqueue_control_job([this]() {
        control_->setForceBias();
      });
    last_fp_bias_seq_ = fp_bias_seq_;
  }

  if (fp_param_seq_ != last_fp_param_seq_ && fp_param_seq_ > 0.0) {
    ForceParameters params;
    params.has_mass = fp_set_imp_mass_ > 0.5;
    params.has_stiffness = fp_set_imp_stiffness_ > 0.5;
    params.has_damping = fp_set_imp_damping_ > 0.5;
    params.has_compliance = fp_set_compliance_ > 0.5;
    for (size_t i = 0; i < 6; ++i) {
      params.mass[i] = fp_imp_mass_[i];
      params.stiffness[i] = fp_imp_stiffness_[i];
      params.damping[i] = fp_imp_damping_[i];
      params.compliance[i] = fp_compliance_[i];
    }

    if (params.has_mass) {
      force_parameters_.has_mass = true;
      force_parameters_.mass = params.mass;
    }
    if (params.has_stiffness) {
      force_parameters_.has_stiffness = true;
      force_parameters_.stiffness = params.stiffness;
    }
    if (params.has_damping) {
      force_parameters_.has_damping = true;
      force_parameters_.damping = params.damping;
    }
    if (params.has_compliance) {
      force_parameters_.has_compliance = true;
      force_parameters_.compliance = params.compliance;
    }

    enqueue_control_job([this, params]() {apply_force_parameters(params);});
    last_fp_param_seq_ = fp_param_seq_;
  }

  if (fp_force_seq_ != last_fp_force_seq_ && fp_force_seq_ > 0.0) {
    const bool enable = fp_force_enable_ > 0.5;
    if (enable) {
      if (!allowed) {
        RCLCPP_ERROR(
          rclcpp::get_logger("SymphonyHardwareInterface"),
          "start_force_control rejected: incompatible with active motion mode");
      } else {
        const auto wrench = analog_vector(fp_wrench_, 6);
        const auto params = force_parameters_;
        force_mode_active_ = true;
        enqueue_control_job([this, wrench, params]() {
            apply_force_parameters(params);
            control_->setTcpForceCommand(wrench);
            control_->setEnableForceControl(true);
          });
      }
    } else {
      force_mode_active_ = false;
      enqueue_control_job([this]() {
          control_->setEnableForceControl(false);
        });
    }
    last_fp_force_seq_ = fp_force_seq_;
  }
}

void SymphonyHardwareInterface::apply_force_parameters(const ForceParameters & params)
{
  if (!control_) {
    return;
  }

  const auto as_vector = [](const std::array<double, 6> & values) {
      return std::vector<double>(values.begin(), values.end());
    };

  if (params.has_mass) {
    control_->setImpedanceMass(as_vector(params.mass));
  }
  if (params.has_stiffness) {
    control_->setImpedanceStiffness(as_vector(params.stiffness));
  }
  if (params.has_damping) {
    control_->setImpedanceDamping(as_vector(params.damping));
  }
  if (params.has_compliance) {
    for (unsigned int i = 0; i < 6; ++i) {
      control_->setCompliance(i, params.compliance[i]);
    }
  }
}

//----- motion primitive -----

void SymphonyHardwareInterface::request_moprim_stop()
{
  {
    std::lock_guard<std::mutex> lock(async_queue_mutex_);
    std::queue<AsyncJob> kept;
    while (!async_queue_.empty()) {
      if (async_queue_.front().kind == AsyncJobKind::Control) {
        kept.push(std::move(async_queue_.front()));
      }
      async_queue_.pop();
    }
    async_queue_ = std::move(kept);
    AsyncJob stop;
    stop.kind = AsyncJobKind::Stop;
    async_queue_.push(std::move(stop));
    moprim_status_code_ = kExecStopping;
    moprim_ready_flag_ = false;
    moprim_wait_start_ = false;
  }
  async_cv_.notify_one();
}

void SymphonyHardwareInterface::handle_moprim_commands()
{
  if (moprim_pause_cmd_ > 0.5 && moprim_pause_cmd_ != last_moprim_pause_cmd_) {
    {
      std::lock_guard<std::mutex> lock(async_queue_mutex_);
      AsyncJob job;
      job.kind = AsyncJobKind::Pause;
      async_queue_.push(std::move(job));
    }
    async_cv_.notify_one();
    last_moprim_pause_cmd_ = moprim_pause_cmd_;
  }
  if (moprim_resume_cmd_ > 0.5 && moprim_resume_cmd_ != last_moprim_resume_cmd_) {
    {
      std::lock_guard<std::mutex> lock(async_queue_mutex_);
      AsyncJob job;
      job.kind = AsyncJobKind::Resume;
      async_queue_.push(std::move(job));
    }
    async_cv_.notify_one();
    last_moprim_resume_cmd_ = moprim_resume_cmd_;
  }

  const int type = moprim_type(moprim_commands_);
  if (type < 0) {
    return;
  }

  if (type == kMoprimResetStop) {
    moprim_status_code_ = kExecIdle;
    moprim_ready_flag_ = true;
    moprim_wait_start_ = false;
    fill_nan(moprim_commands_);
    return;
  }

  if (type == kMoprimStopMotion) {
    request_moprim_stop();
    fill_nan(moprim_commands_);
    return;
  }

  if (type == kMoprimSequenceStart || type == kMoprimSequenceEnd) {
    fill_nan(moprim_commands_);
    return;
  }

  AsyncJob job;
  job.kind = AsyncJobKind::Primitive;
  job.cmd = moprim_commands_;
  moprim_ready_flag_ = false;
  {
    std::lock_guard<std::mutex> lock(async_queue_mutex_);
    async_queue_.push(std::move(job));
  }
  async_cv_.notify_one();
  fill_nan(moprim_commands_);
}

void SymphonyHardwareInterface::update_moprim_state()
{
  const uint8_t code = moprim_status_code_.load();
  if (code == kExecStopping && status_) {
    if (status_->done()) {
      moprim_status_code_ = kExecStopped;
    }
  }

  moprim_execution_status_ = static_cast<double>(moprim_status_code_.load());
  moprim_ready_ = moprim_ready_flag_.load() ? 1.0 : 0.0;
}

bool SymphonyHardwareInterface::snap_tcp_pose_cmd_to_measured()
{
  for (size_t i = 0; i < 3; ++i) {
    if (!std::isfinite(tcp_position_[i])) {
      return false;
    }
  }
  for (size_t i = 0; i < 4; ++i) {
    if (!std::isfinite(tcp_orientation_[i])) {
      return false;
    }
  }
  for (size_t i = 0; i < 3; ++i) {
    tcp_pose_cmd_[i] = tcp_position_[i];
  }
  for (size_t i = 0; i < 4; ++i) {
    tcp_pose_cmd_[3 + i] = tcp_orientation_[i];
  }
  return true;
}

bool SymphonyHardwareInterface::cartesian_to_rtpri_pose(
  double x, double y, double z, double qx, double qy, double qz, double qw,
  std::vector<double> & pose_mm_deg) const
{
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
      !std::isfinite(qx) || !std::isfinite(qy) || !std::isfinite(qz) ||
      !std::isfinite(qw))
  {
    return false;
  }
  double roll_deg = 0.0;
  double pitch_deg = 0.0;
  double yaw_deg = 0.0;
  quat_to_rpy_deg(qx, qy, qz, qw, roll_deg, pitch_deg, yaw_deg);
  // XYZ [m] -> [mm]; RPY [deg]
  pose_mm_deg = {x * 1000.0, y * 1000.0, z * 1000.0, roll_deg, pitch_deg, yaw_deg};
  return true;
}

void SymphonyHardwareInterface::execute_moprim_primitive(const std::array<double, 26> & cmd)
{
  const int type = moprim_type(cmd);
  const double speed_in = cmd[22];
  const int blend_tol = blend_tol_mm(cmd[21]);
  const int sticky_tol = static_cast<int>(std::lround(moprim_opt_tol_));
  const int tol = sticky_tol >= 0 ? sticky_tol : blend_tol;
  const auto coord_jnt = static_cast<uint8_t>(rtpri::RTPRIControl::JNT);
  const auto coord_xyz = static_cast<uint8_t>(rtpri::RTPRIControl::XYZ);
  const int sticky_coord = static_cast<int>(std::lround(moprim_opt_coord_));

  const int work = static_cast<int>(std::lround(moprim_opt_work_));
  const int tool = static_cast<int>(std::lround(moprim_opt_tool_));
  const bool rel = moprim_opt_rel_ > 0.5;
  bool fixedspeed = moprim_opt_fixedspeed_ > 0.5;
  const bool weaving = moprim_opt_weaving_ > 0.5;
  const bool fixedorient = moprim_opt_fixedorient_ > 0.5;
  const int ext = static_cast<int>(std::lround(moprim_opt_ext_));
  int cfg = static_cast<int>(std::lround(moprim_opt_cfg_));
  if (std::isfinite(cmd[24])) {
    fixedspeed = cmd[24] > 0.5;
  }
  if (std::isfinite(cmd[25])) {
    cfg = static_cast<int>(std::lround(cmd[25]));
  }

  auto resolve_coord = [&](uint8_t fallback) -> uint8_t {
      if (sticky_coord < 0) {
        return fallback;
      }
      return static_cast<uint8_t>(sticky_coord);
    };

  moprim_wait_start_ = true;
  moprim_status_code_ = kExecExecuting;
  moprim_ready_flag_ = false;

  int result = 0;
  if (type == kMoprimLinearJoint) {
    std::vector<double> joints_deg(kNumJoints);
    for (size_t i = 0; i < kNumJoints; ++i) {
      if (!std::isfinite(cmd[1 + i])) {
        moprim_status_code_ = kExecError;
        moprim_ready_flag_ = true;
        moprim_wait_start_ = false;
        RCLCPP_ERROR(
          rclcpp::get_logger("SymphonyHardwareInterface"),
          "LINEAR_JOINT has NaN joint command");
        return;
      }
      joints_deg[i] = ros_rad_to_robot_deg(cmd[1 + i]);
    }
    const double speed_pct = moprim_speed_percent(speed_in);
    result = control_->movePTP(
      resolve_coord(coord_jnt), joints_deg, speed_pct, work, tool, rel, tol, ext);
  } else if (type == kMoprimLinearCartesian) {
    std::vector<double> pose;
    if (!cartesian_to_rtpri_pose(
          cmd[7], cmd[8], cmd[9], cmd[10], cmd[11], cmd[12], cmd[13], pose))
    {
      moprim_status_code_ = kExecError;
      moprim_ready_flag_ = true;
      moprim_wait_start_ = false;
      RCLCPP_ERROR(
        rclcpp::get_logger("SymphonyHardwareInterface"),
        "LINEAR_CARTESIAN has NaN pose command");
      return;
    }
    if (cfg >= 0) {
      pose.push_back(static_cast<double>(cfg));
    }
    const double speed_arg = fixedspeed
      ? moprim_speed_mm_s(speed_in) : moprim_speed_percent(speed_in);
    result = control_->moveLIN(
      resolve_coord(coord_xyz), pose, speed_arg, work, tool, rel, tol,
      weaving, fixedspeed, fixedorient);
  } else if (type == kMoprimCircularCartesian) {
    std::vector<double> via;
    std::vector<double> goal;
    if (!cartesian_to_rtpri_pose(
          cmd[14], cmd[15], cmd[16], cmd[17], cmd[18], cmd[19], cmd[20], via) ||
        !cartesian_to_rtpri_pose(
          cmd[7], cmd[8], cmd[9], cmd[10], cmd[11], cmd[12], cmd[13], goal))
    {
      moprim_status_code_ = kExecError;
      moprim_ready_flag_ = true;
      moprim_wait_start_ = false;
      RCLCPP_ERROR(
        rclcpp::get_logger("SymphonyHardwareInterface"),
        "CIRCULAR_CARTESIAN needs via and goal poses");
      return;
    }
    if (cfg >= 0) {
      via.push_back(static_cast<double>(cfg));
      goal.push_back(static_cast<double>(cfg));
    }
    const double speed_arg = fixedspeed
      ? moprim_speed_mm_s(speed_in) : moprim_speed_percent(speed_in);
    if (sticky_coord >= 0 && sticky_coord != static_cast<int>(coord_xyz)) {
      moprim_status_code_ = kExecError;
      moprim_ready_flag_ = true;
      moprim_wait_start_ = false;
      RCLCPP_ERROR(
        rclcpp::get_logger("SymphonyHardwareInterface"),
        "moveARC supports XYZ only (coord sticky=%d)", sticky_coord);
      return;
    }
    result = control_->moveARC(
      coord_xyz, std::vector<std::vector<double>>{via, goal},
      speed_arg, work, tool, rel, tol, weaving, fixedspeed, fixedorient);
  } else {
    moprim_status_code_ = kExecError;
    moprim_ready_flag_ = true;
    moprim_wait_start_ = false;
    RCLCPP_ERROR(
      rclcpp::get_logger("SymphonyHardwareInterface"),
      "Unsupported motion primitive type %d", type);
    return;
  }

  if (result != PS_OK) {
    moprim_status_code_ = kExecError;
    moprim_ready_flag_ = true;
    moprim_wait_start_ = false;
    RCLCPP_ERROR(
      rclcpp::get_logger("SymphonyHardwareInterface"),
      "RTPRI move* failed (type=%d, code=%d)", type, result);
    return;
  }

  if (!wait_for_onboard_motion(2000, 60000)) {
    moprim_wait_start_ = false;
    if (moprim_status_code_.load() == kExecStopping) {
      return;
    }
    moprim_status_code_ = kExecError;
    moprim_ready_flag_ = true;
    return;
  }
  moprim_status_code_ = kExecSuccess;
  moprim_ready_flag_ = true;
  moprim_wait_start_ = false;
}

bool SymphonyHardwareInterface::wait_for_onboard_motion(int start_timeout_ms, int done_timeout_ms)
{
  if (!status_) {
    RCLCPP_ERROR(
      rclcpp::get_logger("SymphonyHardwareInterface"),
      "move* has no status connection");
    return false;
  }

  bool issued_stop = false;
  auto interrupt_requested = [this]() {
    return !async_worker_running_.load() ||
      moprim_status_code_.load() == kExecStopping;
  };
  auto issue_stop_once = [this, &issued_stop]() {
    if (issued_stop || !control_) {
      return;
    }
    control_->moveStop();
    issued_stop = true;
  };

  const auto t0 = std::chrono::steady_clock::now();
  int not_done_hits = 0;
  while (async_worker_running_) {
    if (interrupt_requested()) {
      issue_stop_once();
      return false;
    }
    if (!status_->done()) {
      ++not_done_hits;
      if (not_done_hits >= 3) {
        break;
      }
    } else {
      not_done_hits = 0;
    }
    if (std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - t0).count() > start_timeout_ms)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("SymphonyHardwareInterface"),
        "move* accepted (PS_OK) but robot never started — done() stayed true");
      issue_stop_once();
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  if (!async_worker_running_ || interrupt_requested()) {
    issue_stop_once();
    return false;
  }

  const auto t1 = std::chrono::steady_clock::now();
  while (async_worker_running_ && !status_->done()) {
    if (interrupt_requested()) {
      issue_stop_once();
      return false;
    }
    if (std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - t1).count() > done_timeout_ms)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("SymphonyHardwareInterface"),
        "move* started but did not finish — done() stayed false");
      issue_stop_once();
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  if (interrupt_requested()) {
    issue_stop_once();
    return false;
  }
  return async_worker_running_ && status_->done();
}

//----- native joint trajectory -----

void SymphonyHardwareInterface::reset_native_jt()
{
  native_jt_transfer_state_ = kPtIdle;
  native_jt_abort_ = 0.0;
  native_jt_size_ = 0.0;
  native_jt_time_from_start_ = 0.0;
  native_jt_points_.clear();
  native_jt_times_.clear();
  native_jt_expected_points_ = 0;
  native_jt_executing_ = false;
  native_jt_wait_start_ = false;
  native_jt_failed_ = false;
}

void SymphonyHardwareInterface::fail_native_jt(const char * reason)
{
  RCLCPP_ERROR(
    rclcpp::get_logger("SymphonyHardwareInterface"), "native JTC aborted: %s", reason);
  native_jt_failed_ = true;
  native_jt_abort_ = 1.0;
  native_jt_points_.clear();
  native_jt_times_.clear();
  native_jt_expected_points_ = 0;
  native_jt_transfer_state_ = kPtDone;
}

std::vector<double> SymphonyHardwareInterface::native_jt_speed_rates() const
{
  const size_t n = native_jt_points_.size();
  std::vector<double> speeds(n, kNativeJtDefaultSpeed);
  if (max_joint_velocities_.size() != kNumJoints || native_jt_times_.size() != n) {
    return speeds;
  }

  constexpr double kUnknown = -1.0;
  std::vector<double> rates(n, kUnknown);
  for (size_t i = 0; i < n; ++i) {
    const double previous_time = (i == 0) ? 0.0 : native_jt_times_[i - 1];
    const double dt = native_jt_times_[i] - previous_time;
    if (!std::isfinite(dt) || dt < kNativeJtMinSegmentTime) {
      continue;
    }
    double rate = 0.0;
    for (size_t j = 0; j < kNumJoints; ++j) {
      const double from = (i == 0) ? joint_positions_[j] : native_jt_points_[i - 1][j];
      rate = std::max(rate, std::abs(native_jt_points_[i][j] - from) /
        (dt * max_joint_velocities_[j]));
    }
    if (rate > 0.0) {
      rates[i] = rate;
    }
  }

  double last_known = kUnknown;
  for (size_t i = 0; i < n; ++i) {
    if (rates[i] > 0.0) {
      last_known = rates[i];
    } else if (last_known > 0.0) {
      rates[i] = last_known;
    }
  }
  for (size_t i = n; i-- > 0; ) {
    if (rates[i] > 0.0) {
      last_known = rates[i];
    } else if (last_known > 0.0) {
      rates[i] = last_known;
    }
  }

  for (size_t i = 0; i < n; ++i) {
    speeds[i] = (rates[i] > 0.0)
      ? std::clamp(rates[i] * 100.0, kNativeJtMinSpeed, 100.0)
      : kNativeJtDefaultSpeed;
  }
  return speeds;
}

void SymphonyHardwareInterface::handle_native_jt_commands()
{
  if (active_command_modes_.count(kModeNativeJointTrajectory) == 0) {
    return;
  }

  if (native_jt_abort_ > 0.5) {
    if (native_jt_executing_ ||
        std::lround(native_jt_transfer_state_) != static_cast<long>(kPtDone))
    {
      request_moprim_stop();
      native_jt_executing_ = false;
      native_jt_wait_start_ = false;
      native_jt_failed_ = false;
      native_jt_points_.clear();
      native_jt_times_.clear();
      native_jt_expected_points_ = 0;
      native_jt_transfer_state_ = kPtDone;
    }
    return;
  }

  const int state = static_cast<int>(std::lround(native_jt_transfer_state_));
  if (state == static_cast<int>(kPtSize)) {
    const long n = std::lround(native_jt_size_);
    if (n <= 0 || n > static_cast<long>(native_jt_max_points_)) {
      fail_native_jt("announced point count out of range");
      return;
    }
    native_jt_points_.clear();
    native_jt_points_.reserve(static_cast<size_t>(n));
    native_jt_times_.clear();
    native_jt_times_.reserve(static_cast<size_t>(n));
    native_jt_expected_points_ = static_cast<size_t>(n);
    native_jt_failed_ = false;
    native_jt_transfer_state_ = kPtReady;
    return;
  }

  if (state == static_cast<int>(kPtPoint)) {
    if (native_jt_points_.size() >= native_jt_expected_points_) {
      fail_native_jt("received more points than announced");
      return;
    }
    std::array<double, 6> point{};
    for (size_t i = 0; i < kNumJoints; ++i) {
      if (!std::isfinite(native_jt_positions_[i])) {
        fail_native_jt("point contains NaN/Inf");
        return;
      }
      point[i] = native_jt_positions_[i];
    }
    const double time_from_start = native_jt_time_from_start_;
    if (!std::isfinite(time_from_start) || time_from_start < 0.0 ||
      (!native_jt_times_.empty() && time_from_start < native_jt_times_.back()))
    {
      fail_native_jt("time_from_start must be finite and non-decreasing");
      return;
    }
    native_jt_points_.push_back(point);
    native_jt_times_.push_back(time_from_start);
    native_jt_transfer_state_ = kPtReady;
    return;
  }

  if (state != static_cast<int>(kPtGo)) {
    return;
  }

  if (native_jt_points_.size() != native_jt_expected_points_) {
    fail_native_jt("point count does not match the announced size");
    return;
  }

  const auto speeds = native_jt_speed_rates();

  AsyncJob job;
  job.kind = AsyncJobKind::NativeJointTrajectory;
  job.waypoints.reserve(native_jt_points_.size());
  job.speeds = speeds;
  job.blends.reserve(native_jt_points_.size());
  for (size_t i = 0; i < native_jt_points_.size(); ++i) {
    std::vector<double> waypoint(kMaxAxis, 0.0);
    for (size_t j = 0; j < kNumJoints; ++j) {
      waypoint[j] = ros_rad_to_robot_deg(native_jt_points_[i][j]);
    }
    job.waypoints.push_back(std::move(waypoint));
    job.blends.push_back(
      (i + 1 == native_jt_points_.size()) ? kNativeJtBlendLast : native_jt_blend_);
  }

  native_jt_executing_ = true;
  native_jt_wait_start_ = true;
  native_jt_failed_ = false;
  {
    std::lock_guard<std::mutex> lock(async_queue_mutex_);
    async_queue_.push(std::move(job));
  }
  async_cv_.notify_one();
  native_jt_points_.clear();
  native_jt_times_.clear();
  native_jt_expected_points_ = 0;
  native_jt_transfer_state_ = kPtExec;
}

void SymphonyHardwareInterface::update_native_jt_state()
{
  if (native_jt_failed_.exchange(false)) {
    native_jt_abort_ = 1.0;
    native_jt_transfer_state_ = kPtDone;
    native_jt_executing_ = false;
    return;
  }

  if (std::lround(native_jt_transfer_state_) != static_cast<long>(kPtExec) ||
      !native_jt_executing_ || !status_)
  {
    return;
  }

  const bool done = status_->done();
  if (native_jt_wait_start_) {
    if (!done) {
      native_jt_wait_start_ = false;
    }
    return;
  }
  if (done) {
    native_jt_executing_ = false;
    native_jt_transfer_state_ = kPtDone;
  }
}

void SymphonyHardwareInterface::execute_native_jt(
  const std::vector<std::vector<double>> & waypoints_deg,
  const std::vector<double> & speeds,
  const std::vector<double> & blends)
{
  const auto coord_jnt = static_cast<uint8_t>(rtpri::RTPRIControl::JNT);
  const int work = static_cast<int>(std::lround(moprim_opt_work_));
  const int tool = static_cast<int>(std::lround(moprim_opt_tool_));
  const int sticky_coord = static_cast<int>(std::lround(moprim_opt_coord_));
  const int sticky_tol = static_cast<int>(std::lround(moprim_opt_tol_));
  const uint8_t coord = sticky_coord < 0
    ? coord_jnt : static_cast<uint8_t>(sticky_coord);
  const int tol = sticky_tol >= 0 ? sticky_tol : 10;
  const int result = control_->moveCMP(
    coord, waypoints_deg, speeds, blends, work, tool, tol);
  if (result != PS_OK) {
    native_jt_failed_ = true;
    native_jt_executing_ = false;
    native_jt_wait_start_ = false;
    RCLCPP_ERROR(
      rclcpp::get_logger("SymphonyHardwareInterface"),
      "moveCMP failed (code=%d, points=%zu, work=%d tool=%d)",
      result, waypoints_deg.size(), work, tool);
    return;
  }
}

}  // namespace symphony_driver

PLUGINLIB_EXPORT_CLASS(
  symphony_driver::SymphonyHardwareInterface,
  hardware_interface::SystemInterface)
