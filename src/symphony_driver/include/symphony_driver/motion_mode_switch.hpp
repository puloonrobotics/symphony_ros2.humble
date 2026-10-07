#ifndef SYMPHONY_DRIVER__MOTION_MODE_SWITCH_HPP_
#define SYMPHONY_DRIVER__MOTION_MODE_SWITCH_HPP_

#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace symphony_driver
{

// Mode keys. Force overlays; writers exclusive of each other.
inline constexpr const char * kModePosition = "position";
inline constexpr const char * kModeVelocity = "velocity";
inline constexpr const char * kModeMotionPrimitive = "motion_primitive";
inline constexpr const char * kModeNativeJointTrajectory = "native_joint_trajectory";
inline constexpr const char * kModeCartesianPose = "cartesian_pose";
inline constexpr const char * kModeForce = "force_mode";

using MotionModeSwitchTable =
  std::unordered_map<std::string, std::unordered_map<std::string, bool>>;

MotionModeSwitchTable create_motion_mode_switch_table();

std::optional<std::string> command_mode_key(const std::string & interface_name);

bool modes_compatible(
  const MotionModeSwitchTable & table,
  const std::string & a,
  const std::string & b);

bool mode_set_compatible(
  const MotionModeSwitchTable & table,
  const std::unordered_set<std::string> & modes);

}  // namespace symphony_driver

#endif  // SYMPHONY_DRIVER__MOTION_MODE_SWITCH_HPP_
