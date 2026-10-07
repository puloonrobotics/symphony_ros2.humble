#include "symphony_driver/motion_mode_switch.hpp"

#include <iterator>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace symphony_driver
{

MotionModeSwitchTable create_motion_mode_switch_table()
{
  MotionModeSwitchTable m;

  auto set_pair = [&m](const char * a, const char * b, bool ok) {
    m[a][b] = ok;
    m[b][a] = ok;
  };

  //----- exclusive writers -----
  set_pair(kModePosition, kModeVelocity, false);
  set_pair(kModePosition, kModeMotionPrimitive, false);
  set_pair(kModePosition, kModeNativeJointTrajectory, false);
  set_pair(kModeVelocity, kModeMotionPrimitive, false);
  set_pair(kModeVelocity, kModeNativeJointTrajectory, false);
  set_pair(kModeMotionPrimitive, kModeNativeJointTrajectory, false);
  set_pair(kModeCartesianPose, kModePosition, false);
  set_pair(kModeCartesianPose, kModeVelocity, false);
  set_pair(kModeCartesianPose, kModeMotionPrimitive, false);
  set_pair(kModeCartesianPose, kModeNativeJointTrajectory, false);

  //----- force overlay (allowed with any writer) -----
  set_pair(kModeForce, kModePosition, true);
  set_pair(kModeForce, kModeVelocity, true);
  set_pair(kModeForce, kModeMotionPrimitive, true);
  set_pair(kModeForce, kModeNativeJointTrajectory, true);
  set_pair(kModeForce, kModeCartesianPose, true);

  return m;
}

std::optional<std::string> command_mode_key(const std::string & interface_name)
{
  const auto slash = interface_name.rfind('/');
  const std::string prefix =
    (slash == std::string::npos) ? std::string{} : interface_name.substr(0, slash);
  const std::string type =
    (slash == std::string::npos) ? interface_name : interface_name.substr(slash + 1);

  if (prefix == kModeMotionPrimitive || type.find("motion_primitive") != std::string::npos) {
    return kModeMotionPrimitive;
  }
  if (prefix == kModeNativeJointTrajectory) {
    return kModeNativeJointTrajectory;
  }
  if (prefix == kModeForce) {
    return kModeForce;
  }
  if (prefix == "tcp_pose") {
    return kModeCartesianPose;
  }
  if (type == hardware_interface::HW_IF_POSITION || type == kModePosition ||
    (interface_name.size() >= 9 &&
    interface_name.compare(interface_name.size() - 9, 9, "/position") == 0))
  {
    return kModePosition;
  }
  if (type == hardware_interface::HW_IF_VELOCITY || type == kModeVelocity ||
    (interface_name.size() >= 9 &&
    interface_name.compare(interface_name.size() - 9, 9, "/velocity") == 0))
  {
    return kModeVelocity;
  }
  return std::nullopt;
}

bool modes_compatible(
  const MotionModeSwitchTable & table,
  const std::string & a,
  const std::string & b)
{
  if (a == b) {
    return true;
  }
  const auto row = table.find(a);
  if (row == table.end()) {
    return true;
  }
  const auto cell = row->second.find(b);
  if (cell == row->second.end()) {
    return true;
  }
  return cell->second;
}

bool mode_set_compatible(
  const MotionModeSwitchTable & table,
  const std::unordered_set<std::string> & modes)
{
  for (auto i = modes.begin(); i != modes.end(); ++i) {
    for (auto j = std::next(i); j != modes.end(); ++j) {
      if (!modes_compatible(table, *i, *j)) {
        return false;
      }
    }
  }
  return true;
}

}  // namespace symphony_driver
