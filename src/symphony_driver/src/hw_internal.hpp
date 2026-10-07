#ifndef SYMPHONY_DRIVER__HW_INTERNAL_HPP_
#define SYMPHONY_DRIVER__HW_INTERNAL_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include "symphony_driver/hardware_interface.hpp"
#include "symphony_msgs/hw_constants.hpp"

namespace symphony_driver
{

using symphony_msgs::kIoBankSafety;
using symphony_msgs::kIoBankSafetyEx1;
using symphony_msgs::kIoBankSafetyEx2;
using symphony_msgs::kIoBankTool;
using symphony_msgs::kNativeJtMaxPoints;
using symphony_msgs::kPtDone;
using symphony_msgs::kPtExec;
using symphony_msgs::kPtGo;
using symphony_msgs::kPtIdle;
using symphony_msgs::kPtPoint;
using symphony_msgs::kPtReady;
using symphony_msgs::kPtSize;
using symphony_msgs::kTmAnalogOff;
using symphony_msgs::kTmAnalogOn;
using symphony_msgs::kTmClear;
using symphony_msgs::kTmClose;
using symphony_msgs::kTmOpen;
using symphony_msgs::kTmReadBit;
using symphony_msgs::kTmReadInputBit;
using symphony_msgs::kTmReadReg;
using symphony_msgs::kTmWriteBit;
using symphony_msgs::kTmWriteReg;

constexpr double kDegToRad = 0.017453292519943295;
constexpr double kRadToDeg = 57.29577951308232;
constexpr size_t kNumJoints = 6;
constexpr size_t kMaxAxis = 8;

constexpr double kNativeJtBlendLast = 0.0;
constexpr double kNativeJtDefaultSpeed = 50.0;
constexpr double kNativeJtMinSpeed = 1.0;
constexpr double kNativeJtMinSegmentTime = 1e-4;

constexpr size_t kMoprimCmdCount = 26;
constexpr const char * kMoprimCmdNames[kMoprimCmdCount] = {
  "motion_type",
  "q1", "q2", "q3", "q4", "q5", "q6",
  "pos_x", "pos_y", "pos_z", "pos_qx", "pos_qy", "pos_qz", "pos_qw",
  "pos_via_x", "pos_via_y", "pos_via_z",
  "pos_via_qx", "pos_via_qy", "pos_via_qz", "pos_via_qw",
  "blend_radius", "velocity", "acceleration", "move_time", "pose_cfg",
};

constexpr int kMoprimLinearJoint = 0;
constexpr int kMoprimLinearCartesian = 50;
constexpr int kMoprimCircularCartesian = 51;
constexpr int kMoprimStopMotion = 66;
constexpr int kMoprimResetStop = 67;
constexpr int kMoprimSequenceStart = 100;
constexpr int kMoprimSequenceEnd = 101;

constexpr uint8_t kExecIdle = 0;
constexpr uint8_t kExecExecuting = 1;
constexpr uint8_t kExecSuccess = 2;
constexpr uint8_t kExecError = 3;
constexpr uint8_t kExecStopping = 4;
constexpr uint8_t kExecStopped = 5;

inline double nan_val()
{
  return std::numeric_limits<double>::quiet_NaN();
}

inline void fill_nan(std::array<double, kMoprimCmdCount> & cmd)
{
  cmd.fill(nan_val());
}

inline int moprim_type(const std::array<double, kMoprimCmdCount> & cmd)
{
  if (!std::isfinite(cmd[0])) {
    return -1;
  }
  return static_cast<int>(std::lround(cmd[0]));
}

inline double moprim_speed_percent(double velocity)
{
  if (!std::isfinite(velocity) || velocity <= 0.0) {
    return 50.0;
  }
  if (velocity <= 1.0) {
    return velocity * 100.0;
  }
  return std::clamp(velocity, 1.0, 100.0);
}

inline double moprim_speed_mm_s(double velocity_m_s)
{
  if (!std::isfinite(velocity_m_s) || velocity_m_s <= 0.0) {
    return 50.0;
  }
  return velocity_m_s * 1000.0;
}

inline int blend_tol_mm(double blend_radius_m)
{
  if (!std::isfinite(blend_radius_m) || blend_radius_m <= 0.0) {
    return 10;
  }
  return std::max(1, static_cast<int>(std::lround(blend_radius_m * 1000.0)));
}

inline void copy_analog(const std::vector<double> & src, double * dst, size_t n)
{
  for (size_t i = 0; i < n; ++i) {
    dst[i] = (i < src.size()) ? src[i] : 0.0;
  }
}

inline std::vector<double> analog_vector(const double * src, size_t n)
{
  return std::vector<double>(src, src + n);
}

}  // namespace symphony_driver

#endif  // SYMPHONY_DRIVER__HW_INTERNAL_HPP_
