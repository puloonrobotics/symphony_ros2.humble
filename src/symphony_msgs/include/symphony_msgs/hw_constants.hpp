#ifndef SYMPHONY_MSGS__HW_CONSTANTS_HPP_
#define SYMPHONY_MSGS__HW_CONSTANTS_HPP_

#include <cstddef>
#include <cstdint>

namespace symphony_msgs
{

// tool_modbus/op
constexpr int kTmAnalogOn = 1;
constexpr int kTmAnalogOff = 2;
constexpr int kTmOpen = 3;
constexpr int kTmClose = 4;
constexpr int kTmReadBit = 5;
constexpr int kTmReadInputBit = 6;
constexpr int kTmReadReg = 7;
constexpr int kTmWriteBit = 8;
constexpr int kTmWriteReg = 9;
constexpr int kTmClear = 10;

// native_joint_trajectory/transfer_state
constexpr double kPtIdle = 0.0;
constexpr double kPtReady = 1.0;
constexpr double kPtPoint = 2.0;
constexpr double kPtGo = 3.0;
constexpr double kPtExec = 4.0;
constexpr double kPtDone = 5.0;
constexpr double kPtSize = 6.0;

// io/*_bank (0–2 = Set*Output BANK_*)
constexpr uint8_t kIoBankSafety = 0;
constexpr uint8_t kIoBankSafetyEx1 = 1;
constexpr uint8_t kIoBankSafetyEx2 = 2;
constexpr uint8_t kIoBankTool = 3;

// Match URDF / yaml max_points
constexpr size_t kNativeJtMaxPoints = 500;

}  // namespace symphony_msgs

#endif  // SYMPHONY_MSGS__HW_CONSTANTS_HPP_
