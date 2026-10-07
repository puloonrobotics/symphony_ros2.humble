#ifndef SYMPHONY_DRIVER__HARDWARE_INTERFACE_HPP_
#define SYMPHONY_DRIVER__HARDWARE_INTERFACE_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "puloon_rtpri/rtpriCommand.h"
#include "puloon_rtpri/rtpriControl.h"
#include "puloon_rtpri/rtpriStatus.h"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "symphony_driver/motion_mode_switch.hpp"
#include "symphony_msgs/hw_constants.hpp"

namespace symphony_driver
{

constexpr size_t kArmErrorSlots = 10;

struct ForceParameters
{
  bool has_mass{false};
  bool has_stiffness{false};
  bool has_damping{false};
  bool has_compliance{false};
  std::array<double, 6> mass{};
  std::array<double, 6> stiffness{};
  std::array<double, 6> damping{};
  std::array<double, 6> compliance{};
};

// ROS rad; RTPRI joint API deg. TCP on async_worker_; UDP in read()/write().
class SymphonyHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SymphonyHardwareInterface)

  SymphonyHardwareInterface();
  ~SymphonyHardwareInterface() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & hardware_info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

private:
  bool connect_robot();
  void disconnect_robot();
  bool sync_state_from_robot();
  bool status_sample_plausible(
    unsigned int dof, uint8_t op_mode, uint8_t motion_state, double speed,
    const std::vector<double> & joint_angles_deg) const;
  static double derive_safety_status(uint32_t arm_state);
  void update_arm_errors();
  bool enable_servo();
  bool robot_is_safe_to_command() const;
  bool parse_hardware_parameters();
  void apply_gpio_commands();
  void apply_io_commands();
  void apply_force_commands();
  void apply_tool_modbus_commands();
  void apply_force_parameters(const ForceParameters & params);
  bool force_mode_allowed() const;
  void sync_io_state_from_robot();
  void sync_tcp_sensors_from_robot();
  std::unordered_set<std::string> collect_command_modes(
    const std::vector<std::string> & interface_names) const;
  bool apply_rt_mode(const std::unordered_set<std::string> & modes);
  bool start_rt_servo_locked(bool resync_commands);
  bool start_rt_speed_locked(bool resync_commands);
  bool start_rt_servox_locked(bool resync_commands);
  void stop_rt_servo_locked();
  void stop_rt_speed_locked();
  void stop_rt_servox_locked();
  void stop_rt_streams();
  void request_stop_rt_streams();
  void update_command_watchdog(const rclcpp::Time & time);
  void trip_command_watchdog();
  void start_async_worker();
  void stop_async_worker();
  void enqueue_control_job(std::function<void()> fn);
  void handle_moprim_commands();
  void handle_native_jt_commands();
  void update_moprim_state();
  void update_native_jt_state();
  void fail_native_jt(const char * reason);
  std::vector<double> native_jt_speed_rates() const;
  void request_moprim_stop();
  void async_worker_loop();
  void execute_moprim_primitive(const std::array<double, 26> & cmd);
  bool wait_for_onboard_motion(int start_timeout_ms, int done_timeout_ms);
  void execute_native_jt(
    const std::vector<std::vector<double>> & waypoints_deg,
    const std::vector<double> & speeds,
    const std::vector<double> & blends);
  void reset_native_jt();
  bool cartesian_to_rtpri_pose(
    double x, double y, double z, double qx, double qy, double qz, double qw,
    std::vector<double> & pose_mm_deg) const;
  bool snap_tcp_pose_cmd_to_measured();
  static double robot_deg_to_ros_rad(double robot_deg);
  static double ros_rad_to_robot_deg(double ros_rad);
  static void rpy_deg_to_quat(
    double roll_deg, double pitch_deg, double yaw_deg, double q[4]);
  static void quat_to_rpy_deg(
    double qx, double qy, double qz, double qw,
    double & roll_deg, double & pitch_deg, double & yaw_deg);

  std::shared_ptr<rtpri::RTPRIControl> control_;
  std::shared_ptr<rtpri::RTPRICommand> command_;
  std::shared_ptr<rtpri::RTPRIStatus> status_;

  std::string robot_ip_{"192.168.0.234"};
  double rt_period_s_{0.002};
  uint32_t rt_servo_filter_{10};
  uint32_t rt_speed_filter_{10};
  double initial_speed_{1.0};
  uint16_t rt_client_id_{0};
  double rt_settle_s_{3.0};
  double rt_max_joint_jump_rad_{0.5};
  double rt_max_tcp_jump_m_{0.15};
  std::chrono::steady_clock::time_point last_rtpri_cmd_send_{};
  std::chrono::steady_clock::time_point rt_servo_send_after_{};
  std::chrono::steady_clock::time_point rt_servox_send_after_{};
  bool last_rtpri_cmd_send_valid_{false};
  double command_watchdog_timeout_s_{0.0};

  size_t native_jt_max_points_{symphony_msgs::kNativeJtMaxPoints};
  double native_jt_blend_{10.0};
  std::vector<double> max_joint_velocities_;

  std::vector<double> joint_positions_;
  std::vector<double> joint_velocities_;
  std::vector<double> joint_efforts_;
  std::vector<double> joint_position_commands_;
  std::vector<double> joint_velocity_commands_;

  double speed_rate_{1.0};
  double op_mode_{0.0};
  double safety_status_{0.0};
  double safety_zone_{0.0};
  double arm_state_{0.0};
  double tool_io_status_{0.0};
  double servo_enable_{0.0};
  double arm_errors_[kArmErrorSlots]{};
  std::vector<uint32_t> last_arm_errors_;

  double speed_rate_cmd_{-1.0};
  double servo_enable_cmd_{-1.0};
  double clear_fault_cmd_{0.0};
  double clear_fault_warning_{0.0};
  double last_speed_cmd_{-1.0};
  double last_servo_enable_cmd_{-1.0};
  double last_clear_fault_cmd_{0.0};
  double collision_enable_cmd_{-1.0};
  double last_collision_enable_cmd_{-1.0};
  double rt_period_cmd_{-1.0};
  double rt_filter_cmd_{-1.0};
  double last_rt_period_cmd_{-1.0};
  double last_rt_filter_cmd_{-1.0};
  double pause_rt_cmd_{0.0};
  double resume_rt_cmd_{0.0};
  double last_pause_rt_cmd_{0.0};
  double last_resume_rt_cmd_{0.0};

  double io_safety_di_{0.0};
  double io_safety_do_{0.0};
  double io_safety_ai_[2]{};
  double io_safety_ao_[2]{};
  double io_tool_di_{0.0};
  double io_tool_do_{0.0};
  double io_tool_ai_[2]{};
  double io_ex1_di_{0.0};
  double io_ex1_do_{0.0};
  double io_ex1_ai_[4]{};
  double io_ex1_ao_[4]{};
  double io_ex2_di_{0.0};
  double io_ex2_do_{0.0};
  double io_ex2_ai_[4]{};
  double io_ex2_ao_[4]{};

  double io_digital_seq_{0.0};
  double io_digital_bank_{-1.0};
  double io_digital_pin_{-1.0};
  double io_digital_value_{-1.0};
  double io_analog_seq_{0.0};
  double io_analog_bank_{-1.0};
  double io_analog_channel_{-1.0};
  double io_analog_value_{0.0};
  double last_io_digital_seq_{0.0};
  double last_io_analog_seq_{0.0};

  double tm_seq_{0.0};
  double tm_op_{0.0};
  double tm_device_{0.0};
  double tm_address_{0.0};
  double tm_size_{0.0};
  double tm_value_[8]{};
  double last_tm_seq_{0.0};
  double tm_done_seq_{0.0};
  double tm_ok_{0.0};
  double tm_command_{0.0};
  double tm_echo_address_{0.0};
  double tm_number_{0.0};
  double tm_result_[8]{};

  double force_enabled_{0.0};
  double fp_payload_seq_{0.0};
  double fp_payload_[7]{};
  double fp_gravity_seq_{0.0};
  double fp_gravity_[3]{};
  double fp_bias_seq_{0.0};
  double fp_force_seq_{0.0};
  double fp_force_enable_{0.0};
  double fp_wrench_[6]{};
  double fp_param_seq_{0.0};
  double fp_set_imp_mass_{0.0};
  double fp_imp_mass_[6]{};
  double fp_set_imp_stiffness_{0.0};
  double fp_imp_stiffness_[6]{};
  double fp_set_imp_damping_{0.0};
  double fp_imp_damping_[6]{};
  double fp_set_compliance_{0.0};
  double fp_compliance_[6]{};
  double last_fp_payload_seq_{0.0};
  double last_fp_gravity_seq_{0.0};
  double last_fp_bias_seq_{0.0};
  double last_fp_force_seq_{0.0};
  double last_fp_param_seq_{0.0};
  ForceParameters force_parameters_;
  std::atomic<bool> force_mode_active_{false};

  double tcp_position_[3]{};
  double tcp_orientation_[4]{0.0, 0.0, 0.0, 1.0};
  double tcp_pose_cmd_[7]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  double tcp_pose_seq_cmd_{0.0};
  double servoq_cmd_seq_{0.0};
  double last_servoq_rel_seq_{0.0};
  double last_servox_rel_seq_{0.0};
  bool servoq_rel_primed_{false};
  bool servox_rel_primed_{false};
  double tcp_force_[3]{};
  double tcp_torque_[3]{};

  std::array<double, 26> moprim_commands_{};
  double moprim_pause_cmd_{0.0};
  double moprim_resume_cmd_{0.0};
  double last_moprim_pause_cmd_{0.0};
  double last_moprim_resume_cmd_{0.0};
  double moprim_execution_status_{0.0};
  double moprim_ready_{1.0};

  double moprim_opt_work_{-1.0};
  double moprim_opt_tool_{-1.0};
  double moprim_opt_rel_{0.0};
  double moprim_opt_fixedspeed_{0.0};
  double moprim_opt_cfg_{-1.0};
  double moprim_opt_coord_{-1.0};
  double moprim_opt_weaving_{0.0};
  double moprim_opt_fixedorient_{0.0};
  double moprim_opt_ext_{0.0};
  double moprim_opt_tol_{-1.0};
  double moprim_opt_servo_relative_{0.0};

  enum class AsyncJobKind : uint8_t { Primitive, Stop, Pause, Resume, NativeJointTrajectory, Control };
  struct AsyncJob
  {
    AsyncJobKind kind{AsyncJobKind::Primitive};
    std::array<double, 26> cmd{};
    std::vector<std::vector<double>> waypoints;
    std::vector<double> speeds;
    std::vector<double> blends;
    std::function<void()> fn;
  };
  std::mutex async_queue_mutex_;
  std::condition_variable async_cv_;
  std::queue<AsyncJob> async_queue_;
  std::mutex tcp_mutex_;
  std::thread async_worker_;
  std::atomic<bool> async_worker_running_{false};
  std::atomic<uint8_t> moprim_status_code_{0};
  std::atomic<bool> moprim_ready_flag_{true};
  std::atomic<bool> moprim_wait_start_{false};

  double native_jt_positions_[6]{};
  double native_jt_velocities_[6]{};
  double native_jt_accelerations_[6]{};
  double native_jt_transfer_state_{0.0};
  double native_jt_time_from_start_{0.0};
  double native_jt_abort_{0.0};
  double native_jt_size_{0.0};
  std::vector<std::array<double, 6>> native_jt_points_;
  std::vector<double> native_jt_times_;
  size_t native_jt_expected_points_{0};
  std::atomic<bool> native_jt_executing_{false};
  std::atomic<bool> native_jt_wait_start_{false};
  std::atomic<bool> native_jt_failed_{false};

  bool activated_{false};
  std::atomic<bool> connected_{false};
  std::atomic<bool> status_link_ok_{false};
  std::atomic<bool> rt_servo_active_{false};
  std::atomic<bool> rt_speed_active_{false};
  std::atomic<bool> rt_servox_active_{false};

  MotionModeSwitchTable motion_mode_switch_;
  std::unordered_set<std::string> active_command_modes_;
  std::unordered_set<std::string> pending_command_modes_;

  rclcpp::Time command_watchdog_stamp_{0, 0, RCL_ROS_TIME};
  bool command_watchdog_stamp_valid_{false};
  bool command_watchdog_tripped_{false};
  std::vector<double> watchdog_last_position_cmds_;
  std::vector<double> watchdog_last_velocity_cmds_;
  std::vector<double> watchdog_last_pose_cmds_;
};

}  // namespace symphony_driver

#endif  // SYMPHONY_DRIVER__HARDWARE_INTERFACE_HPP_
