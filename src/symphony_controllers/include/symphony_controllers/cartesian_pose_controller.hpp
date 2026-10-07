#ifndef SYMPHONY_CONTROLLERS__CARTESIAN_POSE_CONTROLLER_HPP_
#define SYMPHONY_CONTROLLERS__CARTESIAN_POSE_CONTROLLER_HPP_

#include <array>
#include <atomic>
#include <cstdint>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/subscription.hpp"
#include "symphony_controllers/visibility_control.h"

namespace symphony_controllers
{

class CartesianPoseController : public controller_interface::ControllerInterface
{
public:
  SYMPHONY_CONTROLLERS_PUBLIC
  CartesianPoseController() = default;

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
  void write_pose(const std::array<double, 7> & pose, uint64_t seq);

  std::string frame_id_{"base"};
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

  std::array<std::atomic<double>, 7> target_{};
  std::atomic<bool> active_{false};
  std::atomic<uint64_t> accept_epoch_{0};
  std::atomic<uint64_t> write_seq_{0};
  std::atomic<uint64_t> pose_seq_{0};
  std::atomic<uint64_t> applied_seq_{0};
};

}  // namespace symphony_controllers

#endif  // SYMPHONY_CONTROLLERS__CARTESIAN_POSE_CONTROLLER_HPP_
