#include "symphony_controllers/cartesian_pose_controller.hpp"

#include <iterator>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "symphony_controllers/loaned_iface.hpp"

namespace symphony_controllers
{

namespace
{
constexpr const char * kPoseCmd[7] = {
  "tcp_pose/position.x",
  "tcp_pose/position.y",
  "tcp_pose/position.z",
  "tcp_pose/orientation.x",
  "tcp_pose/orientation.y",
  "tcp_pose/orientation.z",
  "tcp_pose/orientation.w",
};
constexpr const char * kPoseSeq = "tcp_pose/seq";
}  // namespace

controller_interface::InterfaceConfiguration
CartesianPoseController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.assign(std::begin(kPoseCmd), std::end(kPoseCmd));
  config.names.push_back(kPoseSeq);
  return config;
}

controller_interface::InterfaceConfiguration
CartesianPoseController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::CallbackReturn CartesianPoseController::on_init()
{
  try {
    auto_declare<std::string>("frame_id", "base");
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "cartesian_pose_controller on_init: %s", ex.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  target_[6].store(1.0);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianPoseController::on_configure(
  const rclcpp_lifecycle::State &)
{
  frame_id_ = get_node()->get_parameter("frame_id").as_string();
  if (frame_id_.empty()) {
    frame_id_ = "base";
  }

  pose_sub_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
    "~/pose", rclcpp::QoS(10),
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      if (!active_.load()) {
        return;
      }
      const uint64_t epoch = accept_epoch_.load();
      const std::string & fid = msg->header.frame_id;
      if (!fid.empty() && fid != frame_id_) {
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 2000,
          "cartesian_pose_controller ignored pose frame_id '%s' (want '%s' or empty)",
          fid.c_str(), frame_id_.c_str());
        return;
      }
      const auto & p = msg->pose.position;
      const auto & q = msg->pose.orientation;
      write_seq_.fetch_add(1);
      target_[0].store(p.x);
      target_[1].store(p.y);
      target_[2].store(p.z);
      target_[3].store(q.x);
      target_[4].store(q.y);
      target_[5].store(q.z);
      target_[6].store(q.w);
      if (active_.load() && accept_epoch_.load() == epoch) {
        pose_seq_.fetch_add(1);
      }
      write_seq_.fetch_add(1);
    });

  return controller_interface::CallbackReturn::SUCCESS;
}

void CartesianPoseController::write_pose(
  const std::array<double, 7> & pose, uint64_t seq)
{
  for (size_t i = 0; i < command_interfaces_.size() && i < pose.size(); ++i) {
    bool wrote = false;
    for (auto & iface : command_interfaces_) {
      if (iface.get_name() == kPoseCmd[i]) {
        set_command(iface, pose[i]);
        wrote = true;
        break;
      }
    }
    (void)wrote;
  }
  for (auto & iface : command_interfaces_) {
    if (iface.get_name() == kPoseSeq) {
      set_command(iface, static_cast<double>(seq));
      break;
    }
  }
}

controller_interface::CallbackReturn CartesianPoseController::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (command_interfaces_.size() != 8) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "cartesian_pose_controller expected 8 command interfaces, got %zu",
      command_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  accept_epoch_.fetch_add(1);
  applied_seq_.store(pose_seq_.load());
  active_.store(true);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianPoseController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  active_.store(false);
  accept_epoch_.fetch_add(1);
  applied_seq_.store(pose_seq_.load());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianPoseController::update(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_.load()) {
    return controller_interface::return_type::OK;
  }
  std::array<double, 7> pose{};
  uint64_t seq = 0;
  bool stable = false;
  for (int attempt = 0; attempt < 3 && !stable; ++attempt) {
    const uint64_t s1 = write_seq_.load();
    if (s1 & 1ULL) {
      return controller_interface::return_type::OK;
    }
    for (size_t i = 0; i < pose.size(); ++i) {
      pose[i] = target_[i].load();
    }
    seq = pose_seq_.load();
    stable = write_seq_.load() == s1;
  }
  if (!stable || seq == applied_seq_.load()) {
    return controller_interface::return_type::OK;
  }
  write_pose(pose, seq);
  if (pose_seq_.load() == seq && (write_seq_.load() & 1ULL) == 0ULL) {
    applied_seq_.store(seq);
  }
  return controller_interface::return_type::OK;
}

}  // namespace symphony_controllers

PLUGINLIB_EXPORT_CLASS(
  symphony_controllers::CartesianPoseController,
  controller_interface::ControllerInterface)
