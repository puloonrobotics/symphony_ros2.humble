

#include <rclcpp/rclcpp.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr double kTipZ = 0.122;
constexpr double kCarryZ = 0.50;
constexpr double kPlaceBoost = 0.012;
constexpr double kPlaceFloor = 0.185;
constexpr double kVelScale = 1.0;

class Location {
public:
  double x{};
  double y{};
  Location() = default;
  Location(double x_value, double y_value) : x(x_value), y(y_value) {}
};

class Block {
public:
  double width{};
  double length{};
  double height{};
  double radius{};
  Location location;
  Block() = default;
  Block(double w, double l, double h, double r, Location loc)
  : width(w), length(l), height(h), radius(r), location(loc) {}
};

geometry_msgs::msg::Pose list_to_pose(double x, double y, double z, double roll, double pitch, double yaw) {
  geometry_msgs::msg::Pose pose;
  tf2::Quaternion orientation;
  orientation.setRPY(roll, pitch, yaw);
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation = tf2::toMsg(orientation);
  return pose;
}

geometry_msgs::msg::Pose tool_down(double x, double y, double z, double yaw) {
  return list_to_pose(x, y, z, M_PI, 0.0, yaw);
}

void clear_path_limits(moveit::planning_interface::MoveGroupInterface & arm) {
  arm.clearPathConstraints();
}

void set_stable_path_limits(moveit::planning_interface::MoveGroupInterface & arm,
                            const geometry_msgs::msg::Quaternion & keep_orientation) {
  moveit_msgs::msg::Constraints path;
  moveit_msgs::msg::OrientationConstraint ocm;
  ocm.header.frame_id = arm.getPlanningFrame();
  ocm.link_name = arm.getEndEffectorLink();
  ocm.orientation = keep_orientation;
  ocm.absolute_x_axis_tolerance = 0.20;
  ocm.absolute_y_axis_tolerance = 0.20;
  ocm.absolute_z_axis_tolerance = 0.35;
  ocm.weight = 1.0;
  path.orientation_constraints.push_back(ocm);

  const auto names = arm.getJointNames();
  const auto vals = arm.getCurrentJointValues();
  for (size_t i = 0; i < names.size() && i < vals.size(); ++i) {
    const std::string & n = names[i];
    double window = 2.5;
    if (n.find("joint0") != std::string::npos) {
      window = 0.9;
    } else if (n.find("joint4") != std::string::npos || n.find("joint5") != std::string::npos) {
      window = 0.7;
    } else if (n.find("joint1") != std::string::npos) {
      window = 1.2;
    }
    moveit_msgs::msg::JointConstraint jc;
    jc.joint_name = n;
    jc.position = vals[i];
    jc.tolerance_above = window;
    jc.tolerance_below = window;
    jc.weight = 1.0;
    path.joint_constraints.push_back(jc);
  }
  arm.setPathConstraints(path);
}

bool go_pose_stable(moveit::planning_interface::MoveGroupInterface & arm,
                    const geometry_msgs::msg::Pose & goal) {
  arm.setStartStateToCurrentState();
  set_stable_path_limits(arm, goal.orientation);
  arm.setPoseTarget(goal);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const bool ok = (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (ok) {
    arm.execute(plan);
  } else {
    RCLCPP_WARN(rclcpp::get_logger("pnp_15"), "Stable constrained pose plan failed");
  }
  clear_path_limits(arm);
  return ok;
}

bool follow_cartesian(moveit::planning_interface::MoveGroupInterface & arm,
                      const std::vector<geometry_msgs::msg::Pose> & waypoints) {
  if (waypoints.empty()) {
    return false;
  }
  arm.setStartStateToCurrentState();
  moveit_msgs::msg::RobotTrajectory trajectory;

  const double fraction = arm.computeCartesianPath(waypoints, 0.025, 0.0, trajectory, false);
  if (fraction > 0.95) {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    return arm.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
  }

  const geometry_msgs::msg::Pose goal = waypoints.back();
  geometry_msgs::msg::Pose start = arm.getCurrentPose().pose;
  if (waypoints.size() >= 2) {
    start = waypoints.front();
  }
  geometry_msgs::msg::Pose mid = goal;
  mid.position.x = 0.5 * (start.position.x + goal.position.x);
  mid.position.y = 0.5 * (start.position.y + goal.position.y);
  mid.position.z = std::max(start.position.z, goal.position.z);
  mid.orientation = goal.orientation;
  arm.setStartStateToCurrentState();
  moveit_msgs::msg::RobotTrajectory mid_traj;
  if (arm.computeCartesianPath({mid, goal}, 0.025, mid_traj, false) > 0.95) {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = mid_traj;
    return arm.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
  }
  return go_pose_stable(arm, goal);
}

void gz_hold(const std::string & model, bool attach) {
  const std::string topic = std::string("/pnp/") + model + (attach ? "/attach" : "/detach");
  const std::string gz_cmd =
    "gz topic -t " + topic + " -m gz.msgs.Empty -p 'unused: true' >/dev/null 2>&1 &";
  const std::string ign_cmd =
    "ign topic -t " + topic + " -m ignition.msgs.Empty -p 'unused: true' >/dev/null 2>&1 &";
  (void)std::system(gz_cmd.c_str());
  (void)std::system(ign_cmd.c_str());
}

bool move_gripper(moveit::planning_interface::MoveGroupInterface & gripper,
                  double knuckle_value, const char * label) {
  RCLCPP_INFO(rclcpp::get_logger("pnp_15"), "%s (knuckle=%.3f)", label, knuckle_value);
  for (int attempt = 1; attempt <= 3; ++attempt) {
    gripper.setStartStateToCurrentState();
    if (!gripper.setJointValueTarget("robotiq_85_left_knuckle_joint", knuckle_value)) {
      rclcpp::sleep_for(std::chrono::milliseconds(200));
      continue;
    }
    if (gripper.move() == moveit::core::MoveItErrorCode::SUCCESS) {
      rclcpp::sleep_for(std::chrono::milliseconds(350));
      return true;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(300));
  }
  return false;
}

bool go_joints(moveit::planning_interface::MoveGroupInterface & arm,
               const std::vector<double> & joints, const char * label) {
  RCLCPP_INFO(rclcpp::get_logger("pnp_15"), "Joint goal: %s", label);
  arm.setStartStateToCurrentState();
  if (!arm.setJointValueTarget(joints)) {
    RCLCPP_WARN(rclcpp::get_logger("pnp_15"), "%s: setJointValueTarget rejected", label);
    return false;
  }
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (arm.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_WARN(rclcpp::get_logger("pnp_15"), "%s: plan failed", label);
    return false;
  }
  return arm.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
}

void initial_pose_15(moveit::planning_interface::MoveGroupInterface & arm) {
  RCLCPP_INFO(rclcpp::get_logger("pnp_15"), "Moving to symphony15 ready (0,0,90,0,90,0)...");
  const std::vector<double> joints = {0.0, 0.0, 1.570796327, 0.0, 1.570796327, 0.0};
  if (!go_joints(arm, joints, "ready_s15_1007")) {
    RCLCPP_WARN(rclcpp::get_logger("pnp_15"), "Initial ready pose failed");
  }
}

void move_block(moveit::planning_interface::MoveGroupInterface & arm,
                moveit::planning_interface::MoveGroupInterface & gripper,
                Block block, Location location, double value,
                const std::string & hold = "",
                double place_yaw = M_PI / 2.0) {
  const double pick_yaw = M_PI / 2.0;
  const double pick_z = block.height + kTipZ;
  const double place_z = std::max(pick_z + kPlaceBoost, kPlaceFloor + kPlaceBoost);
  const bool need_yaw = std::abs(place_yaw - pick_yaw) > 1e-3;
  const double travel_yaw = need_yaw ? place_yaw : pick_yaw;

  geometry_msgs::msg::Pose above_pick = tool_down(block.location.x, block.location.y, kCarryZ, pick_yaw);
  geometry_msgs::msg::Pose at_pick = tool_down(block.location.x, block.location.y, pick_z, pick_yaw);
  geometry_msgs::msg::Pose above_pick_travel =
    tool_down(block.location.x, block.location.y, kCarryZ, travel_yaw);
  geometry_msgs::msg::Pose above_place = tool_down(location.x, location.y, kCarryZ, travel_yaw);
  geometry_msgs::msg::Pose at_place = tool_down(location.x, location.y, place_z, travel_yaw);

  const bool want_hold = !hold.empty();
  gz_hold("triangle", false);
  gz_hold("box6", false);
  move_gripper(gripper, 0.0, "Gripper opening");
  rclcpp::sleep_for(std::chrono::milliseconds(250));
  follow_cartesian(arm, {above_pick});
  follow_cartesian(arm, {at_pick});
  move_gripper(gripper, 0.0, "Gripper opening");
  rclcpp::sleep_for(std::chrono::milliseconds(200));
  move_gripper(gripper, value, "Gripper closing");
  rclcpp::sleep_for(std::chrono::milliseconds(500));
  if (want_hold) {
    RCLCPP_INFO(rclcpp::get_logger("pnp_15"), "Hold attach %s", hold.c_str());
    gz_hold(hold, true);
    rclcpp::sleep_for(std::chrono::milliseconds(400));
  }
  follow_cartesian(arm, {above_pick});
  if (need_yaw) {
    follow_cartesian(arm, {above_pick_travel});
  }
  follow_cartesian(arm, {above_place});
  follow_cartesian(arm, {at_place});
  if (want_hold) {
    gz_hold(hold, false);
    rclcpp::sleep_for(std::chrono::milliseconds(300));
  }
  move_gripper(gripper, 0.0, "Gripper opening");
  rclcpp::sleep_for(std::chrono::milliseconds(500));
  follow_cartesian(arm, {above_place});
}

}

class PickAndPlace15Node : public rclcpp::Node {
public:
  PickAndPlace15Node(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("pick_and_place_15", options) {
    use_triangle_hold_ = this->get_parameter_or<bool>("use_triangle_hold", true);
    RCLCPP_INFO(this->get_logger(), "pnp_15 started (symphony15 only).");
  }

  void init() {
    auto node_ptr = this->shared_from_this();
    moveit::planning_interface::MoveGroupInterface arm(node_ptr, "manipulator");
    moveit::planning_interface::MoveGroupInterface gripper(node_ptr, "gripper");

    RCLCPP_INFO(this->get_logger(),
                "symphony15 tip_z=%.3f carry_z=%.2f vel=%.2f use_triangle_hold=%s",
                kTipZ, kCarryZ, kVelScale, use_triangle_hold_ ? "true" : "false");

    arm.setPlanningTime(10.0);
    arm.setMaxVelocityScalingFactor(kVelScale);
    arm.setMaxAccelerationScalingFactor(kVelScale);
    gripper.setMaxVelocityScalingFactor(1.0);
    gripper.setMaxAccelerationScalingFactor(1.0);
    arm.setGoalPositionTolerance(0.008);
    arm.setGoalOrientationTolerance(0.04);
    rclcpp::sleep_for(std::chrono::seconds(1));


    initial_pose_15(arm);
    move_gripper(gripper, 0.0, "Gripper opening");
    rclcpp::sleep_for(std::chrono::milliseconds(800));

    Location target_location[] = {
      Location(-0.15, 0.35), Location(-0.15, 0.25), Location(0.15, 0.25),
      Location(0.05, 0.35), Location(-0.05, 0.25), Location(-0.05, 0.35),
      Location(0.05, 0.25), Location(0.15, 0.35)
    };
    for (auto & l : target_location) {
      l.y += 0.2;
    }

    Block block[] = {
      Block(0.05, 0.05, 0.062, 0, Location(0.2, 0.1)),
      Block(0.05, 0.05, 0.062, 0, Location(0.2, 0.0)),
      Block(0.05, 0.025, 0.062, 0, Location(0.2, -0.1)),
      Block(0.05, 0.05, 0.082, 0, Location(0.3, 0.1)),
      Block(0.05, 0.05, 0.062, 0, Location(0.3, 0.0)),
      Block(0.05, 0.05, 0.072, 0, Location(0.3, -0.1)),
      Block(0.05, 0.05, 0.090, 0, Location(0.4, 0.1)),
      Block(0.05, 0.05, 0.092, 0, Location(0.4, 0.0)),
    };
    for (auto & l : block) {
      l.location.x += 0.2;
    }

    move_block(arm, gripper, block[1], target_location[1], 0.38);
    move_block(arm, gripper, block[4], target_location[4], 0.38);
    move_block(arm, gripper, block[6], target_location[6], 0.47,
               use_triangle_hold_ ? "triangle" : "", 0.0);
    move_block(arm, gripper, block[2], target_location[2], 0.62, "", 0.0);
    move_block(arm, gripper, block[7], target_location[7], 0.38);
    move_block(arm, gripper, block[3], target_location[3], 0.38);
    move_block(arm, gripper, block[5], target_location[5], 0.38);
    move_block(arm, gripper, block[0], target_location[0], 0.38);
  }

private:
  bool use_triangle_hold_{true};
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.allow_undeclared_parameters(true);
  node_options.parameter_overrides({
    rclcpp::Parameter("use_sim_time", true),
    rclcpp::Parameter(
      "robot_description_kinematics.manipulator.kinematics_solver",
      "kdl_kinematics_plugin/KDLKinematicsPlugin"),
    rclcpp::Parameter(
      "robot_description_kinematics.manipulator.kinematics_solver_search_resolution", 0.005),
    rclcpp::Parameter(
      "robot_description_kinematics.manipulator.kinematics_solver_timeout", 0.5),
    rclcpp::Parameter(
      "robot_description_kinematics.manipulator.kinematics_solver_attempts", 15),
  });
  auto node = std::make_shared<PickAndPlace15Node>(node_options);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });
  node->init();
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
