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
#include <iostream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

class Location {
public:
  double x;
  double y;
  Location() = default;
  Location(double x_value, double y_value) : x(x_value), y(y_value) {}
};

class Block {
public:
  double width;
  double length;
  double height;
  double radius;
  Location location;
  Block() = default;
  Block(double width_value, double length_value, double height_value, double radius_value, Location location_value)
    : width(width_value), length(length_value), height(height_value), radius(radius_value), location(location_value) {}
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

struct ModelHints {
  double tip_z;
  double carry_z;
  double place_boost;
  double vel_scale;
};

ModelHints model_hints(const std::string & symphony_type) {
  if (symphony_type == "symphony40") {

    return {0.122, 0.70, 0.015, 0.85};
  }
  if (symphony_type == "symphony20") {

    return {0.122, 0.70, 0.015, 0.85};
  }
  if (symphony_type == "symphony15") {
    return {0.122, 0.50, 0.012, 0.85};
  }
  if (symphony_type == "symphony10") {
    return {0.122, 0.48, 0.010, 0.85};
  }
  return {0.122, 0.45, 0.010, 1.0};
}

std::string detect_symphony_type(
  const std::string & hint,
  const moveit::planning_interface::MoveGroupInterface & arm) {
  if (hint != "symphony5" && !hint.empty()) {
    return hint;
  }
  const auto model = arm.getRobotModel();
  if (!model || !model->getURDF()) {
    return hint.empty() ? "symphony5" : hint;
  }
  const auto joint2 = model->getURDF()->getJoint("joint2");
  const auto joint4 = model->getURDF()->getJoint("joint4");
  if (!joint2) {
    return hint.empty() ? "symphony5" : hint;
  }
  const double j2x = joint2->parent_to_joint_origin_transform.position.x;
  const double j4z = joint4 ? joint4->parent_to_joint_origin_transform.position.z : 0.0;
  if (j4z > 0.4) {
    return "symphony15";
  }

  if (j2x > 0.65 && j2x < 0.75) {

    const auto joint3 = model->getURDF()->getJoint("joint3");
    const double j3x = joint3 ? joint3->parent_to_joint_origin_transform.position.x : 0.0;
    if (j3x > 0.6 && j3x < 0.72) {
      return "symphony40";
    }
  }
  if (j2x > 0.75) {
    return "symphony20";
  }
  if (j2x > 0.5) {
    return "symphony10";
  }
  return "symphony5";
}

bool needs_ik_guard(const std::string & symphony_type) {
  return symphony_type == "symphony20";
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
    RCLCPP_WARN(rclcpp::get_logger("arm"), "Stable constrained pose plan failed");
  }
  clear_path_limits(arm);
  return ok;
}

bool follow_cartesian(moveit::planning_interface::MoveGroupInterface& arm,
                      const std::vector<geometry_msgs::msg::Pose>& waypoints,
                      bool allow_pose_fallback = true,
                      bool use_stable_limits = false) {
  if (waypoints.empty()) {
    return false;
  }
  arm.setStartStateToCurrentState();
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double fraction = arm.computeCartesianPath(waypoints, 0.01, 0.0, trajectory, false);
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
  if (arm.computeCartesianPath({mid, goal}, 0.01, mid_traj, false) > 0.95) {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = mid_traj;
    return arm.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
  }

  if (use_stable_limits) {
    return go_pose_stable(arm, goal);
  }
  if (!allow_pose_fallback) {
    RCLCPP_WARN(rclcpp::get_logger("arm"),
                "Cartesian incomplete (%.2f); skip free IK (avoids wild joint flips)", fraction);
    return false;
  }
  arm.setPoseTarget(goal);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (arm.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    return false;
  }
  return arm.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
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

bool move_gripper(moveit::planning_interface::MoveGroupInterface& gripper_interface,
                  double knuckle_value, const char * label) {
  RCLCPP_INFO(rclcpp::get_logger("gripper"), "%s (knuckle=%.3f)", label, knuckle_value);
  for (int attempt = 1; attempt <= 3; ++attempt) {
    gripper_interface.setStartStateToCurrentState();
    if (!gripper_interface.setJointValueTarget("robotiq_85_left_knuckle_joint", knuckle_value)) {
      RCLCPP_WARN(rclcpp::get_logger("gripper"), "%s: setJointValueTarget failed (try %d)", label, attempt);
      rclcpp::sleep_for(std::chrono::milliseconds(200));
      continue;
    }
    const auto code = gripper_interface.move();
    if (code == moveit::core::MoveItErrorCode::SUCCESS) {
      rclcpp::sleep_for(std::chrono::milliseconds(350));
      return true;
    }
    RCLCPP_WARN(rclcpp::get_logger("gripper"), "%s: move failed code=%d (try %d/3)",
                label, code.val, attempt);
    rclcpp::sleep_for(std::chrono::milliseconds(300));
  }
  RCLCPP_ERROR(rclcpp::get_logger("gripper"), "%s: gave up after 3 tries", label);
  return false;
}

void grab_block(moveit::planning_interface::MoveGroupInterface& gripper_interface, double value) {
  move_gripper(gripper_interface, value, "Gripper closing");
}

void release_block(moveit::planning_interface::MoveGroupInterface& gripper_interface) {
  move_gripper(gripper_interface, 0.0, "Gripper opening");
}

bool go_joints(moveit::planning_interface::MoveGroupInterface& arm,
               const std::vector<double> & joints, const char * label) {
  RCLCPP_INFO(rclcpp::get_logger("arm"), "Joint goal: %s", label);
  arm.setStartStateToCurrentState();
  if (!arm.setJointValueTarget(joints)) {
    RCLCPP_WARN(rclcpp::get_logger("arm"), "%s: setJointValueTarget rejected", label);
    return false;
  }
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (arm.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_WARN(rclcpp::get_logger("arm"), "%s: plan failed", label);
    return false;
  }
  if (arm.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_WARN(rclcpp::get_logger("arm"), "%s: execute failed", label);
    return false;
  }
  return true;
}

void initial_pose(moveit::planning_interface::MoveGroupInterface& arm_interface,
                  const std::string & symphony_type) {
  RCLCPP_INFO(rclcpp::get_logger("arm"), "Moving to initial pose (%s)...", symphony_type.c_str());


  const double j5 = (symphony_type == "symphony15") ? 1.5708 : -1.5708;
  std::vector<double> joints = {0.0971, -0.4337, 1.9769, 0.0277, j5, 0.0971};
  const char * label = (symphony_type == "symphony15") ? "ready_j5_plus90" : "ready_j5_minus90";
  if (symphony_type == "symphony20") {
    joints = {0.05, -0.55, 1.35, 0.0, -1.5708, 0.05};
    label = "ready_s20_mild";
  }
  if (!go_joints(arm_interface, joints, label)) {
    RCLCPP_WARN(rclcpp::get_logger("arm"), "Initial ready pose failed");
  }
}

class PickAndPlaceNode : public rclcpp::Node {
public:
  PickAndPlaceNode(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : Node("pick_and_place", node_options) {
    symphony_type_ = this->get_parameter_or<std::string>("symphony_type", "symphony5");
    use_triangle_hold_ = this->get_parameter_or<bool>("use_triangle_hold", true);
    RCLCPP_INFO(this->get_logger(), "Pick-and-Place Node started.");
  }

  void init() {
    auto node_ptr = this->rclcpp::Node::shared_from_this();
    moveit::planning_interface::MoveGroupInterface arm(node_ptr, "manipulator");
    moveit::planning_interface::MoveGroupInterface gripper(node_ptr, "gripper");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    symphony_type_ = detect_symphony_type(symphony_type_, arm);
    if (symphony_type_ == "symphony15") {
      RCLCPP_ERROR(this->get_logger(),
                   "symphony15 uses pnp_15. Run: ros2 run symphony_pnp pnp_15 "
                   "(sim: ros2 launch symphony_pnp pnp_15.launch.py)");
      return;
    }
    hints_ = model_hints(symphony_type_);
    RCLCPP_INFO(this->get_logger(),
                "symphony_type=%s tip_z=%.3f carry_z=%.2f vel=%.2f use_triangle_hold=%s",
                symphony_type_.c_str(), hints_.tip_z, hints_.carry_z, hints_.vel_scale,
                use_triangle_hold_ ? "true" : "false");

    arm.setPlanningTime(10.0);
    arm.setMaxVelocityScalingFactor(hints_.vel_scale);
    arm.setMaxAccelerationScalingFactor(std::max(0.25, hints_.vel_scale));
    gripper.setMaxVelocityScalingFactor(std::min(1.0, hints_.vel_scale + 0.2));
    gripper.setMaxAccelerationScalingFactor(std::min(1.0, hints_.vel_scale + 0.2));
    arm.setGoalPositionTolerance(0.008);

    arm.setGoalOrientationTolerance(needs_ik_guard(symphony_type_) ? 0.04 : 0.08);
    rclcpp::sleep_for(std::chrono::seconds(1));

    moveit_msgs::msg::CollisionObject ground_plane;
    ground_plane.header.frame_id = arm.getPlanningFrame();
    ground_plane.id = "ground_plane";

    shape_msgs::msg::SolidPrimitive plane_primitive;
    plane_primitive.type = plane_primitive.BOX;
    plane_primitive.dimensions = {4.0, 4.0, 0.01};

    geometry_msgs::msg::Pose plane_pose;
    plane_pose.orientation.w = 1.0;
    plane_pose.position.z = -0.005;

    ground_plane.primitives.push_back(plane_primitive);
    ground_plane.primitive_poses.push_back(plane_pose);
    ground_plane.operation = ground_plane.ADD;

    planning_scene_interface.applyCollisionObjects({ground_plane});
    RCLCPP_INFO(this->get_logger(), "Ground plane added to the scene.");

    initial_pose(arm, symphony_type_);

    release_block(gripper);
    rclcpp::sleep_for(std::chrono::milliseconds(800));

    Location target_location[] = {
      Location(-0.15, 0.35), Location(-0.15, 0.25), Location(0.15, 0.25),
      Location(0.05, 0.35), Location(-0.05, 0.25), Location(-0.05, 0.35),
      Location(0.05, 0.25), Location(0.15, 0.35)
    };

    for (auto& l : target_location) {
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

    for (auto& l : block) {
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
  std::string symphony_type_;
  bool use_triangle_hold_{true};
  ModelHints hints_{0.122, 0.45, 0.010, 1.0};

  void move_block(moveit::planning_interface::MoveGroupInterface& arm_interface,
                  moveit::planning_interface::MoveGroupInterface& gripper_interface,
                  Block block, Location location, double value,
                  const std::string & hold = "",
                  double place_yaw = M_PI / 2.0) {

    const double pick_yaw = M_PI / 2.0;
    const double pick_z = block.height + hints_.tip_z;
    const double place_z = std::max(pick_z + hints_.place_boost, 0.185 + hints_.place_boost);
    const double carry_z = hints_.carry_z;
    const bool need_yaw = std::abs(place_yaw - pick_yaw) > 1e-3;

    const double travel_yaw = need_yaw ? place_yaw : pick_yaw;

    geometry_msgs::msg::Pose above_pick = tool_down(block.location.x, block.location.y, carry_z, pick_yaw);
    geometry_msgs::msg::Pose at_pick = tool_down(block.location.x, block.location.y, pick_z, pick_yaw);
    geometry_msgs::msg::Pose above_pick_travel =
      tool_down(block.location.x, block.location.y, carry_z, travel_yaw);
    geometry_msgs::msg::Pose above_place = tool_down(location.x, location.y, carry_z, travel_yaw);
    geometry_msgs::msg::Pose at_place = tool_down(location.x, location.y, place_z, travel_yaw);


    const bool want_hold = !hold.empty();

    const bool pose_fb = !needs_ik_guard(symphony_type_);
    const bool stable = needs_ik_guard(symphony_type_);

    gz_hold("triangle", false);
    gz_hold("box6", false);
    release_block(gripper_interface);
    rclcpp::sleep_for(std::chrono::milliseconds(250));
    follow_cartesian(arm_interface, {above_pick}, pose_fb, stable);
    if (hints_.vel_scale < 0.6) {
      arm_interface.setMaxVelocityScalingFactor(std::max(0.15, hints_.vel_scale * 0.6));
      arm_interface.setMaxAccelerationScalingFactor(std::max(0.15, hints_.vel_scale * 0.6));
    }
    follow_cartesian(arm_interface, {at_pick}, pose_fb, stable);
    arm_interface.setMaxVelocityScalingFactor(hints_.vel_scale);
    arm_interface.setMaxAccelerationScalingFactor(std::max(0.25, hints_.vel_scale));
    release_block(gripper_interface);
    rclcpp::sleep_for(std::chrono::milliseconds(200));
    grab_block(gripper_interface, value);
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    if (want_hold) {
      RCLCPP_INFO(rclcpp::get_logger("arm"), "Hold attach %s", hold.c_str());
      gz_hold(hold, true);
      rclcpp::sleep_for(std::chrono::milliseconds(400));
    }
    follow_cartesian(arm_interface, {above_pick}, pose_fb, stable);
    if (need_yaw) {
      RCLCPP_INFO(rclcpp::get_logger("arm"), "In-place yaw before XY (long-reach safe)");
      follow_cartesian(arm_interface, {above_pick_travel}, pose_fb, stable);
    }

    follow_cartesian(arm_interface, {above_place}, pose_fb, stable);
    if (hints_.vel_scale < 0.6) {
      arm_interface.setMaxVelocityScalingFactor(std::max(0.15, hints_.vel_scale * 0.6));
      arm_interface.setMaxAccelerationScalingFactor(std::max(0.15, hints_.vel_scale * 0.6));
    }
    follow_cartesian(arm_interface, {at_place}, pose_fb, stable);
    arm_interface.setMaxVelocityScalingFactor(hints_.vel_scale);
    arm_interface.setMaxAccelerationScalingFactor(std::max(0.25, hints_.vel_scale));
    if (want_hold) {
      gz_hold(hold, false);
      rclcpp::sleep_for(std::chrono::milliseconds(300));
      RCLCPP_INFO(rclcpp::get_logger("arm"), "Hold detach %s", hold.c_str());
    }
    release_block(gripper_interface);
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    follow_cartesian(arm_interface, {above_place}, pose_fb, stable);
  }
};

int main(int argc, char** argv) {
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
  auto node = std::make_shared<PickAndPlaceNode>(node_options);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });
  node->init();
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
