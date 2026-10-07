#include <rclcpp/rclcpp.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

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

void stretch_trajectory_time(moveit_msgs::msg::RobotTrajectory & trajectory, double factor) {
  if (factor <= 1.0) {
    return;
  }
  for (auto & pt : trajectory.joint_trajectory.points) {
    const int64_t ns = static_cast<int64_t>(pt.time_from_start.sec) * 1000000000LL +
      pt.time_from_start.nanosec;
    const int64_t scaled = static_cast<int64_t>(static_cast<double>(ns) * factor);
    pt.time_from_start.sec = static_cast<int32_t>(scaled / 1000000000LL);
    pt.time_from_start.nanosec = static_cast<uint32_t>(scaled % 1000000000LL);
    for (double & v : pt.velocities) {
      v /= factor;
    }
    for (double & a : pt.accelerations) {
      a /= factor * factor;
    }
  }
}

void follow_cartesian(moveit::planning_interface::MoveGroupInterface& arm,
                      const std::vector<geometry_msgs::msg::Pose>& waypoints,
                      double time_scale = 1.0) {
  arm.setStartStateToCurrentState();
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double fraction = arm.computeCartesianPath(waypoints, 0.01, 0.0, trajectory, false);
  if (fraction > 0.95) {
    stretch_trajectory_time(trajectory, time_scale);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    arm.execute(plan);
    return;
  }
  geometry_msgs::msg::Pose goal = waypoints.back();
  arm.setPoseTarget(goal);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
    arm.execute(plan);
  }
}

void go_to_pose_goal(moveit::planning_interface::MoveGroupInterface& move_group_interface,
                     geometry_msgs::msg::Pose& target_pose) {
  move_group_interface.setStartStateToCurrentState();
  move_group_interface.setPoseTarget(target_pose);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit::core::MoveItErrorCode planning_result = move_group_interface.plan(my_plan);
  bool success = (planning_result == moveit::core::MoveItErrorCode::SUCCESS);
  if (success) {
    move_group_interface.execute(my_plan);
  }
}

void gz_hold(const std::string & model, bool attach) {
  const std::string topic = std::string("/pnp/") + model + (attach ? "/attach" : "/detach");
  const std::string cmd =
    "ign topic -t " + topic + " -m ignition.msgs.Empty -p 'unused: true' >/dev/null 2>&1";
  const int rc = std::system(cmd.c_str());
  (void)rc;
}

void grab_block(moveit::planning_interface::MoveGroupInterface& gripper_interface, double value) {
  RCLCPP_INFO(rclcpp::get_logger("gripper"), "Gripper closing...");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  gripper_interface.setJointValueTarget("robotiq_85_left_knuckle_joint", value);
  moveit::core::MoveItErrorCode planning_result = gripper_interface.plan(my_plan);
  bool success = (planning_result == moveit::core::MoveItErrorCode::SUCCESS);
  if (success) {
    gripper_interface.execute(my_plan);
  }
}

void release_block(moveit::planning_interface::MoveGroupInterface& gripper_interface) {
  RCLCPP_INFO(rclcpp::get_logger("gripper"), "Gripper opening...");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  gripper_interface.setJointValueTarget("robotiq_85_left_knuckle_joint", 0.0);
  moveit::core::MoveItErrorCode planning_result = gripper_interface.plan(my_plan);
  bool success = (planning_result == moveit::core::MoveItErrorCode::SUCCESS);
  if (success) {
    gripper_interface.execute(my_plan);
  }
}

void initial_pose(moveit::planning_interface::MoveGroupInterface& arm_interface) {
  RCLCPP_INFO(rclcpp::get_logger("arm"), "Moving to initial pose...");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  std::vector<double> joint_group_positions = {0.0971, -0.4337, 1.9769, 0.0277, -1.5708, 0.0971};
  arm_interface.setJointValueTarget(joint_group_positions);
  moveit::core::MoveItErrorCode planning_result = arm_interface.plan(my_plan);
  bool success = (planning_result == moveit::core::MoveItErrorCode::SUCCESS);
  if (success) {
      arm_interface.execute(my_plan);
  }
}

class PickAndPlaceNode : public rclcpp::Node {
public:
  PickAndPlaceNode(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions()) 
  : Node("pick_and_place", node_options) {
    RCLCPP_INFO(this->get_logger(), "Pick-and-Place Node started.");
  }

  void init() {
    auto node_ptr = this->rclcpp::Node::shared_from_this();
    moveit::planning_interface::MoveGroupInterface arm(node_ptr, "manipulator");
    moveit::planning_interface::MoveGroupInterface gripper(node_ptr, "gripper");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    arm.setPlanningTime(10.0);
    arm.setMaxVelocityScalingFactor(1.0);
    arm.setMaxAccelerationScalingFactor(1.0);
    arm.setGoalPositionTolerance(0.008);
    arm.setGoalOrientationTolerance(0.08);
    gripper.setMaxVelocityScalingFactor(0.5);
    gripper.setMaxAccelerationScalingFactor(0.5);
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

    initial_pose(arm);

    Location target_location[] = {
      Location(-0.15, 0.35), Location(-0.15, 0.25), Location(0.15, 0.25),
      Location(0.05, 0.35), Location(-0.05, 0.25), Location(-0.05, 0.35),
      Location(0.05, 0.25), Location(0.15, 0.35)
    };

    for(auto& l : target_location){
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

    for(auto& l : block){
        l.location.x += 0.2;
    }

    move_block(arm, gripper, block[1], target_location[1], 0.38);
    move_block(arm, gripper, block[4], target_location[4], 0.38);
    move_block(arm, gripper, block[6], target_location[6], 0.47, "triangle", 0.0);
    move_block(arm, gripper, block[2], target_location[2], 0.62, "box6", 0.0);
    move_block(arm, gripper, block[7], target_location[7], 0.38);
    move_block(arm, gripper, block[3], target_location[3], 0.38);
    move_block(arm, gripper, block[5], target_location[5], 0.38);
    move_block(arm, gripper, block[0], target_location[0], 0.38);
  }

private:
  void move_block(moveit::planning_interface::MoveGroupInterface& arm_interface,
                  moveit::planning_interface::MoveGroupInterface& gripper_interface,
                  Block block, Location location, double value,
                  const std::string & hold = "",
                  double place_yaw = M_PI / 2.0) {

    const double pick_yaw = M_PI / 2.0;
    const double pick_z = block.height + 0.122;
    const double place_z = (pick_z + 0.010 > 0.185) ? pick_z + 0.010 : 0.185;
    geometry_msgs::msg::Pose above_pick = tool_down(block.location.x, block.location.y, 0.45, pick_yaw);
    geometry_msgs::msg::Pose at_pick = tool_down(block.location.x, block.location.y, pick_z, pick_yaw);
    geometry_msgs::msg::Pose above_carry = tool_down(location.x, location.y, 0.45, pick_yaw);
    geometry_msgs::msg::Pose above_place = tool_down(location.x, location.y, 0.45, place_yaw);
    geometry_msgs::msg::Pose at_place = tool_down(location.x, location.y, place_z, place_yaw);

    release_block(gripper_interface);
    follow_cartesian(arm_interface, {above_pick});
    follow_cartesian(arm_interface, {at_pick});
    grab_block(gripper_interface, value);
    rclcpp::sleep_for(std::chrono::milliseconds(200));
    const bool spin = !hold.empty() && std::abs(place_yaw - pick_yaw) > 1e-3;
    if (spin) {
      RCLCPP_INFO(rclcpp::get_logger("arm"), "Yaw hold on %s", hold.c_str());
      gz_hold(hold, true);
      rclcpp::sleep_for(std::chrono::milliseconds(50));
    }
    follow_cartesian(arm_interface, {above_pick, above_carry});
    follow_cartesian(arm_interface, {above_place});
    follow_cartesian(arm_interface, {at_place});
    if (spin) {
      gz_hold(hold, false);
      rclcpp::sleep_for(std::chrono::milliseconds(50));
      RCLCPP_INFO(rclcpp::get_logger("arm"), "Yaw hold off %s", hold.c_str());
    }
    release_block(gripper_interface);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    follow_cartesian(arm_interface, {above_place});
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