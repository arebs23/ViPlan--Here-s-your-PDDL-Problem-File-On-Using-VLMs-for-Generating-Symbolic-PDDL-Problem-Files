#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <Eigen/Geometry>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("seven_pose_sequence");

void execute_7_pose_sequence(
  moveit::planning_interface::MoveGroupInterface& move_group,
  moveit::planning_interface::MoveGroupInterface& move_group_gripper,
  const std::vector<std::pair<std::string, geometry_msgs::msg::Pose>>& sequence,
  const std::vector<double>& gripper_open,
  const std::vector<double>& gripper_close)
{
  moveit_msgs::msg::RobotTrajectory traj;

  for (size_t i = 0; i < sequence.size(); ++i) {
    const auto& [label, target_pose] = sequence[i];
    RCLCPP_INFO(LOGGER, "Moving to: %s", label.c_str());

    bool cartesian_success = false;
    if (i > 0) {
      std::vector<geometry_msgs::msg::Pose> waypoints = {sequence[i - 1].second, target_pose};
      double fraction = move_group.computeCartesianPath(waypoints, 0.01, 2.0, traj);
      RCLCPP_INFO(LOGGER, "Cartesian path fraction: %.2f%%", fraction * 100.0);
      if (fraction > 0.8) {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = traj;
        move_group.execute(plan);
        cartesian_success = true;
      }
    }

    if (!cartesian_success) {
      move_group.setPoseTarget(target_pose, "panda_hand");
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        move_group.execute(plan);
      } else {
        RCLCPP_ERROR(LOGGER, "Failed to move to %s", label.c_str());
        return;
      }
    }

    std::this_thread::sleep_for(std::chrono::seconds(2));

    if (label == "pick") {
      // Add a final downward nudge to ensure contact
      geometry_msgs::msg::Pose current_pose = move_group.getCurrentPose("panda_hand").pose;
      geometry_msgs::msg::Pose lowered_pose = current_pose;
      lowered_pose.position.z -=  0.009;  /// Lower by 3mm

      std::vector<geometry_msgs::msg::Pose> nudge_path = {lowered_pose};
      moveit_msgs::msg::RobotTrajectory nudge_traj;
      double fraction = move_group.computeCartesianPath(nudge_path, 0.001, 0.0, nudge_traj);

      if (fraction > 0.9) {
        moveit::planning_interface::MoveGroupInterface::Plan nudge_plan;
        nudge_plan.trajectory_ = nudge_traj;
        move_group.execute(nudge_plan);
        RCLCPP_INFO(LOGGER, "Performed 3mm downward nudge to contact object");
      } else {
        RCLCPP_WARN(LOGGER, "Nudge failed: could not compute full Cartesian path");
      }

      move_group_gripper.setJointValueTarget(gripper_close);
      move_group_gripper.move();
      RCLCPP_INFO(LOGGER, "Gripper closed at pick");
    } else if (label == "place") {
      move_group_gripper.setJointValueTarget(gripper_open);
      move_group_gripper.move();
      RCLCPP_INFO(LOGGER, "Gripper opened at place");
    }
  }

  RCLCPP_INFO(LOGGER, "7-pose pick-and-place sequence complete.");
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("seven_pose_sequence");
  auto gripper_node = rclcpp::Node::make_shared("gripper_node");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  moveit::planning_interface::MoveGroupInterface move_group(node, "panda_arm");
  move_group.setEndEffectorLink("panda_hand");
  move_group.setPlanningTime(15.0);

  moveit::planning_interface::MoveGroupInterface move_group_gripper(gripper_node, "hand");
  move_group_gripper.setMaxVelocityScalingFactor(0.1);
  move_group_gripper.setMaxAccelerationScalingFactor(0.1);

  std::vector<double> gripper_open = {0.035, 0.035};
  std::vector<double> gripper_close = {0.0, 0.0};
  move_group_gripper.setJointValueTarget(gripper_open);
  move_group_gripper.move();
  std::this_thread::sleep_for(std::chrono::seconds(1));

  geometry_msgs::msg::Pose prepick_pose;
  prepick_pose.position.x = 0.535264;
  prepick_pose.position.y = -0.051543;
  prepick_pose.position.z = 0.504939;
  prepick_pose.orientation.x = 0.999280;
  prepick_pose.orientation.y = 0.020395;
  prepick_pose.orientation.z = -0.031096;
  prepick_pose.orientation.w = 0.007522;

  geometry_msgs::msg::Pose pick_pose;
  pick_pose.position.x = 0.535264;
  pick_pose.position.y = -0.051543;
  pick_pose.position.z = 0.425;  // Was 0.420 + 0.01; now set directly
  pick_pose.orientation = prepick_pose.orientation;

  geometry_msgs::msg::Pose postpick_pose;
  postpick_pose.position.x = 0.632575;
  postpick_pose.position.y = -0.040577;
  postpick_pose.position.z = 0.5800;
  postpick_pose.orientation.x = 0.999332;
  postpick_pose.orientation.y = 0.026453;
  postpick_pose.orientation.z = 0.013079;
  postpick_pose.orientation.w = 0.021576;

  geometry_msgs::msg::Pose clearance_pose;
  clearance_pose.position.x = 0.580421;
  clearance_pose.position.y = 0.195515;
  clearance_pose.position.z = 0.651070;
  clearance_pose.orientation.x = 0.968212;
  clearance_pose.orientation.y = 0.241663;
  clearance_pose.orientation.z = 0.042857;
  clearance_pose.orientation.w = 0.048241;


  geometry_msgs::msg::Pose preplace_pose;
  preplace_pose.position.x = 0.615095;
  preplace_pose.position.y = 0.312747;
  preplace_pose.position.z = 0.545830;
  preplace_pose.orientation.x = 0.498944;
  preplace_pose.orientation.y = 0.549175;
  preplace_pose.orientation.z = -0.444454;
  preplace_pose.orientation.w = 0.501918;

 geometry_msgs::msg::Pose place_pose;
  // place_pose.position.x = 0.540928;
  // place_pose.position.y = 0.275853;
  // place_pose.position.z = 0.525571;
  // place_pose.orientation.x = 0.750667;
  // place_pose.orientation.y = 0.236323;
  // place_pose.orientation.z = 0.112109;
  // place_pose.orientation.w = 0.606697;
  place_pose.position.x = 0.615095;
  place_pose.position.y = 0.312747;
  place_pose.position.z = 0.485830;
  place_pose.orientation.x = 0.498944;
  place_pose.orientation.y = 0.549175;
  place_pose.orientation.z = -0.444454;
  place_pose.orientation.w = 0.501918;

  geometry_msgs::msg::Pose postplace_pose;
  postplace_pose.position.x = 0.586622;
  postplace_pose.position.y = 0.017014;
  postplace_pose.position.z = 0.671420;
  postplace_pose.orientation.x = 0.989905;
  postplace_pose.orientation.y = 0.077573;
  postplace_pose.orientation.z = 0.003593;
  postplace_pose.orientation.w = 0.118566;

  std::vector<std::pair<std::string, geometry_msgs::msg::Pose>> banana_sequence = {
    {"prepick", prepick_pose},
    {"pick", pick_pose},
    {"postpick", postpick_pose},
    {"clearance", clearance_pose},
    {"preplace", preplace_pose},
    {"place", place_pose},
    {"postplace", postplace_pose}
  };

  execute_7_pose_sequence(move_group, move_group_gripper, banana_sequence, gripper_open, gripper_close);

  std::this_thread::sleep_for(std::chrono::seconds(2));
  rclcpp::shutdown();
  return 0;
}
