
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>

// #include <moveit_msgs/msg/display_robot_state.hpp>
// #include <moveit_msgs/msg/display_trajectory.hpp>

// #include "rclcpp/rclcpp.hpp"
// #include "control_msgs/action/gripper_command.hpp" // Adjust this include based on the actual location
// #include "rclcpp_action/rclcpp_action.hpp"

// #include <rclcpp/rclcpp.hpp>
// #include <thread>
// #include <rclcpp/duration.hpp>

// static const rclcpp::Logger LOGGER = rclcpp::get_logger("test_trajectory");

// int main(int argc, char **argv) {
//   rclcpp::init(argc, argv);
//   rclcpp::NodeOptions node_options;
//   node_options.automatically_declare_parameters_from_overrides(true);
//   auto move_group_node = rclcpp::Node::make_shared("test_trajectory", node_options);
//   auto gripper_client_node = rclcpp::Node::make_shared("gripper_client_node", node_options);
  
//   rclcpp::executors::SingleThreadedExecutor executor;
//   executor.add_node(move_group_node);
//   std::thread([&executor]() { executor.spin(); }).detach();

//   static const std::string PLANNING_GROUP = "panda_arm";
//   static const std::string PLANNING_GROUP_GRIPPER = "hand";

//   moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
//   moveit::planning_interface::MoveGroupInterface move_group_gripper(gripper_client_node, "hand");

//   const moveit::core::JointModelGroup *joint_model_group =
//     move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

//   RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());

//   RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

//   // Sending goal to gripper action server


//   RCLCPP_INFO(LOGGER, "Available Planning Groups:");
//   std::copy(move_group.getJointModelGroupNames().begin(),
//             move_group.getJointModelGroupNames().end(),
//             std::ostream_iterator<std::string>(std::cout, ", "));

//   moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);
//   moveit::core::RobotStatePtr current_state_gripper = move_group_gripper.getCurrentState(10);

//   std::vector<double> joint_group_positions;
//   std::vector<double> joint_group_positions_gripper;
//   current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
//   current_state->copyJointGroupPositions(joint_model_group, joint_group_positions_gripper);


//   RCLCPP_INFO(LOGGER, "Pregrasp Position");
//   geometry_msgs::msg::Pose target_pose1;
//   // target_pose1.orientation.x = 1.;

//   // BANANA
//   // target_pose1.orientation.x =  1.0;
//   // target_pose1.orientation.y =  0.0;
//   // target_pose1.orientation.z = -0.0;
//   // target_pose1.orientation.w = 0.0;
  

//   // target_pose1.position.x = 0.521;
//   // target_pose1.position.y = -0.075;
//   // target_pose1.position.z = 0.50;


// //APPLE
//   target_pose1.orientation.x =  0.995829;
//   target_pose1.orientation.y =  -0.09123;
//   target_pose1.orientation.z = 0.0;
//   target_pose1.orientation.w = 0.0;

//   target_pose1.position.x =  0.68746;
//   target_pose1.position.y = -0.15068;
//   target_pose1.position.z =  0.43496;
//   // // Approach



//   RCLCPP_INFO(LOGGER, "Approach to object!");
//   std::vector<geometry_msgs::msg::Pose> approach_waypoints;
//   target_pose1.position.z = 0.53;
//   approach_waypoints.push_back(target_pose1);

//   moveit_msgs::msg::RobotTrajectory trajectory_approach;
//   const double jump_threshold = 0.0;
//   const double eef_step = 0.01;

//   double fraction = move_group.computeCartesianPath(approach_waypoints, eef_step, jump_threshold, trajectory_approach);
//   move_group.execute(trajectory_approach);

//     // Wait for some time to ensure the action has been sent
      
//   moveit::planning_interface::MoveGroupInterface::Plan my_plan;

//   move_group.setPlannerId("RRTStarkConfigDefault");

//   move_group.setPoseTarget(target_pose1, "panda_hand");
//   bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

//     if (success) {
//       RCLCPP_INFO(LOGGER, "Planning successful, executing...");
//       move_group.execute(my_plan);
//     } else {
//       RCLCPP_ERROR(LOGGER, "Planning failed!");
//         rclcpp::shutdown();
//         return 0;
//     }
//   std::this_thread::sleep_for(std::chrono::seconds(1));


//   target_pose1.position.z =  0.43;
  
//   moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
//   move_group.setPlannerId("RRTConnectkConfigDefault");

//   move_group.setPoseTarget(target_pose1, "panda_hand");
//   move_group.setPlanningTime(20);
//   success = (move_group.plan(my_plan2) == moveit::core::MoveItErrorCode::SUCCESS);

//     if (success) {
//       RCLCPP_INFO(LOGGER, "Planning successful, executing...");
//       move_group.execute(my_plan2);
//     } else {
//       RCLCPP_ERROR(LOGGER, "Planning failed!");
//         rclcpp::shutdown();
//         return 0;
//     }
//   std::this_thread::sleep_for(std::chrono::seconds(1));

// //   move_group.setMaxVelocityScalingFactor(0.18);
// //   std::this_thread::sleep_for(std::chrono::seconds(1));
// //       // Define the joint constraint
// //   // moveit_msgs::msg::JointConstraint joint_constraint;
// //   // joint_constraint.joint_name = "panda_joint5"; // Elbow joint
// //   // joint_constraint.position = -0.0; // Up elbow position in radians
// //   // joint_constraint.tolerance_above = 0.5;
// //   // joint_constraint.tolerance_below = 0.5;
// //   // joint_constraint.weight = 0.8;

// //   // moveit_msgs::msg::Constraints path_constraints;
// //   // path_constraints.joint_constraints.push_back(joint_constraint);
// //   // move_group.setPathConstraints(path_constraints);

// //   // moveit::planning_interface::MoveGroupInterface::Plan my_plan;

// //   // bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

// //   // if (success) {
// //   //   RCLCPP_INFO(LOGGER, "Planning successful, executing...");
// //   //   move_group.execute(my_plan);
// //   // } else {
// //   //   RCLCPP_ERROR(LOGGER, "Planning failed!");
// //   //     rclcpp::shutdown();
// //   //     return 0;
// //   // }

//  // Create the move group interface for the panda hand (gripper)
      

//   // Define a target width for the gripper to close to (in meters)
//   std::vector<double> gripper_target = {0.03, 0.03}; // Fully closed

//   // Set the gripper target positions
//   move_group_gripper.setJointValueTarget(gripper_target);
//   move_group_gripper.setMaxVelocityScalingFactor(0.1);
//   move_group_gripper.setMaxAccelerationScalingFactor(0.1);

//   // Execute the motion
//   bool successGripper = (move_group_gripper.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);


//   if (successGripper)
//   {
//       RCLCPP_INFO(LOGGER, "Gripper closed successfully.");
//   }
//   else
//   {
//       RCLCPP_ERROR(LOGGER, "Failed to close the gripper.");
//   }


//   RCLCPP_INFO(LOGGER, "Retreat from object!");

//   target_pose1.position.z =  0.5;
  
//   moveit::planning_interface::MoveGroupInterface::Plan my_plan3;
//   move_group.setPlannerId("RRTConnectkConfigDefault");

//   move_group.setPoseTarget(target_pose1, "panda_hand");
//   move_group.setPlanningTime(20);
//   success = (move_group.plan(my_plan3) == moveit::core::MoveItErrorCode::SUCCESS);

//     if (success) {
//       RCLCPP_INFO(LOGGER, "Planning successful, executing...");
//       move_group.execute(my_plan3);
//     } else {
//       RCLCPP_ERROR(LOGGER, "Planning failed!");
//         rclcpp::shutdown();
//         return 0;
//     }
//   std::this_thread::sleep_for(std::chrono::seconds(1));


//    RCLCPP_INFO(LOGGER, "Going to goal");
//   geometry_msgs::msg::Pose target_pose2;
//   // target_pose1.orientation.x = 0.8819;
//   // target_pose1.orientation.y = -0.47126;
//   // target_pose1.orientation.z = -0.0;
//   // target_pose1.orientation.w = -0.0;

// // PLATE POSE
//   target_pose2.orientation.x =  1.0;
//   target_pose2.orientation.y =  0.0;
//   target_pose2.orientation.z = -0.0;
//   target_pose2.orientation.w = 0.0;

//   target_pose2.position.x =  0.37599;
//   target_pose2.position.y = 0.22709;
//   target_pose2.position.z =  0.5;

// // BOWL POSE

//   // target_pose2.orientation.x =  1.0;
//   // target_pose2.orientation.y =  0.0;
//   // target_pose2.orientation.z = -0.0;
//   // target_pose2.orientation.w = 0.0;

//   // target_pose2.position.x =  0.624;
//   // target_pose2.position.y = 0.506;
//   // target_pose2.position.z =  0.5;
  
//   // Approach
//   //RCLCPP_INFO(LOGGER, "Approach to object!");
//   //std::vector<geometry_msgs::msg::Pose> approach_waypoints;
//   //target_pose1.position.z = 0.53;
//   //approach_waypoints.push_back(target_pose1);

//   //moveit_msgs::msg::RobotTrajectory trajectory_approach;
//   //const double jump_threshold = 0.0;
//   //const double eef_step = 0.01;

//   //double fraction = move_group.computeCartesianPath(approach_waypoints, eef_step, jump_threshold, trajectory_approach);
//   //move_group.execute(trajectory_approach);

//     // Wait for some time to ensure the action has been sent
      
//   moveit::planning_interface::MoveGroupInterface::Plan my_plan4;

//   move_group.setPlannerId("RRTStarkConfigDefault");

//   move_group.setPoseTarget(target_pose2, "panda_hand");
//   success = (move_group.plan(my_plan4) == moveit::core::MoveItErrorCode::SUCCESS);

//     if (success) {
//       RCLCPP_INFO(LOGGER, "Planning successful, executing...");
//       move_group.execute(my_plan4);
//     } else {
//       RCLCPP_ERROR(LOGGER, "Planning failed!");
//         rclcpp::shutdown();
//         return 0;
//     }
//   std::this_thread::sleep_for(std::chrono::seconds(1));

// RCLCPP_INFO(LOGGER, "Opening Gripper!");

//   // Define a target width for the gripper to close to (in meters)
//   gripper_target = {0.035, 0.035}; // Fully open

//   // Set the gripper target positions
//   move_group_gripper.setJointValueTarget(gripper_target);
//   move_group_gripper.setMaxVelocityScalingFactor(0.1);
//   move_group_gripper.setMaxAccelerationScalingFactor(0.1);

//   // Execute the motion
//   successGripper = (move_group_gripper.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);


//   if (successGripper)
//   {
//       RCLCPP_INFO(LOGGER, "Gripper closed successfully.");
//   }
//   else
//   {
//       RCLCPP_ERROR(LOGGER, "Failed to close the gripper.");
//   }




// //  moveit::planning_interface::MoveGroupInterface::Plan my_plan5;
// //   move_group.setPlannerId("RRTConnectkConfigDefault");

// //   move_group.setPoseTarget(target_pose1, "panda_hand");
// //   move_group.setPlanningTime(20);
// //   success = (move_group.plan(my_plan5) == moveit::core::MoveItErrorCode::SUCCESS);

// //     if (success) {
// //       RCLCPP_INFO(LOGGER, "Planning successful, executing...");
// //       move_group.execute(my_plan5);
// //     } else {
// //       RCLCPP_ERROR(LOGGER, "Planning failed!");
// //         rclcpp::shutdown();
// //         return 0;
// //     }
// //   std::this_thread::sleep_for(std::chrono::seconds(1));


// //    RCLCPP_INFO(LOGGER, "Going to home");
// //   geometry_msgs::msg::Pose target_pose3;
// //   // target_pose1.orientation.x = 0.8819;
// //   // target_pose1.orientation.y = -0.47126;
// //   // target_pose1.orientation.z = -0.0;
// //   // target_pose1.orientation.w = -0.0;

// // // PLATE POSE
// //   target_pose3.orientation.x =  0.999991;
// //   target_pose3.orientation.y =  0.002595;
// //   target_pose3.orientation.z = 0.002977;
// //   target_pose3.orientation.w = 0.001587;

// //   target_pose3.position.x =  0.307615;
// //   target_pose3.position.y = -0.000820;
// //   target_pose3.position.z =  0.591031;

// // std::vector<geometry_msgs::msg::Pose> retreat_waypoints;
// // target_pose1.position.z += 0.02;  // Adjust upwards by 2 cm
// // retreat_waypoints.push_back(target_pose1);

// // target_pose1.position.z += 0.02;  // Another 2 cm retreat
// // retreat_waypoints.push_back(target_pose1);

// // // // Clear constraints to avoid joint/path interference
// // // move_group.clearPathConstraints();

// // // // Optionally set a different planner
// // move_group.setPlannerId("RRTConnectkConfigDefault");


// // moveit_msgs::msg::RobotTrajectory trajectory_retreat;
// // // double eef_step = 0.01;  // Step size (small increments)
// // // double jump_threshold = 0.0;  // No sudden jumps

// // fraction = move_group.computeCartesianPath(retreat_waypoints, eef_step, jump_threshold, trajectory_retreat);
// // move_group.execute(trajectory_retreat);





//   // geometry_msgs::msg::Pose target_pose2;
//   // target_pose2.orientation.x = 0.999952;
//   // target_pose2.orientation.y = -0.009361;
//   // target_pose2.orientation.z = -0.002807;
//   // target_pose2.orientation.w =   0.000262;
//   // target_pose2.position.x =  0.612053;
//   // target_pose2.position.y =  -0.011588;
//   // target_pose2.position.z =  0.555449;
//   // move_group.setPlannerId("RRTstarkConfigDefault");

//   // move_group.setPoseTarget(target_pose2, "panda_hand");

//   // moveit::planning_interface::MoveGroupInterface::Plan my_plan2;

//   // success = (move_group.plan(my_plan2) == moveit::core::MoveItErrorCode::SUCCESS);

//   // if (success) {
//   //   RCLCPP_INFO(LOGGER, "Planning successful, executing...");
//   //   move_group.execute(my_plan2);
//   // } else {
//   //   RCLCPP_ERROR(LOGGER, "Planning failed!");
//   //     rclcpp::shutdown();
//   // return 0;
//   // }

 
//   // RCLCPP_INFO(LOGGER, "Rotate Object");
//   // geometry_msgs::msg::Pose target_pose3;
//   // target_pose3.orientation.x = 0.996606;
//   // target_pose3.orientation.y = -0.082204;
//   // target_pose3.orientation.z = -0.000793;
//   // target_pose3.orientation.w =  0.005379;
//   // target_pose3.position.x =  0.530259;
//   // target_pose3.position.y =  0.428577;
//   // target_pose3.position.z =  0.528329;
//   // move_group.setPlannerId("RRTstarkConfigDefault");

//   // move_group.setPoseTarget(target_pose3, "panda_hand");

//   // moveit::planning_interface::MoveGroupInterface::Plan my_plan3;

//   // success = (move_group.plan(my_plan3) == moveit::core::MoveItErrorCode::SUCCESS);

//   // if (success) {
//   //   RCLCPP_INFO(LOGGER, "Planning successful, executing...");
//   //   move_group.execute(my_plan3);
//   // } else {
//   //   RCLCPP_ERROR(LOGGER, "Planning failed!");
//   // }  
//   // RCLCPP_INFO(LOGGER, "Place Object");
//   // geometry_msgs::msg::Pose target_pose4;
//   // target_pose4.orientation.x = 0.999972;
//   // target_pose4.orientation.y = -0.005967;
//   // target_pose4.orientation.z = -0.004356;
//   // target_pose4.orientation.w =  -0.001250;
//   // target_pose4.position.x =  0.420679;
//   // target_pose4.position.y =   0.115927;
//   // target_pose4.position.z =  0.465778;

//   // move_group.setPlannerId("RRTstarkConfigDefault");

//   // move_group.setPoseTarget(target_pose4, "panda_hand");

//   // moveit::planning_interface::MoveGroupInterface::Plan my_plan4;

//   // success = (move_group.plan(my_plan4) == moveit::core::MoveItErrorCode::SUCCESS);

//   // if (success) {
//   //   RCLCPP_INFO(LOGGER, "Planning successful, executing...");
//   //   move_group.execute(my_plan4);
//   // } else {
//   // }

//   //   // Wait for some time to ensure the action has been sent
//   // std::this_thread::sleep_for(std::chrono::seconds(5));
//   // // Close Gripper

//   //   // Close Gripper
//   // RCLCPP_INFO(LOGGER, "Closing Gripper!");
//   // auto opening_goal_msg = control_msgs::action::GripperCommand::Goal();
//   // opening_goal_msg.command.max_effort = 30;
//   // opening_goal_msg.command.position = 0.03;

//   // future_goal_handle = action_client->async_send_goal(opening_goal_msg);
//   // if(rclcpp::spin_until_future_complete(gripper_client_node,future_goal_handle) != rclcpp::FutureReturnCode::SUCCESS){
//   //    RCLCPP_INFO(LOGGER, "Failed send goal");
//   // }

//   // auto send_open_handler = future_goal_handle.get();
//   // future_result = action_client->async_get_result(send_open_handler);
  

//   // RCLCPP_INFO(LOGGER, "Final Retreat");
//   // geometry_msgs::msg::Pose target_pose5;
//   // target_pose5.orientation.x = 0.995905;
//   // target_pose5.orientation.y = -0.090405;
//   // target_pose5.orientation.z = -0.000896;
//   // target_pose5.orientation.w =  0.000204;
//   // target_pose5.position.x =  0.508867;
//   // target_pose5.position.y =   0.200236;
//   // target_pose5.position.z =  0.638926;
//   // move_group.setPlannerId("RRTstarkConfigDefault");

//   // move_group.setPoseTarget(target_pose5, "panda_hand");

//   // moveit::planning_interface::MoveGroupInterface::Plan my_plan5;

//   // success = (move_group.plan(my_plan5) == moveit::core::MoveItErrorCode::SUCCESS);

//   // if (success) {
//   //   RCLCPP_INFO(LOGGER, "Planning successful, executing...");
//   //   move_group.execute(my_plan5);
//   // } else {
//   //   RCLCPP_ERROR(LOGGER, "Planning failed!");
//   // }


//   // // Home Pose
 
//   // RCLCPP_INFO(LOGGER, "Ready Pose");
//   // geometry_msgs::msg::Pose ready_pose;
//   // ready_pose.orientation.x = 0.999989;
//   // ready_pose.orientation.y = -0.003096;
//   // ready_pose.orientation.z =  0.003040;
//   // ready_pose.orientation.w =  0.001641;
//   // ready_pose.position.x =  0.307589;
//   // ready_pose.position.y =   -0.000230;
//   // ready_pose.position.z =   0.591024;
//   // move_group.setPlannerId("RRTstarkConfigDefault");

//   // move_group.setPoseTarget(ready_pose, "panda_hand");

//   // moveit::planning_interface::MoveGroupInterface::Plan my_home_plan;

//   // success = (move_group.plan(my_home_plan) == moveit::core::MoveItErrorCode::SUCCESS);

//   // if (success) {
//   //   RCLCPP_INFO(LOGGER, "Planning successful, executing...");
//   //   move_group.execute(my_home_plan);
//   // } else {
//   //   RCLCPP_ERROR(LOGGER, "Planning failed!");
//   // }


//   // Shutdown ROS
//   rclcpp::shutdown();
//   return 0;
// }

// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>

// #include <moveit_msgs/msg/display_robot_state.hpp>
// #include <moveit_msgs/msg/display_trajectory.hpp>

// #include "rclcpp/rclcpp.hpp"
// #include "control_msgs/action/gripper_command.hpp"
// #include "rclcpp_action/rclcpp_action.hpp"

// #include <thread>
// #include <rclcpp/duration.hpp>

// static const rclcpp::Logger LOGGER = rclcpp::get_logger("test_trajectory");

// int main(int argc, char **argv) {
//   rclcpp::init(argc, argv);
//   rclcpp::NodeOptions node_options;
//   node_options.automatically_declare_parameters_from_overrides(true);
//   auto move_group_node = rclcpp::Node::make_shared("test_trajectory", node_options);
//   auto gripper_client_node = rclcpp::Node::make_shared("gripper_client_node", node_options);

//   rclcpp::executors::SingleThreadedExecutor executor;
//   executor.add_node(move_group_node);
//   std::thread([&executor]() { executor.spin(); }).detach();

//   static const std::string PLANNING_GROUP = "panda_arm";
//   static const std::string PLANNING_GROUP_GRIPPER = "hand";

//   moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
//   moveit::planning_interface::MoveGroupInterface move_group_gripper(gripper_client_node, PLANNING_GROUP_GRIPPER);

//   const moveit::core::JointModelGroup *joint_model_group =
//     move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

//   RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
//   RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

//   RCLCPP_INFO(LOGGER, "Available Planning Groups:");
//   std::copy(move_group.getJointModelGroupNames().begin(),
//             move_group.getJointModelGroupNames().end(),
//             std::ostream_iterator<std::string>(std::cout, ", "));

//   // Open the gripper before starting
//   std::vector<double> gripper_open = {0.035, 0.035}; // Fully open
//   move_group_gripper.setJointValueTarget(gripper_open);
//   move_group_gripper.setMaxVelocityScalingFactor(0.1);
//   move_group_gripper.setMaxAccelerationScalingFactor(0.1);
//   move_group_gripper.move();
//   std::this_thread::sleep_for(std::chrono::seconds(1));

//   // Function to pick and place an object
//   auto pick_and_place = [&](const geometry_msgs::msg::Pose& pick_pose,
//                             const geometry_msgs::msg::Pose& place_pose,
//                             const std::string& object_name) {
//     RCLCPP_INFO(LOGGER, "Picking up the %s", object_name.c_str());

//     // Move to pre-grasp position
//     move_group.setPoseTarget(pick_pose, "panda_hand");
//     moveit::planning_interface::MoveGroupInterface::Plan pick_plan;
//     bool success = (move_group.plan(pick_plan) == moveit::core::MoveItErrorCode::SUCCESS);
//     if (success) {
//       move_group.execute(pick_plan);
//     } else {
//       RCLCPP_ERROR(LOGGER, "Failed to plan to %s pre-grasp position", object_name.c_str());
//       return;
//     }
//     std::this_thread::sleep_for(std::chrono::seconds(1));

//     // Move down to grasp the object
//     geometry_msgs::msg::Pose grasp_pose = pick_pose;
//     grasp_pose.position.z -= 0.07; // Adjust as needed
//     move_group.setPoseTarget(grasp_pose, "panda_hand");
//     moveit::planning_interface::MoveGroupInterface::Plan grasp_plan;
//     success = (move_group.plan(grasp_plan) == moveit::core::MoveItErrorCode::SUCCESS);
//     if (success) {
//       move_group.execute(grasp_plan);
//     } else {
//       RCLCPP_ERROR(LOGGER, "Failed to plan to %s grasp position", object_name.c_str());
//       return;
//     }
//     std::this_thread::sleep_for(std::chrono::seconds(1));

//     // Close the gripper
//     std::vector<double> gripper_close = {0.03, 0.03}; // Fully closed
//     move_group_gripper.setJointValueTarget(gripper_close);
//     move_group_gripper.move();
//     std::this_thread::sleep_for(std::chrono::seconds(1));

//     // Lift the object
//     move_group.setPoseTarget(pick_pose, "panda_hand");
//     moveit::planning_interface::MoveGroupInterface::Plan lift_plan;
//     success = (move_group.plan(lift_plan) == moveit::core::MoveItErrorCode::SUCCESS);
//     if (success) {
//       move_group.execute(lift_plan);
//     } else {
//       RCLCPP_ERROR(LOGGER, "Failed to plan to lift the %s", object_name.c_str());
//       return;
//     }
//     std::this_thread::sleep_for(std::chrono::seconds(1));

//     // Move to place position
//     RCLCPP_INFO(LOGGER, "Placing the %s", object_name.c_str());
//     move_group.setPoseTarget(place_pose, "panda_hand");
//     moveit::planning_interface::MoveGroupInterface::Plan place_plan;
//     success = (move_group.plan(place_plan) == moveit::core::MoveItErrorCode::SUCCESS);
//     if (success) {
//       move_group.execute(place_plan);
//     } else {
//       RCLCPP_ERROR(LOGGER, "Failed to plan to place the %s", object_name.c_str());
//       return;
//     }
//     std::this_thread::sleep_for(std::chrono::seconds(1));

//     // Open the gripper to release the object
//     move_group_gripper.setJointValueTarget(gripper_open);
//     move_group_gripper.move();
//     std::this_thread::sleep_for(std::chrono::seconds(1));

//     // Retreat
//     geometry_msgs::msg::Pose retreat_pose = place_pose;
//     retreat_pose.position.z += 0.1; // Move up
//     move_group.setPoseTarget(retreat_pose, "panda_hand");
//     moveit::planning_interface::MoveGroupInterface::Plan retreat_plan;
//     success = (move_group.plan(retreat_plan) == moveit::core::MoveItErrorCode::SUCCESS);
//     if (success) {
//       move_group.execute(retreat_plan);
//     } else {
//       RCLCPP_ERROR(LOGGER, "Failed to plan to retreat after placing the %s", object_name.c_str());
//       return;
//     }
//     std::this_thread::sleep_for(std::chrono::seconds(1));
//   };

//   // Define the apple pick pose
//   geometry_msgs::msg::Pose apple_pick_pose;
//   apple_pick_pose.orientation.x =  0.995829;
//   apple_pick_pose.orientation.y =  -0.09123;
//   apple_pick_pose.orientation.z = 0.0;
//   apple_pick_pose.orientation.w = 0.0;

//   apple_pick_pose.position.x =  0.75746;
//   apple_pick_pose.position.y = -0.15068;
//   apple_pick_pose.position.z =  0.43496;
//   //  apple_pick_pose.position.x = 0.682;
//   // apple_pick_pose.position.y = -0.043;
//   // apple_pick_pose.position.z = 0.50;
 

//   // Define the place pose (e.g., plate position)
//   geometry_msgs::msg::Pose place_pose;
//   place_pose.orientation.x = 1.0;
//   place_pose.orientation.y = 0.0;
//   place_pose.orientation.z = 0.0;
//   place_pose.orientation.w = 0.0;
//   place_pose.position.x = 0.37599;
//   place_pose.position.y = 0.22709;
//   place_pose.position.z = 0.5;

//   // Pick and place the apple
//   pick_and_place(apple_pick_pose, place_pose, "apple");

//   // Define the banana pick pose
//   geometry_msgs::msg::Pose banana_pick_pose;
//   banana_pick_pose.orientation.x = 1.0;
//   banana_pick_pose.orientation.y = 0.0;
//   banana_pick_pose.orientation.z = 0.0;
//   banana_pick_pose.orientation.w = 0.0;
//   banana_pick_pose.position.x = 0.521;
//   banana_pick_pose.position.y = -0.075;
//   banana_pick_pose.position.z = 0.50;

//   // Optionally define a different place pose for the banana
//   geometry_msgs::msg::Pose banana_place_pose = place_pose; // Same place as apple

//   // Pick and place the banana
//   pick_and_place(banana_pick_pose, banana_place_pose, "banana");

//   // Shutdown ROS
//   rclcpp::shutdown();
//   return 0;
// }

// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>

// #include <moveit_msgs/msg/display_robot_state.hpp>
// #include <moveit_msgs/msg/display_trajectory.hpp>

// #include "rclcpp/rclcpp.hpp"
// #include "control_msgs/action/gripper_command.hpp"
// #include "rclcpp_action/rclcpp_action.hpp"

// #include <thread>
// #include <rclcpp/duration.hpp>

// static const rclcpp::Logger LOGGER = rclcpp::get_logger("test_trajectory");

// int main(int argc, char **argv) {
//   rclcpp::init(argc, argv);
//   rclcpp::NodeOptions node_options;
//   node_options.automatically_declare_parameters_from_overrides(true);
//   auto move_group_node = rclcpp::Node::make_shared("test_trajectory", node_options);
//   auto gripper_client_node = rclcpp::Node::make_shared("gripper_client_node", node_options);

//   rclcpp::executors::SingleThreadedExecutor executor;
//   executor.add_node(move_group_node);
//   std::thread([&executor]() { executor.spin(); }).detach();

//   static const std::string PLANNING_GROUP = "panda_arm";
//   static const std::string PLANNING_GROUP_GRIPPER = "hand";

//   moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
//   move_group.setPlannerId("RRTStarkConfigDefault");
//   move_group.setPlanningTime(50);
//   moveit::planning_interface::MoveGroupInterface move_group_gripper(gripper_client_node, PLANNING_GROUP_GRIPPER);

//   const moveit::core::JointModelGroup *joint_model_group =
//     move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

//   RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
//   RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

//   RCLCPP_INFO(LOGGER, "Available Planning Groups:");
//   std::copy(move_group.getJointModelGroupNames().begin(),
//             move_group.getJointModelGroupNames().end(),
//             std::ostream_iterator<std::string>(std::cout, ", "));

//   // Open the gripper before starting
//   std::vector<double> gripper_open = {0.035, 0.035}; // Fully open
//   move_group_gripper.setJointValueTarget(gripper_open);
//   move_group_gripper.setMaxVelocityScalingFactor(0.1);
//   move_group_gripper.setMaxAccelerationScalingFactor(0.1);
//   move_group_gripper.move();
//   std::this_thread::sleep_for(std::chrono::seconds(1));

//   // Function to pick and place an object
//   auto pick_and_place = [&](const geometry_msgs::msg::Pose& pick_pose,
//                             const geometry_msgs::msg::Pose& place_pose,
//                             const std::string& object_name) {
//     RCLCPP_INFO(LOGGER, "Picking up the %s", object_name.c_str());

//     // Move to pre-grasp position
//     move_group.setPoseTarget(pick_pose, "panda_hand");
//     moveit::planning_interface::MoveGroupInterface::Plan pick_plan;
//     bool success = (move_group.plan(pick_plan) == moveit::core::MoveItErrorCode::SUCCESS);
//     if (success) {
//       move_group.execute(pick_plan);
//     } else {
//       RCLCPP_ERROR(LOGGER, "Failed to plan to %s pre-grasp position", object_name.c_str());
//       return;
//     }
//     std::this_thread::sleep_for(std::chrono::seconds(1));

//     // Move down to grasp the object
//     geometry_msgs::msg::Pose grasp_pose = pick_pose;
//     grasp_pose.position.z -= 0.04; // Adjust as needed
//     move_group.setPoseTarget(grasp_pose, "panda_hand");
//     moveit::planning_interface::MoveGroupInterface::Plan grasp_plan;
//     success = (move_group.plan(grasp_plan) == moveit::core::MoveItErrorCode::SUCCESS);
//     if (success) {
//       move_group.execute(grasp_plan);
//     } else {
//       RCLCPP_ERROR(LOGGER, "Failed to plan to %s grasp position", object_name.c_str());
//       return;
//     }
//     std::this_thread::sleep_for(std::chrono::seconds(1));

//     // Close the gripper
//     std::vector<double> gripper_close = {0.03, 0.03}; // Fully closed
//     move_group_gripper.setJointValueTarget(gripper_close);
//     move_group_gripper.move();
//     std::this_thread::sleep_for(std::chrono::seconds(1));

//     // Lift the object
//     move_group.setPoseTarget(pick_pose, "panda_hand");
//     moveit::planning_interface::MoveGroupInterface::Plan lift_plan;
//     success = (move_group.plan(lift_plan) == moveit::core::MoveItErrorCode::SUCCESS);
//     if (success) {
//       move_group.execute(lift_plan);
//     } else {
//       RCLCPP_ERROR(LOGGER, "Failed to plan to lift the %s", object_name.c_str());
//       return;
//     }
//     std::this_thread::sleep_for(std::chrono::seconds(1));

//     // Move to place position
//     RCLCPP_INFO(LOGGER, "Placing the %s", object_name.c_str());
//     move_group.setPoseTarget(place_pose, "panda_hand");
//     moveit::planning_interface::MoveGroupInterface::Plan place_plan;
//     success = (move_group.plan(place_plan) == moveit::core::MoveItErrorCode::SUCCESS);
//     if (success) {
//       move_group.execute(place_plan);
//     } else {
//       RCLCPP_ERROR(LOGGER, "Failed to plan to place the %s", object_name.c_str());
//       return;
//     }
//     std::this_thread::sleep_for(std::chrono::seconds(1));

//     // Open the gripper to release the object
//     move_group_gripper.setJointValueTarget(gripper_open);
//     move_group_gripper.move();
//     std::this_thread::sleep_for(std::chrono::seconds(1));

//     // Retreat
//     geometry_msgs::msg::Pose retreat_pose = place_pose;
//     retreat_pose.position.z += 0.1; // Move up
//     move_group.setPoseTarget(retreat_pose, "panda_hand");
//     moveit::planning_interface::MoveGroupInterface::Plan retreat_plan;
//     success = (move_group.plan(retreat_plan) == moveit::core::MoveItErrorCode::SUCCESS);
//     if (success) {
//       move_group.execute(retreat_plan);
//     } else {
//       RCLCPP_ERROR(LOGGER, "Failed to plan to retreat after placing the %s", object_name.c_str());
//       return;
//     }
//     std::this_thread::sleep_for(std::chrono::seconds(1));
//   };

//   // Define the apple pick pose
//   geometry_msgs::msg::Pose banana_pick_pose;
//   banana_pick_pose.orientation.x = 1.0;
//   banana_pick_pose.orientation.y = 0.0;
//   banana_pick_pose.orientation.z = 0.0;
//   banana_pick_pose.orientation.w = 0.0;
//   banana_pick_pose.position.x = 0.704;
//   banana_pick_pose.position.y = -0.144;
//   banana_pick_pose.position.z = 0.50;

//   // Define the place pose (e.g., plate position)
//   geometry_msgs::msg::Pose place_pose;
//   place_pose.orientation.x = 1.0;
//   place_pose.orientation.y = 0.0;
//   place_pose.orientation.z = 0.0;
//   place_pose.orientation.w = 0.0;
//   place_pose.position.x = 0.3396;
//   place_pose.position.y = 0.1596;
//   place_pose.position.z = 0.4658;

//   // Pick and place the apple
//   pick_and_place(banana_pick_pose, place_pose, "banana");

//   // // Define the banana pick pose
//   geometry_msgs::msg::Pose bluecup_pick_pose;
//   bluecup_pick_pose.orientation.x = 1.0;
//   bluecup_pick_pose.orientation.y = 0.0;
//   bluecup_pick_pose.orientation.z = 0.0;
//   bluecup_pick_pose.orientation.w = 0.0;
//   bluecup_pick_pose.position.x = 0.769;
//   bluecup_pick_pose.position.y = 0.185;
//   bluecup_pick_pose.position.z = 0.50;

//   // Optionally define a different place pose for the banana
//   geometry_msgs::msg::Pose bluecup_place_pose = place_pose; // Same place as apple

//   // Pick and place the banana
//   pick_and_place(bluecup_pick_pose, bluecup_place_pose, "banana");

//   // Shutdown ROS
//   rclcpp::shutdown();
//   return 0;
// }





// ////SLERP CARTESIAN
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>

// #include <moveit_msgs/msg/display_robot_state.hpp>
// #include <moveit_msgs/msg/display_trajectory.hpp>

// #include "rclcpp/rclcpp.hpp"
// #include "control_msgs/action/gripper_command.hpp"
// #include "rclcpp_action/rclcpp_action.hpp"

// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <Eigen/Geometry>

// #include <thread>
// #include <rclcpp/duration.hpp>

// static const rclcpp::Logger LOGGER = rclcpp::get_logger("test_trajectory");

// // SLERP function
// geometry_msgs::msg::Quaternion slerp(const geometry_msgs::msg::Quaternion& q1,
//                                      const geometry_msgs::msg::Quaternion& q2, double t) {
//     tf2::Quaternion tf_q1, tf_q2;
//     tf2::fromMsg(q1, tf_q1);
//     tf2::fromMsg(q2, tf_q2);

//     tf2::Quaternion tf_q_slerp = tf_q1.slerp(tf_q2, t);
//     return tf2::toMsg(tf_q_slerp);
// }

// int main(int argc, char **argv) {
//   // Initialize ROS
//   rclcpp::init(argc, argv);
//   rclcpp::NodeOptions node_options;
//   node_options.automatically_declare_parameters_from_overrides(true);
//   auto move_group_node = rclcpp::Node::make_shared("test_trajectory", node_options);
//   auto gripper_client_node = rclcpp::Node::make_shared("gripper_client_node", node_options);

//   rclcpp::executors::SingleThreadedExecutor executor;
//   executor.add_node(move_group_node);
//   std::thread([&executor]() { executor.spin(); }).detach();

//   // Define planning groups
//   static const std::string PLANNING_GROUP = "panda_arm";
//   static const std::string PLANNING_GROUP_GRIPPER = "hand";

//   // Initialize MoveIt interfaces
//   moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
//   // Set correct end-effector link here
//   move_group.setEndEffectorLink("panda_hand");

// // Optionally verify it
//   RCLCPP_INFO(LOGGER, "End-effector link is now: %s", move_group.getEndEffectorLink().c_str());
//   // move_group.setPlannerId("PRMkConfigDefault"); // Enforce the planner
//   move_group.setPlannerId("PRMstarkConfigDefault"); 

//   move_group.setPlanningTime(10.0);               // Allow sufficient planning time

//   moveit::planning_interface::MoveGroupInterface move_group_gripper(gripper_client_node, PLANNING_GROUP_GRIPPER);

//   // Log planning frame and end-effector link
//   RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
//   RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

//   // Open the gripper before starting
//   std::vector<double> gripper_open = {0.035, 0.035}; // Fully open
//   move_group_gripper.setJointValueTarget(gripper_open);
//   move_group_gripper.setMaxVelocityScalingFactor(0.1);
//   move_group_gripper.setMaxAccelerationScalingFactor(0.1);
//   move_group_gripper.move();
//   std::this_thread::sleep_for(std::chrono::seconds(1));

//   // Function to pick and place an object
//   auto pick_and_place = [&](const geometry_msgs::msg::Pose &pick_pose,
//                             const geometry_msgs::msg::Pose &place_pose,
//                             const std::string &object_name) {
//     RCLCPP_INFO(LOGGER, "Picking up the %s", object_name.c_str());

//     // Define pre-grasp pose (slightly above the object)
//     geometry_msgs::msg::Pose pre_grasp_pose = pick_pose;
//     pre_grasp_pose.position.z += 0.10; // Adjust as needed

//     // Move to pre-grasp pose
//     move_group.setPoseTarget(pre_grasp_pose, "panda_hand");
//     moveit::planning_interface::MoveGroupInterface::Plan pre_grasp_plan;
//     bool success = (move_group.plan(pre_grasp_plan) == moveit::core::MoveItErrorCode::SUCCESS);
//     if (success) {
//       move_group.execute(pre_grasp_plan);
//     } else {
//       RCLCPP_ERROR(LOGGER, "Failed to plan to pre-grasp position");
//       return;
//     }
//     std::this_thread::sleep_for(std::chrono::seconds(1));

//     // Plan a Cartesian path from pre-grasp to grasp pose using SLERP
//     std::vector<geometry_msgs::msg::Pose> waypoints;
//     int num_waypoints = 20; // Adjust as needed

//     // Define grasp pose
//     geometry_msgs::msg::Pose grasp_pose = pre_grasp_pose;
//     grasp_pose.position.z -= 0.07; // Move down towards the object

//     // Convert positions to Eigen vectors
//     Eigen::Vector3d start_position(pre_grasp_pose.position.x, pre_grasp_pose.position.y, pre_grasp_pose.position.z);
//     Eigen::Vector3d end_position(grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z);

//     // Generate waypoints with interpolated positions and orientations
//     for (int i = 1; i <= num_waypoints; ++i) {
//       double t = static_cast<double>(i) / static_cast<double>(num_waypoints);

//       // Interpolate position
//       Eigen::Vector3d interp_position = start_position + t * (end_position - start_position);

//       // Interpolate orientation using SLERP
//       geometry_msgs::msg::Quaternion interp_orientation = slerp(pre_grasp_pose.orientation, grasp_pose.orientation, t);

//       // Create intermediate pose
//       geometry_msgs::msg::Pose waypoint;
//       waypoint.position.x = interp_position.x();
//       waypoint.position.y = interp_position.y();
//       waypoint.position.z = interp_position.z();
//       waypoint.orientation = interp_orientation;

//       waypoints.push_back(waypoint);
//     }

//     moveit_msgs::msg::RobotTrajectory cartesian_traj;
//     const double jump_threshold = 2.0; // Set a suitable jump threshold
//     const double eef_step = 0.005; // 1 cm resolution

//     double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, cartesian_traj);
//     if (fraction < 0.9) {
//       RCLCPP_ERROR(LOGGER, "Unable to compute Cartesian path to grasp position. Achieved %.2f%% of the path", fraction * 100.0);
//       return;
//     }

//     // Execute the Cartesian path
//     moveit::planning_interface::MoveGroupInterface::Plan grasp_plan;
//     grasp_plan.trajectory_ = cartesian_traj;
//     move_group.execute(grasp_plan);
//     std::this_thread::sleep_for(std::chrono::seconds(1));

//     // Close the gripper
//     std::vector<double> gripper_close = {0.02, 0.02}; // Adjust as needed
//     move_group_gripper.setJointValueTarget(gripper_close);
//     move_group_gripper.move();
//     std::this_thread::sleep_for(std::chrono::seconds(1));

//     // Lift the object by reversing the Cartesian path
//     waypoints.clear();
//     // Generate waypoints from grasp_pose back to pre_grasp_pose
//     Eigen::Vector3d start_position_lift(grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z);
//     Eigen::Vector3d end_position_lift(pre_grasp_pose.position.x, pre_grasp_pose.position.y, pre_grasp_pose.position.z);

//     for (int i = 1; i <= num_waypoints; ++i) {
//       double t = static_cast<double>(i) / static_cast<double>(num_waypoints);

//       // Interpolate position
//       Eigen::Vector3d interp_position = start_position_lift + t * (end_position_lift - start_position_lift);

//       // Interpolate orientation using SLERP
//       geometry_msgs::msg::Quaternion interp_orientation = slerp(grasp_pose.orientation, pre_grasp_pose.orientation, t);

//       // Create intermediate pose
//       geometry_msgs::msg::Pose waypoint;
//       waypoint.position.x = interp_position.x();
//       waypoint.position.y = interp_position.y();
//       waypoint.position.z = interp_position.z();
//       waypoint.orientation = interp_orientation;

//       waypoints.push_back(waypoint);
//     }

//     fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, cartesian_traj);
//     if (fraction < 0.9) {
//       RCLCPP_ERROR(LOGGER, "Unable to compute Cartesian path to lift the object. Achieved %.2f%% of the path", fraction * 100.0);
//       return;
//     }

//     grasp_plan.trajectory_ = cartesian_traj;
//     move_group.execute(grasp_plan);
//     std::this_thread::sleep_for(std::chrono::seconds(1));

//     // Move to place position
//     RCLCPP_INFO(LOGGER, "Placing the %s", object_name.c_str());
//     move_group.setPoseTarget(place_pose, "panda_hand");
//     moveit::planning_interface::MoveGroupInterface::Plan place_plan;
//     success = (move_group.plan(place_plan) == moveit::core::MoveItErrorCode::SUCCESS);
//     if (success) {
//       move_group.execute(place_plan);
//     } else {
//       RCLCPP_ERROR(LOGGER, "Failed to plan to place the %s", object_name.c_str());
//       return;
//     }
//     std::this_thread::sleep_for(std::chrono::seconds(1));

//     // Open the gripper to release the object
//     move_group_gripper.setJointValueTarget(gripper_open);
//     move_group_gripper.move();
//     std::this_thread::sleep_for(std::chrono::seconds(1));

//     // Retreat by moving up using a Cartesian path
//     waypoints.clear();

//     // Define retreat pose
//     geometry_msgs::msg::Pose retreat_pose = place_pose;
//     retreat_pose.position.z += 0.07; // Move up

//     // Generate waypoints from place_pose to retreat_pose
//     Eigen::Vector3d start_position_retreat(place_pose.position.x, place_pose.position.y, place_pose.position.z);
//     Eigen::Vector3d end_position_retreat(retreat_pose.position.x, retreat_pose.position.y, retreat_pose.position.z);

//     for (int i = 1; i <= num_waypoints; ++i) {
//       double t = static_cast<double>(i) / static_cast<double>(num_waypoints);

//       // Interpolate position
//       Eigen::Vector3d interp_position = start_position_retreat + t * (end_position_retreat - start_position_retreat);

//       // Interpolate orientation using SLERP
//       geometry_msgs::msg::Quaternion interp_orientation = slerp(place_pose.orientation, retreat_pose.orientation, t);

//       // Create intermediate pose
//       geometry_msgs::msg::Pose waypoint;
//       waypoint.position.x = interp_position.x();
//       waypoint.position.y = interp_position.y();
//       waypoint.position.z = interp_position.z();
//       waypoint.orientation = interp_orientation;

//       waypoints.push_back(waypoint);
//     }

//     fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, cartesian_traj);
//     if (fraction < 0.9) {
//       RCLCPP_ERROR(LOGGER, "Unable to compute Cartesian path to retreat after placing the object. Achieved %.2f%% of the path", fraction * 100.0);
//       return;
//     }

//     grasp_plan.trajectory_ = cartesian_traj;
//     move_group.execute(grasp_plan);
//     std::this_thread::sleep_for(std::chrono::seconds(1));
//   };
//    // for plump
//     // Define pick poses for multiple fruits
//     geometry_msgs::msg::Pose apple_pick_pose;
//     apple_pick_pose.orientation.x = 0.995829;
//     apple_pick_pose.orientation.y = -0.09123;
//     apple_pick_pose.orientation.z = 0.0;
//     apple_pick_pose.orientation.w = 0.0;
//     apple_pick_pose.position.x = 0.6308;
//     apple_pick_pose.position.y = 0.063;
//     apple_pick_pose.position.z = 0.42;
    
//     geometry_msgs::msg::Pose banana_pick_pose;
//     banana_pick_pose.orientation.x = 1.0;
//     banana_pick_pose.orientation.y = 0.0;
//     banana_pick_pose.orientation.z = 0.0;
//     banana_pick_pose.orientation.w = 0.0;
//     banana_pick_pose.position.x = 0.586;
//     banana_pick_pose.position.y = -0.185;
//     banana_pick_pose.position.z = 0.41;

//     geometry_msgs::msg::Pose orange_pick_pose;
//     orange_pick_pose.orientation.x = 1.0;
//     orange_pick_pose.orientation.y = 0.0;
//     orange_pick_pose.orientation.z = 0.0;
//     orange_pick_pose.orientation.w = 0.0;
//     orange_pick_pose.position.x = 0.530;
//     orange_pick_pose.position.y = -0.06;
//     orange_pick_pose.position.z = 0.42;

//     // Use the same place pose for all fruits
//     geometry_msgs::msg::Pose place_pose;
//     place_pose.orientation.x = 1.0;
//     place_pose.orientation.y = 0.0;
//     place_pose.orientation.z = 0.0;
//     place_pose.orientation.w = 0.0;
//     place_pose.position.x = 0.37599;
//     place_pose.position.y = 0.22709;
//     place_pose.position.z = 0.5;
    
//     pick_and_place(orange_pick_pose, place_pose, "orange");
//     std::this_thread::sleep_for(std::chrono::seconds(2));
//     // Execute pick and place for each fruit with pause
//     pick_and_place(apple_pick_pose, place_pose, "apple");
//     std::this_thread::sleep_for(std::chrono::seconds(2));

//     pick_and_place(banana_pick_pose, place_pose, "banana");
//     std::this_thread::sleep_for(std::chrono::seconds(2));

  
   

//     // Final pause to let the last move settle
//     std::this_thread::sleep_for(std::chrono::seconds(4));
//     // Shutdown ROS
//     rclcpp::shutdown();
//     return 0;

//   // Shutdown ROS

// }

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
  prepick_pose.position.z = 0.07;
  prepick_pose.orientation.x = 0.999280;
  prepick_pose.orientation.y = 0.020395;
  prepick_pose.orientation.z = -0.031096;
  prepick_pose.orientation.w = 0.007522;

  geometry_msgs::msg::Pose pick_pose;
  pick_pose.position.x = 0.845;
  pick_pose.position.y = -0.3678;
  pick_pose.position.z = 0.21;  // Was 0.420 + 0.01; now set directly
  pick_pose.orientation = prepick_pose.orientation;


  geometry_msgs::msg::Pose postpick_pose;
  postpick_pose.position.x = 0.632575;
  postpick_pose.position.y = -0.040577;
  postpick_pose.position.z = 0.07;
  postpick_pose.orientation.x = 0.999332;
  postpick_pose.orientation.y = 0.026453;
  postpick_pose.orientation.z = 0.013079;
  postpick_pose.orientation.w = 0.021576;

  geometry_msgs::msg::Pose clearance_pose;
  clearance_pose.position.x = 0.580421;
  clearance_pose.position.y = 0.195515;
  clearance_pose.position.z = 0.21070;
  clearance_pose.orientation.x = 0.968212;
  clearance_pose.orientation.y = 0.241663;
  clearance_pose.orientation.z = 0.042857;
  clearance_pose.orientation.w = 0.048241;


  geometry_msgs::msg::Pose preplace_pose;
  preplace_pose.position.x = 0.615095;
  preplace_pose.position.y = 0.312747;
  preplace_pose.position.z = 0.07;
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
  place_pose.position.z = 0.07;
  place_pose.orientation.x = 0.498944;
  place_pose.orientation.y = 0.549175;
  place_pose.orientation.z = -0.444454;
  place_pose.orientation.w = 0.501918;

  geometry_msgs::msg::Pose postplace_pose;
  postplace_pose.position.x = 0.586622;
  postplace_pose.position.y = 0.017014;
  postplace_pose.position.z = 0.21420;
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


// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <moveit_msgs/msg/display_robot_state.hpp>
// #include <moveit_msgs/msg/display_trajectory.hpp>
// #include <rclcpp/rclcpp.hpp>
// #include <control_msgs/action/gripper_command.hpp>
// #include <rclcpp_action/rclcpp_action.hpp>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <tf2/LinearMath/Quaternion.h>
// #include <Eigen/Geometry>
// #include <thread>
// #include <rclcpp/duration.hpp>

// static const rclcpp::Logger LOGGER = rclcpp::get_logger("test_trajectory");

// int main(int argc, char **argv) {
//   rclcpp::init(argc, argv);
//   rclcpp::NodeOptions node_options;
//   node_options.automatically_declare_parameters_from_overrides(true);
//   auto move_group_node = rclcpp::Node::make_shared("test_trajectory", node_options);
//   auto gripper_client_node = rclcpp::Node::make_shared("gripper_client_node", node_options);

//   rclcpp::executors::SingleThreadedExecutor executor;
//   executor.add_node(move_group_node);
//   std::thread([&executor]() { executor.spin(); }).detach();

//   const std::string PLANNING_GROUP = "panda_arm";
//   const std::string PLANNING_GROUP_GRIPPER = "hand";

//   moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
//   move_group.setEndEffectorLink("panda_hand");
 
//   // move_group.setPlannerId("RRTStarConfigDefault");
//   move_group.setPlannerId("PRMstarkConfigDefault");
//   move_group.setPlanningTime(15.0);

//   moveit::planning_interface::MoveGroupInterface move_group_gripper(gripper_client_node, PLANNING_GROUP_GRIPPER);

//   RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
//   RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

//   // Open the gripper before starting
//   std::vector<double> gripper_open = {0.035, 0.035};
//   move_group_gripper.setJointValueTarget(gripper_open);
//   move_group_gripper.setMaxVelocityScalingFactor(0.1);
//   move_group_gripper.setMaxAccelerationScalingFactor(0.1);
//   move_group_gripper.move();
//   std::this_thread::sleep_for(std::chrono::seconds(1));

//   // Helper function to move to pose
//   auto move_to = [&](const geometry_msgs::msg::Pose &pose, const std::string &desc) {
//     RCLCPP_INFO(LOGGER, "Attempting to move to %s", desc.c_str());
//     RCLCPP_INFO(LOGGER, "Position: x=%.6f, y=%.6f, z=%.6f",
//                 pose.position.x, pose.position.y, pose.position.z);
//     RCLCPP_INFO(LOGGER, "Orientation: x=%.6f, y=%.6f, z=%.6f, w=%.6f",
//                 pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

//     move_group.setPoseTarget(pose, "panda_hand");
//     moveit::planning_interface::MoveGroupInterface::Plan plan;
//     if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
//       move_group.execute(plan);
//       RCLCPP_INFO(LOGGER, "Moved to %s", desc.c_str());
//     } else {
//       RCLCPP_ERROR(LOGGER, "Failed to move to %s", desc.c_str());
//     }
//     std::this_thread::sleep_for(std::chrono::seconds(5));
//   };

//   // 7 Semantic Poses from user-provided measurements
//   geometry_msgs::msg::Pose prepick_pose;
//   prepick_pose.position.x = 0.535264;
//   prepick_pose.position.y = -0.051543;
//   prepick_pose.position.z = 0.504939;
//   prepick_pose.orientation.x = 0.999280;
//   prepick_pose.orientation.y = 0.020395;
//   prepick_pose.orientation.z = -0.031096;
//   prepick_pose.orientation.w = 0.007522;

//   geometry_msgs::msg::Pose pick_pose;
//   pick_pose.position.x = 0.535264;
//   pick_pose.position.y = -0.051543;
//   pick_pose.position.z =  0.45;
//   pick_pose.orientation.x = 0.999280;
//   pick_pose.orientation.y = 0.020395;
//   pick_pose.orientation.z = -0.031096;
//   pick_pose.orientation.w = 0.007522;


//   geometry_msgs::msg::Pose postpick_pose;
//   postpick_pose.position.x = 0.632575;
//   postpick_pose.position.y = -0.040577;
//   postpick_pose.position.z = 0.5800;
//   postpick_pose.orientation.x = 0.999332;
//   postpick_pose.orientation.y = 0.026453;
//   postpick_pose.orientation.z = 0.013079;
//   postpick_pose.orientation.w = 0.021576;

//   geometry_msgs::msg::Pose clearance_pose;
//   clearance_pose.position.x = 0.580421;
//   clearance_pose.position.y = 0.195515;
//   clearance_pose.position.z = 0.611070;
//   clearance_pose.orientation.x = 0.968212;
//   clearance_pose.orientation.y = 0.241663;
//   clearance_pose.orientation.z = 0.042857;
//   clearance_pose.orientation.w = 0.048241;

//   geometry_msgs::msg::Pose preplace_pose = clearance_pose;
  

//   geometry_msgs::msg::Pose place_pose;
//   place_pose.position.x = 0.540928;
//   place_pose.position.y = 0.275853;
//   place_pose.position.z = 0.525571;
//   place_pose.orientation.x = 0.750667;
//   place_pose.orientation.y = 0.236323;
//   place_pose.orientation.z = 0.112109;
//   place_pose.orientation.w = 0.606697;

//   geometry_msgs::msg::Pose postplace_pose;
//   postplace_pose.position.x = 0.586622;
//   postplace_pose.position.y = 0.017014;
//   postplace_pose.position.z = 0.671420;
//   postplace_pose.orientation.x = 0.989905;
//   postplace_pose.orientation.y = 0.077573;
//   postplace_pose.orientation.z = 0.003593;
//   postplace_pose.orientation.w = 0.118566;

//   std::vector<geometry_msgs::msg::Pose> banana_sequence = {
//     prepick_pose,
//     pick_pose,
//     postpick_pose,
//     clearance_pose,
//     preplace_pose,
//     place_pose,
//     postplace_pose
//   };

//   RCLCPP_INFO(LOGGER, "Starting semantic 7-pose pick and place for banana");

//   for (size_t i = 0; i < banana_sequence.size(); ++i) {
//     move_to(banana_sequence[i], "pose step " + std::to_string(i));

//     // Gripper logic at key steps
//     if (i == 1) {  // after reaching 'pick'
//       std::vector<double> gripper_close = {0.025, 0.025};
//       move_group_gripper.setPlannerId("RRTStarConfigDefault");
//       move_group_gripper.setJointValueTarget(gripper_close);
//       move_group_gripper.move();  // ← add this
//       std::this_thread::sleep_for(std::chrono::seconds(5));

//     } else if (i == 5) { // after reaching 'place'
//       std::vector<double> gripper_open = {0.035, 0.035};
//       move_group_gripper.setPlannerId("RRTStarConfigDefault");
//       move_group_gripper.setJointValueTarget(gripper_open);
//       move_group_gripper.move();  // ← add this
//       std::this_thread::sleep_for(std::chrono::seconds(5));

//     }
//     // if (i == 1) {  // after reaching 'pick'
//     //   std::vector<double> gripper_close = {0.02, 0.02};
//     //   move_group_gripper.setJointValueTarget(gripper_close);
//     // } else if (i == 5) { // after reaching 'place'
//     //   std::vector<double> gripper_open = {0.035, 0.035}; // open
//     //   move_group_gripper.setJointValueTarget(gripper_open);
//     // }
//   }

//   RCLCPP_INFO(LOGGER, "Banana pick and place completed");
//   rclcpp::shutdown();
//   return 0;
// }


// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>

// #include <moveit_msgs/msg/display_robot_state.hpp>
// #include <moveit_msgs/msg/display_trajectory.hpp>

// #include "rclcpp/rclcpp.hpp"
// #include "control_msgs/action/gripper_command.hpp"
// #include "rclcpp_action/rclcpp_action.hpp"

// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <Eigen/Geometry>

// #include <thread>
// #include <rclcpp/duration.hpp>

// static const rclcpp::Logger LOGGER = rclcpp::get_logger("test_trajectory");

// geometry_msgs::msg::Quaternion slerp(const geometry_msgs::msg::Quaternion& q1,
//                                      const geometry_msgs::msg::Quaternion& q2, double t) {
//     tf2::Quaternion tf_q1, tf_q2;
//     tf2::fromMsg(q1, tf_q1);
//     tf2::fromMsg(q2, tf_q2);

//     tf2::Quaternion tf_q_slerp = tf_q1.slerp(tf_q2, t);
//     return tf2::toMsg(tf_q_slerp);
// }

// int main(int argc, char **argv) {
//   rclcpp::init(argc, argv);
//   rclcpp::NodeOptions node_options;
//   node_options.automatically_declare_parameters_from_overrides(true);
//   auto move_group_node = rclcpp::Node::make_shared("test_trajectory", node_options);
//   auto gripper_client_node = rclcpp::Node::make_shared("gripper_client_node", node_options);

//   rclcpp::executors::SingleThreadedExecutor executor;
//   executor.add_node(move_group_node);
//   std::thread([&executor]() { executor.spin(); }).detach();

//   static const std::string PLANNING_GROUP = "panda_arm";
//   static const std::string PLANNING_GROUP_GRIPPER = "hand";

//   moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
//   move_group.setEndEffectorLink("panda_hand");
//   RCLCPP_INFO(LOGGER, "End-effector link is now: %s", move_group.getEndEffectorLink().c_str());
//   move_group.setPlannerId("RRTStarConfigDefault");
//   move_group.setPlanningTime(50.0);

//   moveit::planning_interface::MoveGroupInterface move_group_gripper(gripper_client_node, PLANNING_GROUP_GRIPPER);

//   RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());

//   std::vector<double> gripper_open = {0.035, 0.035};
//   move_group_gripper.setJointValueTarget(gripper_open);
//   move_group_gripper.setMaxVelocityScalingFactor(0.1);
//   move_group_gripper.setMaxAccelerationScalingFactor(0.1);
//   move_group_gripper.move();
//   std::this_thread::sleep_for(std::chrono::seconds(1));

//   auto pick_and_place = [&](const geometry_msgs::msg::Pose &pick_pose,
//                             const geometry_msgs::msg::Pose &place_pose,
//                             const std::string &object_name) {
//     RCLCPP_INFO(LOGGER, "Picking up the %s", object_name.c_str());

//     geometry_msgs::msg::Pose pre_grasp_pose = pick_pose;
//     pre_grasp_pose.position.z += 0.09;

//     move_group.setPoseTarget(pre_grasp_pose, "panda_hand");
//     moveit::planning_interface::MoveGroupInterface::Plan pre_grasp_plan;
//     if (move_group.plan(pre_grasp_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
//       move_group.execute(pre_grasp_plan);
//     } else {
//       RCLCPP_ERROR(LOGGER, "Failed to plan to pre-grasp position");
//       return;
//     }
//     std::this_thread::sleep_for(std::chrono::seconds(1));

//     std::vector<geometry_msgs::msg::Pose> waypoints;
//     int num_waypoints = 20;
//     geometry_msgs::msg::Pose grasp_pose = pre_grasp_pose;
//     grasp_pose.position.z -= 0.07;

//     Eigen::Vector3d start(pre_grasp_pose.position.x, pre_grasp_pose.position.y, pre_grasp_pose.position.z);
//     Eigen::Vector3d end(grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z);

//     for (int i = 1; i <= num_waypoints; ++i) {
//       double t = static_cast<double>(i) / num_waypoints;
//       Eigen::Vector3d interp = start + t * (end - start);
//       geometry_msgs::msg::Quaternion q = slerp(pre_grasp_pose.orientation, grasp_pose.orientation, t);
//       geometry_msgs::msg::Pose waypoint;
//       waypoint.position.x = interp.x();
//       waypoint.position.y = interp.y();
//       waypoint.position.z = interp.z();
//       waypoint.orientation = q;
//       waypoints.push_back(waypoint);
//     }

//     moveit_msgs::msg::RobotTrajectory traj;
//     double fraction = move_group.computeCartesianPath(waypoints, 0.005, 2.0, traj);
//     if (fraction < 0.9) {
//       RCLCPP_ERROR(LOGGER, "Failed Cartesian path to grasp (%.2f%%)", fraction * 100.0);
//       return;
//     }

//     moveit::planning_interface::MoveGroupInterface::Plan grasp_plan;
//     grasp_plan.trajectory_ = traj;
//     move_group.execute(grasp_plan);
//     std::this_thread::sleep_for(std::chrono::seconds(1));

//     std::vector<double> gripper_close = {0.02, 0.02};
//     move_group_gripper.setJointValueTarget(gripper_close);
//     move_group_gripper.move();
//     std::this_thread::sleep_for(std::chrono::seconds(1));

//     waypoints.clear();
//     Eigen::Vector3d lift_start(grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z);
//     Eigen::Vector3d lift_end(pre_grasp_pose.position.x, pre_grasp_pose.position.y, pre_grasp_pose.position.z);

//     for (int i = 1; i <= num_waypoints; ++i) {
//       double t = static_cast<double>(i) / num_waypoints;
//       Eigen::Vector3d interp = lift_start + t * (lift_end - lift_start);
//       geometry_msgs::msg::Quaternion q = slerp(grasp_pose.orientation, pre_grasp_pose.orientation, t);
//       geometry_msgs::msg::Pose waypoint;
//       waypoint.position.x = interp.x();
//       waypoint.position.y = interp.y();
//       waypoint.position.z = interp.z();
//       waypoint.orientation = q;
//       waypoints.push_back(waypoint);
//     }

//     fraction = move_group.computeCartesianPath(waypoints, 0.005, 2.0, traj);
//     if (fraction < 0.9) {
//       RCLCPP_ERROR(LOGGER, "Failed Cartesian path to lift (%.2f%%)", fraction * 100.0);
//       return;
//     }

//     grasp_plan.trajectory_ = traj;
//     move_group.execute(grasp_plan);
//     std::this_thread::sleep_for(std::chrono::seconds(1));

//     // RCLCPP_INFO(LOGGER, "Placing the %s", object_name.c_str());
//     // move_group.setPoseTarget(place_pose, "panda_hand");
//     // moveit::planning_interface::MoveGroupInterface::Plan place_plan;
//     // if (move_group.plan(place_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
//     //   move_group.execute(place_plan);
//     // } else {
//     //   RCLCPP_ERROR(LOGGER, "Failed to plan to place pose");
//     //   return;
//     // }
//     // std::this_thread::sleep_for(std::chrono::seconds(1));
//     RCLCPP_INFO(LOGGER, "Placing the %s", object_name.c_str());

//     // Step 1: Go to a pre-place pose (above the target)
//     geometry_msgs::msg::Pose pre_place_pose = place_pose;
//     pre_place_pose.position.z += 0.03;

//     move_group.setPoseTarget(pre_place_pose, "panda_hand");
//     moveit::planning_interface::MoveGroupInterface::Plan pre_place_plan;
//     if (move_group.plan(pre_place_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
//       move_group.execute(pre_place_plan);
//     } else {
//       RCLCPP_ERROR(LOGGER, "Failed to plan to pre-place pose");
//       return;
//     }
//     std::this_thread::sleep_for(std::chrono::seconds(1));

//     // Step 2: Lower slightly to release object (not all the way down)
//     geometry_msgs::msg::Pose adjusted_place_pose = pre_place_pose;
//     adjusted_place_pose.position.z -= 0.03;

//     move_group.setPoseTarget(adjusted_place_pose, "panda_hand");
//     moveit::planning_interface::MoveGroupInterface::Plan place_plan;
//     if (move_group.plan(place_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
//       move_group.execute(place_plan);
//     } else {
//       RCLCPP_ERROR(LOGGER, "Failed to plan to adjusted place pose");
//       return;
//     }
//     std::this_thread::sleep_for(std::chrono::seconds(1));

//     move_group_gripper.setJointValueTarget(gripper_open);
//     move_group_gripper.move();
//     std::this_thread::sleep_for(std::chrono::seconds(1));

//     waypoints.clear();
//     geometry_msgs::msg::Pose retreat_pose = place_pose;
//     retreat_pose.position.z += 0.07;
//     Eigen::Vector3d ret_start(place_pose.position.x, place_pose.position.y, place_pose.position.z);
//     Eigen::Vector3d ret_end(retreat_pose.position.x, retreat_pose.position.y, retreat_pose.position.z);

//     for (int i = 1; i <= num_waypoints; ++i) {
//       double t = static_cast<double>(i) / num_waypoints;
//       Eigen::Vector3d interp = ret_start + t * (ret_end - ret_start);
//       geometry_msgs::msg::Quaternion q = slerp(place_pose.orientation, retreat_pose.orientation, t);
//       geometry_msgs::msg::Pose waypoint;
//       waypoint.position.x = interp.x();
//       waypoint.position.y = interp.y();
//       waypoint.position.z = interp.z();
//       waypoint.orientation = q;
//       waypoints.push_back(waypoint);
//     }

//     fraction = move_group.computeCartesianPath(waypoints, 0.005, 2.0, traj);
//     if (fraction < 0.9) {
//       RCLCPP_ERROR(LOGGER, "Failed Cartesian retreat (%.2f%%)", fraction * 100.0);
//       return;
//     }

//     grasp_plan.trajectory_ = traj;
//     move_group.execute(grasp_plan);
//     std::this_thread::sleep_for(std::chrono::seconds(1));
//   };

//   std::vector<geometry_msgs::msg::Pose> pick_poses(5);
//   std::vector<geometry_msgs::msg::Pose> place_poses(5);
//   std::vector<std::string> object_names = {"apple", "banana", "orange", "cup", "bottle"};

//   pick_poses[0].position.x = 0.68746;
//   pick_poses[0].position.y = -0.15068;
//   pick_poses[0].position.z = 0.43496;

//   pick_poses[1].position.x = 0.682;
//   pick_poses[1].position.y = -0.043;
//   pick_poses[1].position.z = 0.50;

//   pick_poses[2].position.x = 0.695;
//   pick_poses[2].position.y = -0.1;
//   pick_poses[2].position.z = 0.46;

//   pick_poses[3].position.x = 0.690;
//   pick_poses[3].position.y = -0.02;
//   pick_poses[3].position.z = 0.44;

//   pick_poses[4].position.x = 0.688;
//   pick_poses[4].position.y = 0.04;
//   pick_poses[4].position.z = 0.435;

//   for (auto &p : pick_poses) {
//     p.orientation.x = 0.995829;
//     p.orientation.y = -0.09123;
//     p.orientation.z = 0.0;
//     p.orientation.w = 0.0;
//   }

//   for (int i = 0; i < 5; ++i) {
//     place_poses[i].position.x = 0.37599;
//     place_poses[i].position.y = 0.22709 + 0.06 * i;
//     place_poses[i].position.z = 0.5;

//     place_poses[i].orientation.x = 1.0;
//     place_poses[i].orientation.y = 0.0;
//     place_poses[i].orientation.z = 0.0;
//     place_poses[i].orientation.w = 0.0;
//   }

//   for (int i = 0; i < 5; ++i) {
//     pick_and_place(pick_poses[i], place_poses[i], object_names[i]);
//   }

//   rclcpp::shutdown();
//   return 0;
// }























































































































// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <moveit/robot_state/conversions.h>

// #include <moveit_msgs/msg/display_robot_state.hpp>
// #include <moveit_msgs/msg/display_trajectory.hpp>

// #include "rclcpp/rclcpp.hpp"
// #include "control_msgs/action/gripper_command.hpp"
// #include "rclcpp_action/rclcpp_action.hpp"

// #include <thread>
// #include <rclcpp/duration.hpp>
// #include <Eigen/Geometry>

// static const rclcpp::Logger LOGGER = rclcpp::get_logger("test_trajectory");

// // Slerp interpolation function for orientation
// Eigen::Quaterniond slerp_orientation(const Eigen::Quaterniond& start, 
//                                       const Eigen::Quaterniond& end, 
//                                       double t) {
//     return start.slerp(t, end);
// }

// int main(int argc, char **argv) {
//   // Initialize ROS
//   rclcpp::init(argc, argv);
//   rclcpp::NodeOptions node_options;
//   node_options.automatically_declare_parameters_from_overrides(true);
//   auto move_group_node = rclcpp::Node::make_shared("test_trajectory", node_options);
//   auto gripper_client_node = rclcpp::Node::make_shared("gripper_client_node", node_options);

//   rclcpp::executors::SingleThreadedExecutor executor;
//   executor.add_node(move_group_node);
//   std::thread([&executor]() { executor.spin(); }).detach();

//   // Define planning groups
//   static const std::string PLANNING_GROUP = "panda_arm";
//   static const std::string PLANNING_GROUP_GRIPPER = "hand";

//   // Initialize MoveIt interfaces
//   moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
//   move_group.setPlannerId("RRTStarConfigDefault"); // Enforce the planner
//   move_group.setPlanningTime(50.0);               // Allow sufficient planning time

//   moveit::planning_interface::MoveGroupInterface move_group_gripper(gripper_client_node, PLANNING_GROUP_GRIPPER);

//   // Log planning frame and end-effector link
//   RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
//   RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

//   // Open the gripper before starting
//   std::vector<double> gripper_open = {0.035, 0.035}; // Fully open
//   move_group_gripper.setJointValueTarget(gripper_open);
//   move_group_gripper.setMaxVelocityScalingFactor(0.1);
//   move_group_gripper.setMaxAccelerationScalingFactor(0.1);
//   move_group_gripper.move();
//   std::this_thread::sleep_for(std::chrono::seconds(1));

//   // Function to pick and place an object with Slerp interpolation
//   auto pick_and_place = [&](const geometry_msgs::msg::Pose &pick_pose,
//                             const geometry_msgs::msg::Pose &place_pose,
//                             const std::string &object_name) {
//     RCLCPP_INFO(LOGGER, "Picking up the %s", object_name.c_str());

//     // Convert start and end orientations to Eigen quaternions for Slerp
//     Eigen::Quaterniond start_orientation(
//         pick_pose.orientation.w, 
//         pick_pose.orientation.x, 
//         pick_pose.orientation.y, 
//         pick_pose.orientation.z
//     );
//     Eigen::Quaterniond end_orientation(
//         place_pose.orientation.w, 
//         place_pose.orientation.x, 
//         place_pose.orientation.y, 
//         place_pose.orientation.z
//     );

//     // Define pre-grasp pose (slightly above the object)
//     geometry_msgs::msg::Pose pre_grasp_pose = pick_pose;
//     pre_grasp_pose.position.z += 0.07; // Adjust as needed

//     // Move to pre-grasp pose
//     move_group.setPoseTarget(pre_grasp_pose, "panda_hand");
//     moveit::planning_interface::MoveGroupInterface::Plan pre_grasp_plan;
//     bool success = (move_group.plan(pre_grasp_plan) == moveit::core::MoveItErrorCode::SUCCESS);
//     if (success) {
//       move_group.execute(pre_grasp_plan);
//     } else {
//       RCLCPP_ERROR(LOGGER, "Failed to plan to pre-grasp position");
//       return;
//     }
//     std::this_thread::sleep_for(std::chrono::seconds(1));

//     // Plan a Cartesian path from pre-grasp to grasp pose
//     std::vector<geometry_msgs::msg::Pose> waypoints;
//     geometry_msgs::msg::Pose grasp_pose = pre_grasp_pose;
//     grasp_pose.position.z -= 0.07; // Move down towards the object
//     waypoints.push_back(grasp_pose);

//     moveit_msgs::msg::RobotTrajectory cartesian_traj;
//     const double jump_threshold = 2.0;
//     const double eef_step = 0.005;
//     double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, cartesian_traj);
//     if (fraction < 0.9) {
//       RCLCPP_ERROR(LOGGER, "Unable to compute Cartesian path to grasp position. Only achieved %f%% of the path", fraction * 100.0);
//       return;
//     }

//     // Execute the Cartesian path
//     moveit::planning_interface::MoveGroupInterface::Plan grasp_plan;
//     grasp_plan.trajectory_ = cartesian_traj;
//     move_group.execute(grasp_plan);
//     std::this_thread::sleep_for(std::chrono::seconds(1));

//     // Close the gripper
//     std::vector<double> gripper_close = {0.02, 0.02}; // Adjust as needed
//     move_group_gripper.setJointValueTarget(gripper_close);
//     move_group_gripper.move();
//     std::this_thread::sleep_for(std::chrono::seconds(1));

//     // Lift the object by reversing the Cartesian path
//     waypoints.clear();
//     waypoints.push_back(pre_grasp_pose); // Move back to pre-grasp pose
//     fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, cartesian_traj);
//     if (fraction < 0.9) {
//       RCLCPP_ERROR(LOGGER, "Unable to compute Cartesian path to lift the object. Only achieved %f%% of the path", fraction * 100.0);
//       return;
//     }

//     grasp_plan.trajectory_ = cartesian_traj;
//     move_group.execute(grasp_plan);
//     std::this_thread::sleep_for(std::chrono::seconds(1));

//     // Create interpolated waypoints with Slerp for movement to place position
//     std::vector<geometry_msgs::msg::Pose> slerp_waypoints;
//     int interpolation_points = 20;
//     for (int i = 0; i <= interpolation_points; ++i) {
//         double t = static_cast<double>(i) / interpolation_points;
        
//         // Interpolate position linearly
//         geometry_msgs::msg::Pose interpolated_pose;
//         interpolated_pose.position.x = pre_grasp_pose.position.x + 
//             t * (place_pose.position.x - pre_grasp_pose.position.x);
//         interpolated_pose.position.y = pre_grasp_pose.position.y + 
//             t * (place_pose.position.y - pre_grasp_pose.position.y);
//         interpolated_pose.position.z = pre_grasp_pose.position.z + 
//             t * (place_pose.position.z - pre_grasp_pose.position.z);

//         // Interpolate orientation using Slerp
//         Eigen::Quaterniond interpolated_orientation = slerp_orientation(start_orientation, end_orientation, t);
//         interpolated_pose.orientation.w = interpolated_orientation.w();
//         interpolated_pose.orientation.x = interpolated_orientation.x();
//         interpolated_pose.orientation.y = interpolated_orientation.y();
//         interpolated_pose.orientation.z = interpolated_orientation.z();

//         slerp_waypoints.push_back(interpolated_pose);
//     }

//     // Compute and execute Cartesian path with interpolated waypoints
//     moveit_msgs::msg::RobotTrajectory slerp_trajectory;
//     fraction = move_group.computeCartesianPath(slerp_waypoints, eef_step, jump_threshold, slerp_trajectory);
//     if (fraction < 0.9) {
//         RCLCPP_ERROR(LOGGER, "Unable to compute Cartesian path with Slerp. Only achieved %f%% of the path", fraction * 100.0);
//         return;
//     }

//     moveit::planning_interface::MoveGroupInterface::Plan slerp_plan;
//     slerp_plan.trajectory_ = slerp_trajectory;
//     move_group.execute(slerp_plan);
//     std::this_thread::sleep_for(std::chrono::seconds(1));

//     // Open the gripper to release the object
//     move_group_gripper.setJointValueTarget(gripper_open);
//     move_group_gripper.move();
//     std::this_thread::sleep_for(std::chrono::seconds(1));

//     // Retreat by moving up using a Cartesian path
//     waypoints.clear();
//     geometry_msgs::msg::Pose retreat_pose = place_pose;
//     retreat_pose.position.z += 0.1; // Move up
//     waypoints.push_back(retreat_pose);

//     fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, cartesian_traj);
//     if (fraction < 0.9) {
//       RCLCPP_ERROR(LOGGER, "Unable to compute Cartesian path to retreat after placing the object. Only achieved %f%% of the path", fraction * 100.0);
//       return;
//     }

//     grasp_plan.trajectory_ = cartesian_traj;
//     move_group.execute(grasp_plan);
//     std::this_thread::sleep_for(std::chrono::seconds(1));
//   };

//   // Define the pick pose
//      // Define the banana pick pose
//   geometry_msgs::msg::Pose banana_pick_pose;
//   banana_pick_pose.orientation.x = 1.0;
//   banana_pick_pose.orientation.y = 0.0;
//   banana_pick_pose.orientation.z = 0.0;
//   banana_pick_pose.orientation.w = 0.0;
//   banana_pick_pose.position.x = 0.566;
//   banana_pick_pose.position.y = -0.015;
//   banana_pick_pose.position.z = 0.45;


//   // Define the apple pick pose

//   // Define the place pose (same for both)
//   geometry_msgs::msg::Pose place_pose;
//   place_pose.orientation.x = 1.0;
//   place_pose.orientation.y = 0.0;
//   place_pose.orientation.z = 0.0;
//   place_pose.orientation.w = 0.0;
//   place_pose.position.x = 0.464;
//   place_pose.position.y = 0.329;
//   place_pose.position.z = 0.42;

//   pick_and_place(banana_pick_pose, place_pose, "banana");

//   // geometry_msgs::msg::Pose bluecup_pick_pose;
//   // bluecup_pick_pose.orientation.x = 1.0;
//   // bluecup_pick_pose.orientation.y = 0.0;
//   // bluecup_pick_pose.orientation.z = -0.0;
//   // bluecup_pick_pose.orientation.w = 0.0;
//   // bluecup_pick_pose.position.x = 0.462;
//   // bluecup_pick_pose.position.y = -0.160;
//   // bluecup_pick_pose.position.z = 0.42;
//   // // Pick and place the banana

//   // // Optionally define a different place pose for the banana
//   // geometry_msgs::msg::Pose bluecup_place_pose = place_pose; // Same place as apple

//   // // Pick and place the banana
//   // pick_and_place(bluecup_pick_pose, bluecup_place_pose, "blue-cup");

 

//   // Shutdown ROS
//   rclcpp::shutdown();
//   return 0;
// }
