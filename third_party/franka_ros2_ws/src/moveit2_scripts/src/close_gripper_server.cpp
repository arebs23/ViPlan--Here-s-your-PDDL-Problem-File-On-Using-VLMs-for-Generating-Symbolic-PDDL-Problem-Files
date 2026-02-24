#include <functional>
#include <memory>
#include <thread>

#include "athena_exe_msgs/action/move_joint.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include "control_msgs/action/gripper_command.hpp" 


#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.h"
#include "tf2/exceptions.h"
#include "rclcpp/parameter_client.hpp"
class CloseGripperActionServer : public rclcpp::Node
{
public:
  using MoveJoint = athena_exe_msgs::action::MoveJoint;
  using GoalHandleMoveJoint = rclcpp_action::ServerGoalHandle<MoveJoint>;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
  explicit CloseGripperActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("move_to_pose_action_server", options)
  {
    using namespace std::placeholders;
     tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    auto param_client = std::make_shared<rclcpp::SyncParametersClient>(this, "move_group");

    if (param_client->wait_for_service(std::chrono::seconds(5))) {
        if (param_client->has_parameter("robot_description_semantic")) {
            std::string srdf_string = param_client->get_parameter<std::string>("robot_description_semantic");

            // ✅ Declare it locally before setting it
            if (!this->has_parameter("robot_description_semantic")) {
                this->declare_parameter("robot_description_semantic", srdf_string);
            } else {
                this->set_parameter(rclcpp::Parameter("robot_description_semantic", srdf_string));
            }

            RCLCPP_INFO(this->get_logger(), "robot_description_semantic parameter loaded successfully");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Parameter not declared on move_group");
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Timed out waiting for move_group parameter service");
    }


    this->action_server_ = rclcpp_action::create_server<MoveJoint>(
      this,
      "/robot1/send_load",
      std::bind(&CloseGripperActionServer::handle_goal, this, _1, _2),
      std::bind(&CloseGripperActionServer::handle_cancel, this, _1),
      std::bind(&CloseGripperActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<MoveJoint>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveJoint::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal to close the gripper!");
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleMoveJoint> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMoveJoint> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&CloseGripperActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleMoveJoint> goal_handle)
  {
      // Close Gripper

      auto now = this->get_clock()->now();
    geometry_msgs::msg::TransformStamped transform_stamped = 
      tf_buffer_->lookupTransform("panda_link0", "panda_hand", now, tf2::durationFromSec(5.0));


    auto curr_x = transform_stamped.transform.translation.x;
    auto curr_y = transform_stamped.transform.translation.y;

    auto curr_qx = transform_stamped.transform.rotation.x;
    auto curr_qy = transform_stamped.transform.rotation.y;
    auto curr_qz = transform_stamped.transform.rotation.z;
    auto curr_qw = transform_stamped.transform.rotation.w;
      
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto gripper_client_node = rclcpp::Node::make_shared("gripper_client_node", node_options);
    auto move_to_grasp_client_node = rclcpp::Node::make_shared("move_to_grasp_client_node", node_options);

    rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr action_client; 
    
    action_client = rclcpp_action::create_client<control_msgs::action::GripperCommand>(gripper_client_node,"/panda_gripper/gripper_action");
    RCLCPP_INFO(this->get_logger(), "Move to Pregrasp position");


    static const std::string PLANNING_GROUP = "panda_arm";
    moveit::planning_interface::MoveGroupInterface move_group(move_to_grasp_client_node, PLANNING_GROUP);

    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation.x = 1.;
    target_pose.orientation.y = 0.;
    target_pose.orientation.z = 0.;
    target_pose.orientation.w = 0.;
    target_pose.position.x = curr_x;
    target_pose.position.y = curr_y;
    target_pose.position.z = 0.435;

    move_group.setPlannerId("RRTStarkConfigDefault");
    move_group.setPoseTarget(target_pose, "panda_hand");
    move_group.setPlanningTime(50);


    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success2 = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success2) {
      RCLCPP_INFO(this->get_logger(), "Planning successful, executing...");
      move_group.execute(my_plan);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Planning failed!");

    }

    RCLCPP_INFO(this->get_logger(), "Closing gripper");


    // Create the move group interface for the panda hand (gripper)
      moveit::planning_interface::MoveGroupInterface move_group_gripper(gripper_client_node, "hand");

      // Define a target width for the gripper to close to (in meters)
      std::vector<double> gripper_target = {0.03, 0.03}; // Fully closed

      // Set the gripper target positions
      move_group_gripper.setJointValueTarget(gripper_target);
      move_group_gripper.setMaxVelocityScalingFactor(0.1);
      move_group_gripper.setMaxAccelerationScalingFactor(0.1);

      // Execute the motion
      bool successGripper = (move_group_gripper.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);


      if (successGripper)
      {
          RCLCPP_INFO(this->get_logger(), "Gripper closed successfully.");
      }
      else
      {
          RCLCPP_ERROR(this->get_logger(), "Failed to close the gripper.");
      }

    // // Assuming the gripper has two joints indexed 0 and 1 in its joint group 
    // auto is_action_server_ready = action_client->wait_for_action_server(std::chrono::seconds(5));
    // if(!is_action_server_ready){
    //   RCLCPP_ERROR(this->get_logger(), "close gripper action server not ready!");
    // }
    // auto goal_msg = control_msgs::action::GripperCommand::Goal();
    // goal_msg.command.max_effort = 1000;
    // goal_msg.command.position = 0.02;

    // auto future_goal_handle = action_client->async_send_goal(goal_msg);
    // if(rclcpp::spin_until_future_complete(gripper_client_node,future_goal_handle) != rclcpp::FutureReturnCode::SUCCESS){
    //   RCLCPP_INFO(this->get_logger(), "Failed send goal");
    // }

    // auto send_close_handler = future_goal_handle.get();
    // auto future_result = action_client->async_get_result(send_close_handler);
    // rclcpp::spin_until_future_complete(gripper_client_node,future_result);
    


    // // rclcpp::sleep_for(std::chrono::seconds(5));
    // auto result = std::make_shared<MoveJoint::Result>();

    // // Check if goal is done
    // if (rclcpp::ok()) {
    //   goal_handle->succeed(result);
    //   RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    // }

     RCLCPP_INFO(this->get_logger(), "Move to Postgrasp position");
  
    geometry_msgs::msg::Pose target_pose3;
    target_pose3.orientation.x = 1.;
    target_pose3.orientation.y = 0.;
    target_pose3.orientation.z = 0.;
    target_pose3.orientation.w = 0.;
    target_pose3.position.x = curr_x;
    target_pose3.position.y = curr_y;
    target_pose3.position.z = 0.56;

    move_group.setPlannerId("RRTStarkConfigDefault");
    move_group.setPoseTarget(target_pose3, "panda_hand");
    move_group.setPlanningTime(50);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan2;

    bool success3 = (move_group.plan(my_plan2) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success3) {
      RCLCPP_INFO(this->get_logger(), "Planning successful, executing...");
      move_group.execute(my_plan2);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Planning failed!");

    }

    //rclcpp::sleep_for(std::chrono::seconds(5));
    auto result = std::make_shared<MoveJoint::Result>();
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");

  }
};  

RCLCPP_COMPONENTS_REGISTER_NODE(CloseGripperActionServer)