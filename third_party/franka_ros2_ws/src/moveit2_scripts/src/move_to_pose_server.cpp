#include <functional>
#include <memory>
#include <thread>
#include <cmath>

#include "athena_exe_msgs/action/move_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"


#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include "rclcpp/rclcpp.hpp"
#include "control_msgs/action/gripper_command.hpp" // Adjust this include based on the actual location
#include "rclcpp_action/rclcpp_action.hpp"

#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <rclcpp/duration.hpp>


#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.h"
#include "tf2/exceptions.h"

#include "rclcpp/parameter_client.hpp"






class MoveToPoseActionServer : public rclcpp::Node
{
public:
  using MoveToPose = athena_exe_msgs::action::MoveToPose;
  using GoalHandleMoveToPose = rclcpp_action::ServerGoalHandle<MoveToPose>;
      std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
  geometry_msgs::msg::Pose goal_;

  explicit MoveToPoseActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("move_to_pose_action_server", options)
  {
    using namespace std::placeholders;

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    this->action_server_ = rclcpp_action::create_server<MoveToPose>(
      this,
      "/robot1/send_moveit_pose",
      std::bind(&MoveToPoseActionServer::handle_goal, this, _1, _2),
      std::bind(&MoveToPoseActionServer::handle_cancel, this, _1),
      std::bind(&MoveToPoseActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<MoveToPose>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveToPose::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal to move to pose (%f, %f, %f) with orientation (%f, %f,%f,%f)", 
    goal->pose.pose.position.x,goal->pose.pose.position.y,goal->pose.pose.position.z,
    goal->pose.pose.orientation.x,goal->pose.pose.orientation.y,goal->pose.pose.orientation.z,goal->pose.pose.orientation.w);
    goal_ = goal->pose.pose;
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&MoveToPoseActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing Moving action");
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<MoveToPose::Result>();
    
    auto now = this->get_clock()->now();
    geometry_msgs::msg::TransformStamped transform_stamped = 
      tf_buffer_->lookupTransform("panda_link0", "panda_hand", now, tf2::durationFromSec(5.0));

    RCLCPP_INFO(this->get_logger(), "Current position hand (%f, %f, %f)", 
    transform_stamped.transform.translation.x,transform_stamped.transform.translation.y,transform_stamped.transform.translation.z);

    RCLCPP_INFO(this->get_logger(), "Goal position hand (%f, %f, %f)", 
    goal->pose.pose.position.x,goal->pose.pose.position.y,goal->pose.pose.position.z);

    auto dist_sq = std::pow((transform_stamped.transform.translation.x - goal->pose.pose.position.x), 2) + std::pow((transform_stamped.transform.translation.y - goal->pose.pose.position.y), 2) + std::pow((transform_stamped.transform.translation.z - goal->pose.pose.position.z), 2);
    auto dist = std::sqrt(dist_sq);
    RCLCPP_INFO(this->get_logger(), "distance: (%f)", dist); 

    if (dist<0.02){
      RCLCPP_INFO(this->get_logger(), "Start == Goal.");
     
        if (rclcpp::ok()) {
          goal_handle->succeed(result);
          RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }

    }


    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("test_trajectory", node_options);


    auto param_client = std::make_shared<rclcpp::SyncParametersClient>(move_group_node, "move_group");

    if (param_client->wait_for_service(std::chrono::seconds(5))) {
        if (param_client->has_parameter("robot_description_semantic")) {
            std::string srdf_string = param_client->get_parameter<std::string>("robot_description_semantic");

            // ✅ Declare it locally before setting it
            if (!move_group_node->has_parameter("robot_description_semantic")) {
                move_group_node->declare_parameter("robot_description_semantic", srdf_string);
            } else {
                move_group_node->set_parameter(rclcpp::Parameter("robot_description_semantic", srdf_string));
            }

            RCLCPP_INFO(move_group_node->get_logger(), "robot_description_semantic parameter loaded successfully");
        } else {
            RCLCPP_ERROR(move_group_node->get_logger(), "Parameter not declared on move_group");
        }
    } else {
        RCLCPP_ERROR(move_group_node->get_logger(), "Timed out waiting for move_group parameter service");
    }
  
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    static const std::string PLANNING_GROUP = "panda_arm";
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

    const moveit::core::JointModelGroup *joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    RCLCPP_INFO(this->get_logger(), "Planning frame: %s", move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(this->get_logger(), "End effector link: %s", move_group.getEndEffectorLink().c_str());

    // Sending goal to gripper action server  


    RCLCPP_INFO(this->get_logger(), "Available Planning Groups:");
    std::copy(move_group.getJointModelGroupNames().begin(),
              move_group.getJointModelGroupNames().end(),
              std::ostream_iterator<std::string>(std::cout, ", "));

    moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);

    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);


    RCLCPP_INFO(this->get_logger(), "Pregrasp Position");
    RCLCPP_INFO(this->get_logger(), "Goal position pregrasp (%f, %f, %f)", 
    goal->pose.pose.position.x,goal->pose.pose.position.y,goal->pose.pose.position.z);
 
    geometry_msgs::msg::Pose target_pose1;
     // geometry_msgs::msg::Pose target_pose2;
    // target_pose1.orientation.x = 0.999952;
    // target_pose1.orientation.y = -0.009361;
    // target_pose1.orientation.z = -0.002807;
    // target_pose1.orientation.w =   0.000262;
    // target_pose1.position.x =  0.472053;
    // target_pose1.position.y =  -0.011588;
    // target_pose1.position.z =  0.555449;

    target_pose1.position.x = goal_.position.x;
    target_pose1.position.y = goal_.position.y;
    target_pose1.position.z = goal_.position.z;

    target_pose1.orientation.x = goal_.orientation.x;
    target_pose1.orientation.y = goal_.orientation.y;
    target_pose1.orientation.z = goal_.orientation.z;
    target_pose1.orientation.w = goal_.orientation.w;

    // target_pose1.orientation.x = 1.;
    // target_pose1.orientation.y = 0.;
    // target_pose1.orientation.z = 0.;
    // target_pose1.orientation.w = 0.;


    move_group.setPlannerId("RRTStarkConfigDefault");
    move_group.setPoseTarget(target_pose1, "panda_hand");
    move_group.setPlanningTime(50);
    // move_group.setMaxVelocityScalingFactor(0.18);

      // Define the joint constraint
    moveit_msgs::msg::JointConstraint joint_constraint;
    joint_constraint.joint_name = "panda_joint5"; // Elbow joint
    joint_constraint.position = -0.0; // Up elbow position in radians
    joint_constraint.tolerance_above = 0.5;
    joint_constraint.tolerance_below = 0.5;
    joint_constraint.weight = 0.8;

    moveit_msgs::msg::JointConstraint joint_constraint1;
    joint_constraint.joint_name = "panda_joint1";
    joint_constraint.position = -0.0; 
    joint_constraint.tolerance_above = 1.4;
    joint_constraint.tolerance_below = 1.4;
    joint_constraint.weight = 0.8;

    // moveit_msgs::msg::JointConstraint joint_constraint1;
    // joint_constraint.joint_name = "panda_joint0"; // Elbow joint
    // joint_constraint.position = -0.0; // Up elbow position in radians
    // joint_constraint.tolerance_above = 0.5;
    // joint_constraint.tolerance_below = 0.5;
    // joint_constraint.weight = 0.8;

    moveit_msgs::msg::Constraints path_constraints;
    path_constraints.joint_constraints.push_back(joint_constraint);
    path_constraints.joint_constraints.push_back(joint_constraint1);
    move_group.setPathConstraints(path_constraints);


    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
      RCLCPP_INFO(this->get_logger(), "Planning successful, executing...");
      move_group.execute(my_plan);
    } 
    else {
      RCLCPP_ERROR(this->get_logger(), "Planning failed!");
    }
    RCLCPP_INFO(this->get_logger(), "Reached goal Location!");

    //rclcpp::sleep_for(std::chrono::seconds(5));
    result = std::make_shared<MoveToPose::Result>();
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    // // Check if goal is done
    //  if (rclcpp::ok()) {
    //    goal_handle->succeed(result);
    //    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    // }
  }
};  

RCLCPP_COMPONENTS_REGISTER_NODE(MoveToPoseActionServer)