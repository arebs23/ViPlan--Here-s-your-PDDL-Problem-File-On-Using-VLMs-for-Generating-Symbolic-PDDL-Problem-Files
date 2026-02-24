#include <functional>
#include <memory>
#include <thread>
#include <cmath>
#include <string>

#include "athena_exe_msgs/action/move_to_pose.hpp"
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

// VISION INCLUDE
#include "object_detection_msgs/srv/get_object_position.hpp" 
#include "std_msgs/msg/string.hpp"

class MoveToPoseActionServer : public rclcpp::Node
{
public:
  using MoveToPose = athena_exe_msgs::action::MoveToPose;
  using GoalHandleMoveToPose = rclcpp_action::ServerGoalHandle<MoveToPose>;
  using VisionService = object_detection_msgs::srv::GetObjectPosition; 

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
  geometry_msgs::msg::Pose goal_;
  
  rclcpp::Client<VisionService>::SharedPtr vision_client_; 

  explicit MoveToPoseActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("move_to_pose_server", options)
  {
    using namespace std::placeholders;

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // -------------------------------------------------------------
    // 1. DECLARE PARAMETER (Default to "apple" just in case)
    // -------------------------------------------------------------
    this->declare_parameter<std::string>("target_object", "apple");

    // Initialize Vision Client
    vision_client_ = this->create_client<VisionService>("get_object_positions");

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
    RCLCPP_INFO(this->get_logger(), "Received Action Goal. Will check vision system first...");
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
    std::thread{std::bind(&MoveToPoseActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Starting execution sequence...");
    auto result = std::make_shared<MoveToPose::Result>();

    // =================================================================================
    // 2. READ PARAMETER AND CALL VISION SERVICE
    // =================================================================================
    
    if (!vision_client_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(this->get_logger(), "Vision service not available! Moving to hardcoded goal instead?");
    } 
    else {
        // --- RETRIEVE PARAMETER ---
        std::string target_object_name;
        this->get_parameter("target_object", target_object_name);

        // Prepare the request
        auto request = std::make_shared<VisionService::Request>();
        std_msgs::msg::String obj_name;
        
        // Use the variable from the parameter server
        obj_name.data = target_object_name; 
        request->objects_names.push_back(obj_name);

        RCLCPP_INFO(this->get_logger(), "Asking Vision System: Where is the '%s'?", obj_name.data.c_str());

        auto result_future = vision_client_->async_send_request(request);

        if (result_future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
            auto vision_response = result_future.get();
            
            if (!vision_response->objects_positions.empty()) {
                // SUCCESS! We found the object.
                geometry_msgs::msg::Pose detected_pose = vision_response->objects_positions[0];
                
                // --- SAFETY UPDATE: ADD HOVER OFFSET ---
                goal_.position.x = detected_pose.position.x;
                goal_.position.y = detected_pose.position.y;
                // Add 0.20 meters (20cm) to Z so we don't crash into the object
                goal_.position.z = detected_pose.position.z + 0.20; 
                
                // Use vision orientation
                goal_.orientation = detected_pose.orientation;

                RCLCPP_INFO(this->get_logger(), "✅ Vision Updated Target -> X: %.3f, Y: %.3f, Z: %.3f (Includes +0.20m hover)", 
                    goal_.position.x, goal_.position.y, goal_.position.z);
            } else {
                RCLCPP_WARN(this->get_logger(), "⚠️ Vision returned 0 objects. Proceeding with original Action goal.");
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "❌ Vision service timed out.");
        }
    }
    // =================================================================================

    // The rest is your original MoveIt logic
    
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("test_trajectory", node_options);

    auto param_client = std::make_shared<rclcpp::SyncParametersClient>(move_group_node, "move_group");
    
    // Check robot_description parameters
    if (param_client->wait_for_service(std::chrono::seconds(5))) {
        if (param_client->has_parameter("robot_description_semantic")) {
            std::string srdf_string = param_client->get_parameter<std::string>("robot_description_semantic");
            if (!move_group_node->has_parameter("robot_description_semantic")) {
                move_group_node->declare_parameter("robot_description_semantic", srdf_string);
            } else {
                move_group_node->set_parameter(rclcpp::Parameter("robot_description_semantic", srdf_string));
            }
        }
    }

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    static const std::string PLANNING_GROUP = "panda_arm";
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

    // Set Target
    geometry_msgs::msg::Pose target_pose1;
    target_pose1.position.x = goal_.position.x; 
    target_pose1.position.y = goal_.position.y;
    target_pose1.position.z = goal_.position.z;
    target_pose1.orientation = goal_.orientation;

    move_group.setPlannerId("RRTStarkConfigDefault");
    move_group.setPoseTarget(target_pose1, "panda_hand");
    move_group.setPlanningTime(10.0); 

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
      RCLCPP_INFO(this->get_logger(), "Planning successful, executing...");
      move_group.execute(my_plan);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
      goal_handle->succeed(result);
    } 
    else {
      RCLCPP_ERROR(this->get_logger(), "Planning failed!");
      goal_handle->abort(result);
    }
  }
};  

RCLCPP_COMPONENTS_REGISTER_NODE(MoveToPoseActionServer)