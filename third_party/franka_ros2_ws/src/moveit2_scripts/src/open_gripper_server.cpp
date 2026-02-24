#include <functional>
#include <memory>
#include <thread>

#include "athena_exe_msgs/action/move_joint.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "control_msgs/action/gripper_command.hpp" 
#include "rclcpp/parameter_client.hpp"
class OpenGripperActionServer : public rclcpp::Node
{
public:
  using MoveJoint = athena_exe_msgs::action::MoveJoint;
  using GoalHandleMoveJoint = rclcpp_action::ServerGoalHandle<MoveJoint>;

  explicit OpenGripperActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("move_to_pose_action_server", options)
  {
    using namespace std::placeholders;

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
      "/robot1/send_dump",
      std::bind(&OpenGripperActionServer::handle_goal, this, _1, _2),
      std::bind(&OpenGripperActionServer::handle_cancel, this, _1),
      std::bind(&OpenGripperActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<MoveJoint>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveJoint::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal to open the gripper!");
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
    std::thread{std::bind(&OpenGripperActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleMoveJoint> goal_handle)
  {
      // Open Gripper
    RCLCPP_INFO(this->get_logger(), "Opening gripper");

    // It's crucial to obtain the current state to ensure the vectors have the correct size
    // std::vector<double> joint_group_positions_gripper;
    // move_group_gripper.getCurrentState()->copyJointGroupPositions(move_group_gripper.getCurrentState()->getJointModelGroup(PLANNING_GROUP_GRIPPER), joint_group_positions_gripper);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto gripper_client_node = rclcpp::Node::make_shared("gripper_client_node", node_options);

    rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr action_client; 
    action_client = rclcpp_action::create_client<control_msgs::action::GripperCommand>(gripper_client_node,"/panda_gripper/gripper_action");


    // Assuming the gripper has two joints indexed 0 and 1 in its joint group 
    auto is_action_server_ready = action_client->wait_for_action_server(std::chrono::seconds(5));
    if(!is_action_server_ready){
      RCLCPP_ERROR(this->get_logger(), "close gripper action server not ready!");
    }
    auto goal_msg = control_msgs::action::GripperCommand::Goal();
    goal_msg.command.max_effort = 30;
    goal_msg.command.position = 0.04;

    auto future_goal_handle = action_client->async_send_goal(goal_msg);
    if(rclcpp::spin_until_future_complete(gripper_client_node,future_goal_handle) != rclcpp::FutureReturnCode::SUCCESS){
      RCLCPP_INFO(this->get_logger(), "Failed send goal");
    }

    auto send_close_handler = future_goal_handle.get();
    auto future_result = action_client->async_get_result(send_close_handler);

    // rclcpp::sleep_for(std::chrono::seconds(5));
    auto result = std::make_shared<MoveJoint::Result>();

    // Check if goal is done
    if (rclcpp::ok()) {
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
};  

RCLCPP_COMPONENTS_REGISTER_NODE(OpenGripperActionServer)