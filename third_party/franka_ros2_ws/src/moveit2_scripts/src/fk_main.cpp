// #include "rclcpp/rclcpp.hpp"
// #include "moveit_msgs/srv/get_position_fk.hpp"
// #include <chrono>
// #include <cstdlib>
// #include <memory>

// using namespace std::chrono_literals;

// class JointSubscriber : public rclcpp::Node
// {
//   public:
//     JointSubscriber()
//     : Node("minimal_subscriber")
//     { 
//       subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
//       "joint_states", 10, std::bind(&JointSubscriber::joint_position_callback, this, std::placeholders::_1));
//       node_ = rclcpp::Node::make_shared("compute_fk_main");
//       client_ = node_->create_client<moveit_msgs::srv::GetPositionFK>("compute_fk");
//     }

//   private:
//     void joint_position_callback(const sensor_msgs::msg::JointState & msg) const
//     {
//       RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
   
//   auto request = std::make_shared<moveit_msgs::srv::GetPositionFK::Request>();
//   std::vector<std::string> joints;
//   joints.push_back("hand");
//   request->fk_link_names = joints;
//   request->robot_state.joint_state = msg;


//   while (!client_->wait_for_service(1s)) {
//     if (!rclcpp::ok()) {
//       RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      
//     }
//     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
//   }

//   auto result = client_->async_send_request(request);
//   // Wait for the result.
//   if (rclcpp::spin_until_future_complete(node_, result) ==
//     rclcpp::FutureReturnCode::SUCCESS)
//   {
//       auto ee_pos = result.get()->pose_stamped[0];
//     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "EE Position: (%f,%f,%f) and Orientation (%f,%f,%f, %f)", 
//     ee_pos.pose.position.x,
//     ee_pos.pose.position.y,
//     ee_pos.pose.position.z,
//     ee_pos.pose.orientation.x,
//     ee_pos.pose.orientation.y,
//     ee_pos.pose.orientation.z,
//     ee_pos.pose.orientation.w);
//   } else {
//     RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
//   }

//     }
//     rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
//      rclcpp::Client<moveit_msgs::srv::GetPositionFK>::SharedPtr client_;
//      std::shared_ptr<rclcpp::Node> node_;
//      int cont =1;
    
// };

// int main(int argc, char **argv)
// {
//   rclcpp::init(argc, argv);

//   rclcpp::spin(std::make_shared<JointSubscriber>());
 
//   rclcpp::shutdown();
//   return 0;
// }
#include "rclcpp/rclcpp.hpp"
#include "moveit_msgs/srv/get_position_fk.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

class JointSubscriber : public rclcpp::Node {
public:
    JointSubscriber()
        : Node("joint_subscriber_node") {
        // Initialize the subscription to the joint states topic.
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10, std::bind(&JointSubscriber::joint_position_callback, this, std::placeholders::_1));

        // Initialize the FK service client
        fk_client_ = this->create_client<moveit_msgs::srv::GetPositionFK>("compute_fk");

        // Wait for the FK service to be available.
        RCLCPP_INFO(this->get_logger(), "Waiting for FK service to become available...");
        fk_client_->wait_for_service();
        RCLCPP_INFO(this->get_logger(), "FK service is now available.");
    }

private:
   void joint_position_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (!fk_client_->service_is_ready()) {
        RCLCPP_ERROR(this->get_logger(), "FK service not available.");
        return;
    }

    auto request = std::make_shared<moveit_msgs::srv::GetPositionFK::Request>();
    request->fk_link_names = {"panda_hand"}; // Ensure this is a link name expected by the FK service
    request->robot_state.joint_state = *msg;

    // Debugging: Log the request details
    RCLCPP_INFO(this->get_logger(), "Sending FK request for link: %s", request->fk_link_names[0].c_str());
    for (size_t i = 0; i < request->robot_state.joint_state.name.size(); ++i) {
        RCLCPP_INFO(this->get_logger(), "Joint %s: Position %f", request->robot_state.joint_state.name[i].c_str(), request->robot_state.joint_state.position[i]);
    }

    // Asynchronous service call with callback
    using ServiceResponseFuture = rclcpp::Client<moveit_msgs::srv::GetPositionFK>::SharedFuture;
    auto response_received_callback = [this](ServiceResponseFuture future) {
        auto response = future.get();
        if (!response->pose_stamped.empty()) {
            const auto& ee_pos = response->pose_stamped[0].pose;
            RCLCPP_INFO(this->get_logger(), "EE Position: (%f, %f, %f) and Orientation (%f, %f, %f, %f)",
                        ee_pos.position.x, ee_pos.position.y, ee_pos.position.z,
                        ee_pos.orientation.x, ee_pos.orientation.y, ee_pos.orientation.z, ee_pos.orientation.w);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to compute FK: Response is empty.");
        }
    };

    fk_client_->async_send_request(request, response_received_callback);
}


    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Client<moveit_msgs::srv::GetPositionFK>::SharedPtr fk_client_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
