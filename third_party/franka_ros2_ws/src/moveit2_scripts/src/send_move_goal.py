#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from athena_exe_msgs.action import MoveToPose


class MoveToPoseClient(Node):

    def __init__(self):
        super().__init__('move_to_pose_client')
        self._action_client = ActionClient(self, MoveToPose, '/robot1/send_moveit_pose')
        self.goal_index = 0  # Track which goal we're on
        self.goals = self._create_goal_poses()  # List of poses

    def _create_goal_poses(self):
        """Create a list of PoseStamped objects to send as goals."""
        goals = []

        # 🟢 First goal
        pose1 = PoseStamped()
        pose1.header.frame_id = 'map'
        pose1.pose.position.x = 0.36
        pose1.pose.position.y = -0.32
        pose1.pose.position.z = 0.84
        pose1.pose.orientation.x = 0.718177
        pose1.pose.orientation.y = -0.029795
        pose1.pose.orientation.z = 0.693915
        pose1.pose.orientation.w = 0.042609
        goals.append(pose1)

        # 🟢 Second goal
        pose2 = PoseStamped()
        pose2.header.frame_id = 'map'
        pose2.pose.position.x = 0.595890
        pose2.pose.position.y = 0.070996
        pose2.pose.position.z = 0.543929
        pose2.pose.orientation.x = 0.953790
        pose2.pose.orientation.y = 0.017147
        pose2.pose.orientation.z = -0.298707
        pose2.pose.orientation.w = 0.027662
        goals.append(pose2)

        # 🟢 Back to home
        pose3 = PoseStamped()
        pose3.header.frame_id = 'map'
        pose3.pose.position.x = 0.306844
        pose3.pose.position.y = 0.000124
        pose3.pose.position.z = 0.588269
        pose3.pose.orientation.x = 0.999973
        pose3.pose.orientation.y = -0.004091
        pose3.pose.orientation.z = -0.005858
        pose3.pose.orientation.w = -0.001642
        goals.append(pose3)

        return goals

    def send_next_goal(self):
        if self.goal_index >= len(self.goals):
            self.get_logger().info('🎯 All goals completed.')
            rclpy.shutdown()
            return

        # Delay sending goal by 1 second using ROS timer
        self.get_logger().info(f'⏳ Waiting 1 second before sending goal #{self.goal_index + 1}...')
        self.timer = self.create_timer(8.0, self._send_delayed_goal)

    def _send_delayed_goal(self):
        # Cancel the timer so it doesn't repeat
        self.timer.cancel()

        goal_pose = self.goals[self.goal_index]
        goal_pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg = MoveToPose.Goal()
        goal_msg.pose = goal_pose
        goal_msg.behavior_tree = ''  # Add behavior tree string if needed

        # Wait for server
        self._action_client.wait_for_server()
        self.get_logger().info(f'🚀 Sending goal #{self.goal_index + 1}...')

        # Send async goal
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'❌ Goal #{self.goal_index + 1} rejected')
            self.goal_index += 1
            self.send_next_goal()
            return

        self.get_logger().info(f'✅ Goal #{self.goal_index + 1} accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'🏁 Goal #{self.goal_index + 1} finished!')
        self.goal_index += 1
        self.send_next_goal()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        pose = feedback.current_pose.pose
        nav_time = feedback.navigation_time
        eta = feedback.estimated_time_remaining

        self.get_logger().info(
            f'📍 At ({pose.position.x:.2f}, {pose.position.y:.2f}), '
            f'Distance left: {feedback.distance_remaining:.2f} m, '
            f'Time nav: {nav_time.sec}s, ETA: {eta.sec}s, '
            f'Recoveries: {feedback.number_of_recoveries}'
        )


def main(args=None):
    rclpy.init(args=args)
    client = MoveToPoseClient()
    client.send_next_goal()
    rclpy.spin(client)


if __name__ == '__main__':
    main()
