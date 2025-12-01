#!/usr/bin/env python3
"""
Goal Navigator - Autonomous navigation with obstacle avoidance and goal actions
Uses Nav2 for path planning and obstacle avoidance
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
import math
import time


class GoalNavigator(Node):
    def __init__(self):
        super().__init__('goal_navigator')
        
        # Parameters
        self.declare_parameter('goal_action', 'spin')  # Action to perform at goal
        self.declare_parameter('spin_duration', 3.0)  # seconds
        self.goal_action = self.get_parameter('goal_action').value
        self.spin_duration = self.get_parameter('spin_duration').value
        
        # Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Publisher for goal actions (e.g., spinning)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Goal queue
        self.goal_queue = []
        self.current_goal = None
        self.goal_handle = None
        
        self.get_logger().info('Goal Navigator ready')
        self.get_logger().info('Waiting for Nav2 action server...')
        self.nav_client.wait_for_server()
        self.get_logger().info('Nav2 action server connected!')
    
    def add_goal(self, x, y, theta=0.0):
        """Add a goal to the navigation queue"""
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0
        
        # Convert theta to quaternion
        goal_pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_pose.pose.orientation.w = math.cos(theta / 2.0)
        
        self.goal_queue.append(goal_pose)
        self.get_logger().info(f'Goal added: ({x:.2f}, {y:.2f}, {math.degrees(theta):.1f}°)')
        
        # Start navigation if not already navigating
        if self.current_goal is None:
            self.navigate_next_goal()
    
    def navigate_next_goal(self):
        """Navigate to the next goal in the queue"""
        if not self.goal_queue:
            self.get_logger().info('No more goals in queue')
            self.current_goal = None
            return
        
        self.current_goal = self.goal_queue.pop(0)
        
        self.get_logger().info(f'Navigating to goal: '
                             f'({self.current_goal.pose.position.x:.2f}, '
                             f'{self.current_goal.pose.position.y:.2f})')
        
        # Create navigation goal
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = self.current_goal
        
        # Send goal to Nav2
        send_goal_future = self.nav_client.send_goal_async(
            nav_goal,
            feedback_callback=self.navigation_feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection"""
        self.goal_handle = future.result()
        
        if not self.goal_handle.accepted:
            self.get_logger().error('Goal rejected by Nav2!')
            self.navigate_next_goal()
            return
        
        self.get_logger().info('Goal accepted by Nav2')
        
        # Wait for result
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)
    
    def navigation_feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        # Can be used to monitor progress
        pass
    
    def navigation_result_callback(self, future):
        """Handle navigation result"""
        result = future.result()
        status = result.status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal reached successfully!')
            self.perform_goal_action()
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error('Navigation aborted!')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('Navigation canceled!')
        else:
            self.get_logger().warn(f'Navigation ended with status: {status}')
        
        # Navigate to next goal
        time.sleep(1.0)  # Brief pause
        self.navigate_next_goal()
    
    def perform_goal_action(self):
        """Perform action upon reaching goal"""
        if self.goal_action == 'spin':
            self.spin_360()
        # Add more actions here as needed
        # elif self.goal_action == 'release_candy':
        #     self.release_candy()
        # elif self.goal_action == 'play_sound':
        #     self.play_sound()
    
    def spin_360(self):
        """Spin 360 degrees at goal"""
        self.get_logger().info('Performing goal action: Spinning 360°')
        
        twist = Twist()
        twist.angular.z = 1.0  # 1 rad/s
        
        # Calculate time needed for 360 degrees
        angular_speed = 1.0  # rad/s
        angle_to_rotate = 2 * math.pi  # 360 degrees in radians
        duration = angle_to_rotate / angular_speed
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        
        # Stop spinning
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        
        self.get_logger().info('Spin complete!')
    
    def cancel_current_goal(self):
        """Cancel current navigation goal"""
        if self.goal_handle is not None:
            self.get_logger().info('Canceling current goal')
            self.goal_handle.cancel_goal_async()
    
    def clear_goals(self):
        """Clear all goals from queue"""
        self.goal_queue.clear()
        self.get_logger().info('Goal queue cleared')


def main(args=None):
    rclpy.init(args=args)
    navigator = GoalNavigator()
    
    print("\n" + "="*50)
    print("GOAL NAVIGATOR")
    print("="*50)
    print("Commands:")
    print("  goal <x> <y> [theta]  - Add navigation goal")
    print("  cancel                - Cancel current goal")
    print("  clear                 - Clear goal queue")
    print("  quit                  - Exit")
    print("\nExample: goal 1.0 0.5 90")
    
    import threading
    
    def spin_thread():
        rclpy.spin(navigator)
    
    spinner = threading.Thread(target=spin_thread, daemon=True)
    spinner.start()
    
    try:
        while rclpy.ok():
            cmd = input("\nCommand: ").strip().split()
            
            if not cmd:
                continue
            
            if cmd[0] == 'quit':
                break
            elif cmd[0] == 'goal':
                if len(cmd) < 3:
                    print("Usage: goal <x> <y> [theta_degrees]")
                else:
                    try:
                        x = float(cmd[1])
                        y = float(cmd[2])
                        theta = math.radians(float(cmd[3])) if len(cmd) > 3 else 0.0
                        navigator.add_goal(x, y, theta)
                    except ValueError:
                        print("Invalid coordinates!")
            elif cmd[0] == 'cancel':
                navigator.cancel_current_goal()
            elif cmd[0] == 'clear':
                navigator.clear_goals()
            else:
                print("Unknown command!")
    
    except KeyboardInterrupt:
        pass
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
