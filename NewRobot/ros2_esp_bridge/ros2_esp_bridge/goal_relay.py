#!/usr/bin/env python3
"""
Simple relay that converts /goal_pose topic to direct motor commands
For testing - bypasses Nav2 entirely
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist


class GoalRelay(Node):
    def __init__(self):
        super().__init__('goal_relay')
        
        # Subscribe to goal_pose topic (published by RViz 2D Goal Pose tool)
        self.goal_sub = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self._goal_callback,
            10
        )
        
        # Publisher for cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Timer for continuous command sending
        self.cmd_timer = None
        self.remaining_duration = 0.0
        self.command_active = False
        
        self.get_logger().info('Goal relay ready - converts /goal_pose to continuous 0.5 m/s for 5s')
    
    def _goal_callback(self, msg: PoseStamped):
        """Send continuous velocity commands for 5 seconds"""
        # Ignore if we're already running a command
        if self.command_active:
            self.get_logger().warn('Command already active, ignoring new goal')
            return
            
        self.get_logger().info(f'Received goal - sending 0.5 m/s forward for 5 seconds')
        self.command_active = True
        
        # Cancel any existing timer
        if self.cmd_timer is not None:
            self.cmd_timer.cancel()
        
        # Send commands at 10Hz for 5 seconds
        self.remaining_duration = 5.0
        self.cmd_timer = self.create_timer(0.1, self._send_cmd)
    
    def _send_cmd(self):
        """Send cmd_vel repeatedly"""
        if self.remaining_duration > 0:
            # Create cmd_vel message for straight forward motion
            cmd = Twist()
            cmd.linear.x = 0.5  # 0.5 m/s forward
            cmd.angular.z = 0.0  # No rotation
            
            # Publish command
            self.cmd_vel_pub.publish(cmd)
            self.remaining_duration -= 0.1
        else:
            # Stop the robot
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            
            # Cancel timer and reset flag
            if self.cmd_timer is not None:
                self.cmd_timer.cancel()
                self.cmd_timer = None
            self.command_active = False
            self.get_logger().info('Stopped - 5 seconds elapsed, ready for next goal')


def main(args=None):
    rclpy.init(args=args)
    node = GoalRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
