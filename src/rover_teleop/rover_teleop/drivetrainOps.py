#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoyToTwist(Node):
    def __init__(self):
        super().__init__('joy_to_twist')
        
        # Publishers
        self.twist_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.joy_sub = self.create_subscription(
            Joy,
            'j0',
            self.joy_callback,
            10
        )
        
        # Parameters for axis mappings
        self.declare_parameter('axis_linear', 5)
        self.declare_parameter('axis_angular', 4)
        self.declare_parameter('linear_speed_scale', 0.2)
        self.declare_parameter('angular_speed_scale', 0.5)
        
        self.axis_linear = self.get_parameter('axis_linear').value
        self.axis_angular = self.get_parameter('axis_angular').value
        self.linear_speed_scale = self.get_parameter('linear_speed_scale').value
        self.angular_speed_scale = self.get_parameter('angular_speed_scale').value
        
        # Store last velocities
        self.last_linear_speed = 0.0
        self.last_angular_speed = 0.0
        
        # Timer for publishing at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_twist)
        
        self.get_logger().info('Joy to Twist node started')
        self.get_logger().info(f'Linear axis: {self.axis_linear}, Angular axis: {self.axis_angular}')
    
    def joy_callback(self, msg):
        """Process joystick input"""
        # Get linear velocity from the specified axis
        if len(msg.axes) > self.axis_linear:
            self.last_linear_speed = msg.axes[self.axis_linear] * self.linear_speed_scale
        
        # Get angular velocity from the specified axis
        if len(msg.axes) > self.axis_angular:
            self.last_angular_speed = msg.axes[self.axis_angular] * self.angular_speed_scale
    
    def publish_twist(self):
        """Publish Twist message at regular intervals"""
        twist_msg = Twist()
        twist_msg.linear.x = self.last_linear_speed
        twist_msg.angular.z = self.last_angular_speed
        self.twist_pub.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = JoyToTwist()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()