#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
from sensor_msgs.msg import Joy
import signal
import sys

msg = """
Reading from the joystick and Publishing to /arm_velocities!
---------------------------
Moving around:
Y on Rstick : Forward/back
X on Rstick : Rotate arm base
Y on Lstick : Up/down
X on Lstick : Wrist rotate
Y on Crosspad : Hand Up/down

RT : Open hand
LT : Close hand
RB + LB : Go Home

CTRL-C to quit (arm will go home on shutdown)
"""

class ArmTeleopJoy(Node):
    def __init__(self):
        super().__init__('arm_teleop_joy')
        
        # Velocity scaling parameters 

        ## USE THESE IF WE WANT TO UPDATE ON THE FLY
        self.declare_parameter('velocity_scale', 500.0)
        self.declare_parameter('wrist_velocity_scale', 200.0)
        self.declare_parameter('hand_velocity_scale', 300.0)
        
        self.velocity_scale = self.get_parameter('velocity_scale').value
        self.wrist_velocity_scale = self.get_parameter('wrist_velocity_scale').value
        self.hand_velocity_scale = self.get_parameter('hand_velocity_scale').value
        
        # Publishers
        self.velocity_pub = self.create_publisher(
            Float32MultiArray,
            '/arm_velocities',
            10
        )
        
        self.home_pub = self.create_publisher(
            Bool,
            '/arm_go_home',
            10
        )
        
        # Subscriber for joystick
        self.joy_sub = self.create_subscription(
            Joy,
            '/j1',
            self.joy_callback,
            10
        )
        
        # State variables
        self.joyJoystick = [0.0] * 6
        self.joyButtons = [0] * 12
        self.prev_rb_lb_pressed = False  # Track RB+LB combo state
        
        # Timer for publishing (50 Hz)
        self.timer = self.create_timer(0.02, self.publish_velocities)
        
        self.get_logger().info(msg)
    
    def joy_callback(self, msg):
        """Store joystick data"""
        self.joyJoystick = list(msg.axes)
        self.joyButtons = list(msg.buttons)
    
    def send_home_command(self):
        """Send the arm home"""
        self.get_logger().info('Sending arm home...')
        
        # Send home command
        home_msg = Bool()
        home_msg.data = True
        self.home_pub.publish(home_msg)
        
        # Zero out velocities
        vel_msg = Float32MultiArray()
        vel_msg.data = [0.0] * 6
        self.velocity_pub.publish(vel_msg)
        
        # Give it a moment to process
        import time
        time.sleep(0.1)
    
    def shutdown_hook(self):
        """Called when node is shutting down"""
        self.get_logger().info('Shutting down - sending arm home')
        self.send_home_command()
    
    def publish_velocities(self):
        """Convert joystick input to arm velocities and publish"""
        
        # Check for RB + LB press (rising edge detection)
        # RB is typically button 5, LB is typically button 4 - Check on this
        rb_lb_pressed = False
        if len(self.joyButtons) > 5:
            rb_lb_pressed = (self.joyButtons[5] == 1 and self.joyButtons[4] == 1)
            
            # Rising edge - buttons just pressed together
            if rb_lb_pressed and not self.prev_rb_lb_pressed:
                self.send_home_command()
                
            self.prev_rb_lb_pressed = rb_lb_pressed
            
            # Skip normal control if buttons are held
            if rb_lb_pressed:
                return
        
        # Normal velocity control
        vel_msg = Float32MultiArray()
        
        # DOF0: Base rotation (X on right stick)
        dof0_vel = -self.joyJoystick[2] * (1 - 0.9 * self.joyButtons[5]) * self.velocity_scale if len(self.joyButtons) > 5 else 0.0
        
        # DOF1 and DOF2: Direct control
        dof1_vel = self.joyJoystick[1] * (1 - 0.5 * self.joyButtons[5]) * self.velocity_scale if len(self.joyButtons) > 5 else 0.0
        dof2_vel = self.joyJoystick[3] * (1 - 0.5 * self.joyButtons[5]) * self.velocity_scale if len(self.joyButtons) > 5 else 0.0
        
        # DOF3: Wrist rotate (X on left stick)
        dof3_vel = -self.joyJoystick[5] * self.wrist_velocity_scale if len(self.joyJoystick) > 5 else 0.0
        
        # DOF4: Hand up/down (triggers)
        dof4_vel = 0.0
        if len(self.joyButtons) > 7:
            dof4_vel = -(self.joyButtons[7] - self.joyButtons[6]) * self.hand_velocity_scale
        
        # DOF5: Hand open/close (buttons)
        dof5_vel = 0.0
        if len(self.joyButtons) > 2:
            dof5_vel = (self.joyButtons[0] - self.joyButtons[2]) * self.hand_velocity_scale
        
        vel_msg.data = [
            float(dof0_vel),
            float(dof1_vel),
            float(dof2_vel),
            float(dof3_vel),
            float(dof4_vel),
            float(dof5_vel)
        ]
        
        self.velocity_pub.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = ArmTeleopJoy()
    
    # Register shutdown hook
    def signal_handler(sig, frame):
        node.get_logger().info('Interrupt received, shutting down...')
        node.shutdown_hook()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_hook()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()