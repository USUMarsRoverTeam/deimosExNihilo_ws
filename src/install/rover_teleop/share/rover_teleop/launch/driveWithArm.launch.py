from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory 
import os 

def generate_launch_description():
    paramsFile = os.path.join(get_package_share_directory('rover_teleop'), 'config', 'defaultParams.yaml')
    
    return LaunchDescription([
        # Micro-ROS agent for Arduino communication
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=['serial', '--dev', '/dev/ttyACM0'],
            output='screen'
        ),
        
        # Joystick 0 (drivetrain) - /dev/input/js0 → j0 topic
        Node(
            package='joy',
            executable='joy_node',
            name='joy0',
            parameters=[{
                'device_id': 0,
                'dev': '/dev/input/js0'
            }],
            remappings=[('joy', 'j0')],
            output='screen'
        ),
        
        # Joystick 1 (arm) - /dev/input/js1 → j1 topic
        Node(
            package='joy',
            executable='joy_node',
            name='joy1',
            parameters=[{
                'device_id': 1,
                'dev': '/dev/input/js1'
            }],
            remappings=[('joy', 'j1')],
            output='screen'
        ),
        
        # Drivetrain control
        Node(
            package='rover_teleop',
            executable='drivetrainOps.py',
            name='drivetrainOps',
            parameters=[paramsFile],
            output='screen'
        ),
        
        # Arm control
        Node(
            package='rover_teleop',
            executable='armOps.py',
            name='armOps',
            parameters=[paramsFile],
            output='screen'
        ),
    ])