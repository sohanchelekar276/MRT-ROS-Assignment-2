from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='rover_status', executable='sensor_sim', name='sensor_sim'),
        Node(package='rover_status', executable='health_monitor', name='health_monitor'),
        Node(package='rover_status', executable='dashboard', name='dashboard', output='screen'),
    ])
