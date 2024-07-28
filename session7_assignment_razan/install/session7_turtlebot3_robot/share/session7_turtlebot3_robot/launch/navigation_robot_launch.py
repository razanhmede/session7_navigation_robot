from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='session7_turtlebot3_robot',
            executable='robot_driver',
            name='robot_driver',
            output='screen',
            
        ),
        Node(
            package='session7_turtlebot3_robot',
            executable='wall_finder_service_server',
            name='wall_finder_service_server',
            output='screen'
        ),
        Node(
            package='session7_turtlebot3_robot',
            executable='lap_time_action_server',
            name='lap_time_action_server',
            output='screen'
        ),
        Node(
            package='session7_turtlebot3_robot',
            executable='lap_time_action_client',
            name='lap_time_action_client',
            output='screen'
        ),
    ])
