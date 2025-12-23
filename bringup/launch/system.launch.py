from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    controller_node = Node(
        package='lane_keeper',
        executable='controller_node',
        name='controller_node',
        output='screen'
    )

    lane_vision_node = Node(
        package='lane_keeper',
        executable='lane_vision_node',
        name='lane_vision_node',
        output='screen'
    )

    path_node = Node(
        package='lane_keeper',
        executable='path_node',
        name='path_node',
        output='screen'
    )

    return LaunchDescription([
        controller_node,
        lane_vision_node,
        path_node,
    ])