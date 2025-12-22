from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_world = get_package_share_directory('world_lane_1')
    pkg_robot = get_package_share_directory('robot_description')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gazebo.launch.py'])]
            ),
            launch_arguments={'world': PathJoinSubstitution([pkg_world, 'worlds', 'lane_keep.world'])}.items()
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'custom_diffbot',
                       '-file', PathJoinSubstitution([pkg_robot, 'urdf', 'custom_diffbot.urdf']),
                       '-x', '0', '-y', '0', '-z', '0.1'],
            output='screen'
        )
    ])
