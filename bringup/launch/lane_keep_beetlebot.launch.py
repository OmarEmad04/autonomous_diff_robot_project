from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
import os


def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_world = get_package_share_directory('world_lane_1')
    pkg_robot = get_package_share_directory('beetlebot_description')
    xacro_file = os.path.join(pkg_robot, 'xacro', 'beetlebot.xacro')
    robot_description = Command(['xacro ', xacro_file])
    
    robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description': robot_description}]
    )
    
    spawn_entity = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=[
        '-entity', 'custom_diffbot',
        '-topic', 'robot_description',
        '-x', '0',
        '-y', '0',
        '-z', '0.1'
    ],
    output='screen'
    )




    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gazebo.launch.py'])]
            ),
            launch_arguments={'world': PathJoinSubstitution([pkg_world, 'worlds', 'lane_keep.world'])}.items()
        ),

        robot_state_publisher,
        spawn_entity,
    ])
