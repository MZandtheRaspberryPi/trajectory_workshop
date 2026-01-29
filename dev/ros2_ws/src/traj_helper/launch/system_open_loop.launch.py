import os
import time


from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    GroupAction,
    SetLaunchConfiguration,
)
from launch_ros.actions import SetRemap
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
    FrontendLaunchDescriptionSource,
)
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():


    unitree_msg_converter = Node(
        package="unitree_examples_py", executable="cmd_vel_to_unitree", output="screen"
    )

    traj_follower_node = Node(
        package="traj_helper",
        executable="trajectory_follower",
        parameters=[{
            "state_from_fastlio": False,
            "use_open_loop": True
        }],
        output="screen",
    )

    ld = LaunchDescription()
    
    ld.add_action(unitree_msg_converter)
    ld.add_action(traj_follower_node)

    return ld
