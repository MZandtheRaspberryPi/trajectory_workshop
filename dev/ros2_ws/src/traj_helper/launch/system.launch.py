import os


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

    odom_fastlio = LaunchConfiguration("odom_fastlio")
    declare_odom_fastlio = DeclareLaunchArgument(
        "odom_fastlio",
        default_value="false",
        description="whether to bring up odom via fast lio, or rely on unitree msg for odom",
    )

    fastlio_cfg_file = LaunchConfiguration("fastlio_cfg_file")
    declare_fastlio_cfg_file = DeclareLaunchArgument(
        "fastlio_cfg_file", default_value="scan_go2w.yaml", description=""
    )

    traj_follower_node = Node(
        package="traj_helper",
        executable="trajectory_follower",
        parameters=[{"state_from_fastlio": odom_fastlio}],
        output="screen",
    )

    unitree_msg_converter = Node(
        package="unitree_examples_py", executable="cmd_vel_to_unitree", output="screen"
    )

    # e.g. fastlio_cfg_file_path = "/home/developer/ros_ws/src/Rpl-RoboHike-ROS2/rpl_nav/cfg/fast_lio/xxx.yaml"
    fastlio_cfg_file_path = PathJoinSubstitution(
        [
            FindPackageShare("traj_helper"),
            "cfg",
            "fastlio",
            LaunchConfiguration("fastlio_cfg_file.yaml"),
        ]
    )

    start_fastlio = GroupAction(
        actions=[
            SetRemap(src="/path", dst="/fastlio_path"),
            SetRemap(src="/Odometry", dst="/state_estimation"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("fast_lio"),
                        "launch",
                        "mapping.launch.py",
                    )
                ),
                launch_arguments={
                    "config_file": fastlio_cfg_file_path,
                    "rviz": "false",
                    "use_sim_time": "false",
                }.items(),
                condition=IfCondition(odom_fastlio),
            ),
        ]
    )

    xfer_format = 1  # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
    multi_topic = 0  # 0-All LiDARs share the same topic, 1-One LiDAR one topic
    data_src = 0  # 0-lidar, others-Invalid data src
    publish_freq = 10.0  # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
    output_type = 0
    frame_id = "livox_frame"
    lvx_file_path = "/home/livox/livox_test.lvx"
    cmdline_bd_code = "livox0000000001"

    user_config_path = "/home/developer/ros_ws/src/livox_ros_driver2/config/MID360_config_go2w_jetson.json"
    ################### user configure parameters for ros2 end #####################

    livox_ros2_params = [
        {"xfer_format": xfer_format},
        {"multi_topic": multi_topic},
        {"data_src": data_src},
        {"publish_freq": publish_freq},
        {"output_data_type": output_type},
        {"frame_id": frame_id},
        {"lvx_file_path": lvx_file_path},
        {"user_config_path": user_config_path},
        {"cmdline_input_bd_code": cmdline_bd_code},
    ]

    livox_driver = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_lidar_publisher",
        output="screen",
        parameters=livox_ros2_params,
        condition=IfCondition(odom_fastlio),
    )

    ld = LaunchDescription()

    # Add the actions
    ld.add_action(declare_odom_fastlio)
    ld.add_action(declare_fastlio_cfg_file)

    ld.add_action(traj_follower_node)
    ld.add_action(unitree_msg_converter)
    ld.add_action(start_fastlio)
    ld.add_action(livox_driver)

    return ld
