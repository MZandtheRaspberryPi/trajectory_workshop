from datetime import datetime
import os
from copy import copy
import math
import time
from typing import Dict

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from unitree_go.msg import SportModeState
from nav_msgs.msg import Odometry

from traj_lib.traj_sim import (
    NO_NOISE,
    LOW_NOISE,
    MEDIUM_NOISE,
    HIGH_NOISE,
    UnicycleSim,
    State,
    Command,
)
from traj_lib.cfg import VX_BOUNDS, VTHETA_BOUNDS, CMD_BOUNDS, enforce_bounds
from traj_lib.traj_shapes import LineTraj, CircleTraj, TrajShape, Point, SquareTraj
from traj_lib.traj_vis import TrajVisualizer


def quaternion_to_euler(q_w, q_x, q_y, q_z):
    # Roll (x-axis rotation)
    roll = math.atan2(2 * (q_w * q_x + q_y * q_z), 1 - 2 * (q_x**2 + q_y**2))

    # Pitch (y-axis rotation)
    pitch = math.asin(2 * (q_w * q_y - q_z * q_x))

    # Yaw (z-axis rotation)
    yaw = math.atan2(2 * (q_w * q_z + q_x * q_y), 1 - 2 * (q_y**2 + q_z**2))

    return roll, pitch, yaw


def get_cmd(
    t: float, goal_traj: TrajShape, cur_state: State, my_cache: Dict
) -> Command:
    """This is a function that will be called in a loop and used to get commands to pass to the robot.
    If this function returns non-sensical values they will be bounded before being passed to the robot,
    using CMD_BOUNDS.

    Args:
        t (float): time in seconds from the start of the run
        goal_traj (TrajShape): goal trajectory that can be queried for the current goal point, or future/prior goal points
        cur_state (State): current state of the robot at t.
        my_cache (Dict): a helper variable that will be passed to this function allowing caching of variables between function invocations
            by setting a key in the dictionary to some value. That key/value will be in the dictionary when the function returns
            and the next function call will have the same key/value.
            Not nescessary, but helpful for those that wish to persist variables between function invocations.

    Returns:
        Command: The command to pass to the robot.
    """

    goal_pt = goal_traj.get_goal_pt(t)
    vx_cmd = 0.2
    vtheta_cmd = 0.4
    # your code starts here

    # your code ends here

    new_cmd = Command(vx_cmd, vtheta_cmd)
    return new_cmd


class TrajectoryFollower(Node):
    def __init__(self):
        super().__init__("trajectory_follower")

        # Publisher for cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.declare_parameter("state_from_fastlio", False)
        self.state_from_fastlio = self.get_parameter("state_from_fastlio").value

        if not self.state_from_fastlio:
            # Subscriber for sportmodestate
            self.state_sub = self.create_subscription(
                SportModeState, "lf/sportmodestate", self.state_cb, 10
            )
        else:
            self.state_sub = self.create_subscription(
                Odometry, "state_estimation", self.state_cb, 10
            )

        # Timer for publishing cmd_vel at regular intervals
        self.dt = 1 / 20
        self.time_start = None
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.get_logger().info("Trajectory Follower Node initialized")

        # Store the latest sport mode state
        self.current_state = None

        self.new_cmd = None
        self.my_cache = dict()

        # get path to current file and make a subfolder "traj_data" to log our trajectories inside
        self.log_dir: str = "/docker_mount"
        self.log_dir: str = os.path.join(self.log_dir, "traj_data")
        os.makedirs(self.log_dir, exist_ok=True)

        # initialize the requied modules
        self.start: State = None

    def state_cb(self, msg):
        """Callback for receiving state"""
        x, y, heading = (None, None, None)
        if isinstance(msg, SportModeState):
            x = msg.position[0]
            y = msg.position[1]
            heading = msg.imu_state.rpy[2]
        elif isinstance(msg, Odometry):
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            quat_x, quat_y, quat_z, quat_w = (
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            )

            r, p, heading = quaternion_to_euler(quat_w, quat_x, quat_y, quat_z)

        else:
            raise NotImplementedError

        self.current_state = State(x, y, heading)

        if self.start is None:
            self.start_time = time.time()
            self.start = copy(self.current_state)

            self.start_pt: Point = Point(x=self.start.x, y=self.start.y)
            self.end_pt_goal: Point = Point(x=1.0, y=1.0)
            self.goal_seconds: float = 5.0
            self.goal_traj: LineTraj = LineTraj(
                start=self.start_pt, end=self.end_pt_goal, seconds=self.goal_seconds
            )
            # self.goal_traj: SquareTraj = SquareTraj(
            # start=self.start_pt, seconds=self.goal_seconds, side_len=1.0
            # )
            # self.goal_traj: CircleTraj = CircleTraj(
            #     start=self.start_pt, radius=1.5, seconds=self.goal_seconds
            # )
            self.traj_vis: TrajVisualizer = TrajVisualizer(
                goal_traj=self.goal_traj, start=self.start
            )

        if not self.new_cmd is None:
            current_time = time.time() - self.start_time
            self.traj_vis.log(self.new_cmd, self.current_state, current_time)

    def timer_callback(self):
        """Timer callback for publishing cmd_vel"""

        if self.current_state is None:
            return

        cmd_vel = Twist()

        # TODO: Implement trajectory following logic here
        # For now, publish zero velocity

        t = time.time() - self.start_time

        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = 0.0
        cmd_vel.linear.z = 0.0
        cmd_vel.angular.x = 0.0
        cmd_vel.angular.y = 0.0
        cmd_vel.angular.z = 0.0

        if t < self.goal_seconds:
            self.new_cmd: Command = get_cmd(
                t=t,
                goal_traj=self.goal_traj,
                cur_state=self.current_state,
                my_cache=self.my_cache,
            )
            cmd_vel.linear.x = enforce_bounds(self.new_cmd.vx, VX_BOUNDS)
            cmd_vel.angular.z = enforce_bounds(self.new_cmd.vtheta, VTHETA_BOUNDS)
        else:
            self.new_cmd = Command(0.0, 0.0)
            self.get_logger().info(f"done, {t}")
        self.cmd_vel_pub.publish(cmd_vel)

        # print command velocity
        self.get_logger().info(f"Command Velocity: {self.new_cmd}")


def main(args=None):
    rclpy.init(args=args)

    node = TrajectoryFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        new_plot_name: str = (
            "traj_" + datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + ".png"
        )
        new_plot_path: str = os.path.join(node.log_dir, new_plot_name)

        node.traj_vis.visualize_and_save_traj(
            cmd_bounds=CMD_BOUNDS, file_path=new_plot_path, show=False
        )

    node.destroy_node()
    # rclpy.shutdown() # rclpy._rclpy_pybind11.RCLError: failed to shutdown: rcl_shutdown already called on the given context, at ./src/rcl/init.c:241


if __name__ == "__main__":
    main()
