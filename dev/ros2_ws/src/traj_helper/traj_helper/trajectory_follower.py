

from datetime import datetime
import os
from copy import copy
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from unitree_go.msg import SportModeState

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
from traj_lib.traj_shapes import LineTraj, CircleTraj, TrajShape, Point
from traj_lib.traj_vis import TrajVisualizer


def get_cmd(t: float, goal_traj: TrajShape, cur_state: State) -> Command:

    goal_pt = goal_traj.get_goal_pt(t)

    vx_cmd = 0.2
    vtheta_cmd = 0.4

    # your code starts here

    # your code ends here

    new_cmd = Command(vx_cmd, vtheta_cmd)
    return new_cmd


class TrajectoryFollower(Node):
    def __init__(self):
        super().__init__('trajectory_follower')
        
        # Publisher for cmd_vel
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        
        # Subscriber for sportmodestate
        self.sport_state_sub = self.create_subscription(
            SportModeState,
            'lf/sportmodestate',
            self.sport_state_callback,
            10
        )
        
        # Timer for publishing cmd_vel at regular intervals
        self.dt = 1/20
        self.time_start = None
        self.timer = self.create_timer(self.dt, self.timer_callback)  

        self.get_logger().info('Trajectory Follower Node initialized')
        
        # Store the latest sport mode state
        self.current_state = None

        self.new_cmd = None

        # get path to current file and make a subfolder "traj_data" to log our trajectories inside
        self.log_dir: str = "/docker_mount"
        self.log_dir: str = os.path.join(self.log_dir, "traj_data")
        os.makedirs(self.log_dir, exist_ok=True)

        # initialize the requied modules
        self.start: State = None
        
        
    def sport_state_callback(self, msg):
        """Callback for receiving sport mode state"""
        x = msg.position[0]
        y = msg.position[1]
        heading = msg.imu_state.rpy[2]

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
            # self.goal_traj: CircleTraj = CircleTraj(start=self.start_pt, radius=1.5, seconds=self.goal_seconds)
            self.traj_vis: TrajVisualizer = TrajVisualizer(goal_traj=self.goal_traj, start=self.start)
        
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
                t=t, goal_traj=self.goal_traj, cur_state=self.current_state
            )
            cmd_vel.linear.x = enforce_bounds(self.new_cmd.vx, VX_BOUNDS)
            cmd_vel.angular.z = enforce_bounds(self.new_cmd.vtheta, VTHETA_BOUNDS)
        else:
            self.new_cmd = Command(0.0, 0.0)
            self.get_logger().info("done")
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
        new_plot_name: str = "traj_" + datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + ".png"
        new_plot_path: str = os.path.join(node.log_dir, new_plot_name)

        node.traj_vis.visualize_and_save_traj(cmd_bounds=CMD_BOUNDS, file_path=new_plot_path, show=False)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

