from datetime import datetime
import os
from typing import Dict
import math

from traj_lib.traj_sim import (
    NO_NOISE,
    LOW_NOISE,
    MEDIUM_NOISE,
    HIGH_NOISE,
    UnicycleSim,
    State,
    Command,
    wrap_angle,
)
from traj_lib.cfg import CMD_BOUNDS
from traj_lib.traj_shapes import LineTraj, CircleTraj, TrajShape, Point, SquareTraj
from traj_lib.traj_vis import TrajVisualizer

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
    Kp_x = 1.0
    Kp_theta = 1.0


    goal_pt = goal_traj.get_goal_pt(t)
    x_tar = goal_pt.x
    y_tar = goal_pt.y

    x_cur = cur_state.x
    y_cur = cur_state.y
    heading_cur = wrap_angle(cur_state.heading)

    # your code starts here

    # distance error
    e_dist = math.sqrt((x_tar - x_cur)**2 + (y_tar - y_cur)**2)

    # calculate angle error
    heading_tar = math.atan2(y_tar - y_cur, x_tar - x_cur)
    e_theta = heading_tar - heading_cur

    vx_cmd = Kp_x * e_dist
    vtheta_cmd = Kp_theta * e_theta

    # optional damping on v_x
    vx_cmd *= max(0, math.cos(e_theta))    # when current heading aligns with target heading (cos=1), v_x is not scaled down
 
    # your code ends here

    new_cmd = Command(vx_cmd, vtheta_cmd)
    return new_cmd


def main():

    # get path to current file and make a subfolder "traj_data" to log our trajectories inside
    log_dir: str = os.path.dirname(os.path.abspath(__file__))
    log_dir: str = os.path.join(log_dir, "traj_data")
    os.makedirs(log_dir, exist_ok=True)

    # initialize the requied modules
    sim: UnicycleSim = UnicycleSim(dt=0.05, noise_cfg=LOW_NOISE, cmd_bounds=CMD_BOUNDS)
    start: State = sim.get_cur_state()
    start_pt: Point = Point(x=start.x, y=start.y)
    end_pt_goal: Point = Point(x=1.0, y=1.0)
    goal_seconds: float = 5.0
    goal_traj: LineTraj = LineTraj(
        start=start_pt, end=end_pt_goal, seconds=goal_seconds
    )
    # goal_traj: CircleTraj = CircleTraj(start=start_pt, radius=1.5, seconds=goal_seconds)
    # goal_traj: SquareTraj = SquareTraj(start=start_pt, seconds=5, side_len=1.0)
    traj_vis: TrajVisualizer = TrajVisualizer(goal_traj=goal_traj, start=start)

    my_cache = dict()

    # loop and run the simulation

    while sim.get_cur_time() < goal_seconds:
        new_cmd: Command = get_cmd(
            t=sim.get_cur_time(),
            goal_traj=goal_traj,
            cur_state=sim.get_cur_state(),
            my_cache=my_cache,
        )
        new_state: State = sim.step(new_cmd)
        traj_vis.log(new_cmd, new_state, sim.get_cur_time())

    new_plot_name: str = "traj_" + datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + ".png"
    new_plot_path: str = os.path.join(log_dir, new_plot_name)

    traj_vis.visualize_and_save_traj(cmd_bounds=CMD_BOUNDS, file_path=new_plot_path)


if __name__ == "__main__":
    main()
