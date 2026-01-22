from datetime import datetime
import os

from traj_lib.traj_sim import (
    NO_NOISE,
    LOW_NOISE,
    MEDIUM_NOISE,
    HIGH_NOISE,
    UnicycleSim,
    State,
    Command,
)
from traj_lib.cfg import CMD_BOUNDS
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
    traj_vis: TrajVisualizer = TrajVisualizer(goal_traj=goal_traj, start=start)

    # loop and run the simulation

    while sim.get_cur_time() < goal_seconds:
        new_cmd: Command = get_cmd(
            t=sim.get_cur_time(), goal_traj=goal_traj, cur_state=sim.get_cur_state()
        )
        new_state: State = sim.step(new_cmd)
        traj_vis.log(new_cmd, new_state, sim.get_cur_time())

    new_plot_name: str = "traj_" + datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + ".png"
    new_plot_path: str = os.path.join(log_dir, new_plot_name)

    traj_vis.visualize_and_save_traj(cmd_bounds=CMD_BOUNDS, file_path=new_plot_path)


if __name__ == "__main__":
    main()
