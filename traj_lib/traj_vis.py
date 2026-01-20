from traj_lib.traj_sim import State, Command
from dataclasses import dataclass
import math
from typing import List

import matplotlib.pyplot as plt

from traj_lib.cfg import CmdBoundaries
from traj_lib.traj_shapes import TrajShape


class TrajVisualizer:
    def __init__(self, goal_traj: TrajShape, start: State) -> None:
        self.state_seq: List[State] = []
        self.cmd_seq: List[Command] = []
        self.time_seq: List[float] = []
        self.goal_traj = goal_traj
        self.start = start

    def reset(self) -> None:
        self.state_seq = []
        self.cmd_seq = []
        self.time_seq = []

    def log(self, cmd: Command, state: State, cur_time: float) -> None:
        self.state_seq.append(state)
        self.cmd_seq.append(cmd)
        self.time_seq.append(cur_time)

    def gen_traj_figs(self, cmd_bounds: CmdBoundaries) -> None:
        plt.rcParams["figure.constrained_layout.use"] = True
        fig = plt.figure(figsize=(12, 12))

        gs = fig.add_gridspec(3, 4)
        full_traj_ax = fig.add_subplot(gs[:2, :2])
        vx_cmd_ax = fig.add_subplot(gs[0, 2])
        vtheta_cmd_ax = fig.add_subplot(gs[2, 2])
        x_ax = fig.add_subplot(gs[0, 3])
        y_ax = fig.add_subplot(gs[1, 3])
        theta_ax = fig.add_subplot(gs[2, 3])

        actual_x = []
        actual_y = []
        actual_heading = []

        vx = []
        vtheta = []

        x_goal = []
        y_goal = []

        times = []

        squared_errors = []

        for i in range(len(self.state_seq)):
            state = self.state_seq[i]
            actual_x.append(state.x)
            actual_y.append(state.y)
            actual_heading.append(state.heading)

            ctrl = self.cmd_seq[i]
            vx.append(ctrl.vx)
            vtheta.append(ctrl.vtheta)
            t = self.time_seq[i]
            times.append(t)
            g = self.goal_traj.get_goal_pt(t)
            x_goal.append(g.x)
            y_goal.append(g.y)

            x_err = abs(state.x - g.x)
            y_err = abs(state.y - g.y)
            squared_errors.append((x_err + y_err) ** 2)

        rms_error = math.sqrt(sum(squared_errors) / len(squared_errors))

        full_traj_ax.plot(
            [self.start.x], [self.start.y], "gx", markersize=12, label="start"
        )
        full_traj_ax.plot(
            [actual_x[-1]], [actual_y[-1]], "rx", markersize=12, label="end"
        )
        full_traj_ax.plot(
            [self.start.x] + actual_x,
            [self.start.y] + actual_y,
            "b-",
            label="actual_traj",
        )
        full_traj_ax.plot(
            [self.start.x] + x_goal, [self.start.y] + y_goal, "m--", label="goal_traj"
        )
        full_traj_ax.legend(loc="upper left")
        full_traj_ax.set_xlabel("x (meters)")
        full_traj_ax.set_ylabel("y (meters)")
        full_traj_ax.set_title(
            "2D Trajectory vs Goal, RMS Error {}".format(round(rms_error, 2))
        )

        vx_cmd_ax.plot(times, vx)
        vx_cmd_ax.plot(times, [cmd_bounds.vx_bounds.low] * len(times), "r--")
        vx_cmd_ax.plot(times, [cmd_bounds.vx_bounds.high] * len(times), "r--")
        vx_cmd_ax.set_ylabel("vx (meters per second)")
        vx_cmd_ax.set_title("Commands")

        vtheta_cmd_ax.plot(times, vtheta)
        vtheta_cmd_ax.plot(times, [cmd_bounds.vtheta_bounds.low] * len(times), "r--")
        vtheta_cmd_ax.plot(times, [cmd_bounds.vtheta_bounds.high] * len(times), "r--")
        vtheta_cmd_ax.set_ylabel("vtheta (radians per second)")
        vtheta_cmd_ax.set_xlabel("t (seconds)")

        x_ax.set_title("Traj by Time")
        x_ax.plot([0.0] + times, [self.start.x] + actual_x, label="actual")
        x_ax.plot([0.0] + times, [self.start.x] + x_goal, "--", label="goal")
        x_ax.set_ylabel("x (meters)")
        x_ax.legend(loc="upper left")

        y_ax.plot([0.0] + times, [self.start.y] + actual_y, label="actual")
        y_ax.plot([0.0] + times, [self.start.y] + y_goal, "--", label="goal")
        y_ax.set_ylabel("y (meters)")
        y_ax.legend(loc="upper left")

        theta_ax.plot(
            [0.0] + times, [self.start.heading] + actual_heading, label="actual"
        )
        theta_ax.set_ylabel("heading (radians)")
        theta_ax.set_xlabel("t (seconds)")

        return fig

    def visualize_and_save_traj(
        self, cmd_bounds: CmdBoundaries, file_path: str
    ) -> None:
        fig = self.gen_traj_figs(cmd_bounds)
        plt.show()
        fig.savefig(file_path)
