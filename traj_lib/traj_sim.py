from copy import copy
from dataclasses import dataclass
import math
import random
from typing import Tuple

from traj_lib.cfg import CMD_BOUNDS, CmdBoundaries, LowHighBound, enforce_bounds


@dataclass
class NoiseConfig:
    linear_noise: float
    angular_noise: float


@dataclass
class Command:
    vx: float
    vtheta: float


@dataclass
class State:
    x: float
    y: float
    heading: float


NO_NOISE = NoiseConfig(linear_noise=0.0, angular_noise=0.0)
LOW_NOISE = NoiseConfig(linear_noise=0.01, angular_noise=0.2 * math.pi / 180)
MEDIUM_NOISE = NoiseConfig(linear_noise=0.025, angular_noise=0.5 * math.pi / 180)
HIGH_NOISE = NoiseConfig(linear_noise=0.05, angular_noise=1.0 * math.pi / 180)


class UnicycleSim:
    def __init__(
        self,
        dt: float = 0.05,
        noise_cfg: NoiseConfig = LOW_NOISE,
        cmd_bounds: CmdBoundaries = CMD_BOUNDS,
    ) -> None:
        """Simulates a unicycle model with gaussian noise (centered on zero) on the control inputs. Intialize the Simulation by choosing a noise configuration.

        Args:
            dt (float, optional): how much to advance time each time step is called, in seconds. Defaults to 0.20.
            noise_cfg (NoiseConfig, optional): How much noise to add to linear and angular velocity. Defaults to LOW_NOISE.
            cmd_bounds (CmdBoundaries, optional): The low/high boundaries for velocity commands. Defaults to CMD_BOUNDS.
        """

        """Simulates a unicycle model with gaussian noise (centered on zero) on the control inputs. Intialize the Simulation by choosing a noise configuration.

        Args:
            dt (float, optional): how much to advance time each time step is called, in seconds. Defaults to 0.20.
            noise_cfg (NoiseConfig, optional): How much noise to add to linear and angular velocity. Defaults to LOW_NOISE.
        """
        self.noise_cfg = noise_cfg
        self.cur_state = State(0.0, 0.0, 0.0)
        self.cur_time = 0.0
        self.dt = dt
        self.cmd_bounds = cmd_bounds
        self.vx_bound = cmd_bounds.vx_bounds
        self.vtheta_bound = cmd_bounds.vtheta_bounds

    def get_cur_state(self) -> State:
        """Get the current state of the robot.

        Returns:
            State: the current state of the robot in the sim.
        """
        return copy(self.cur_state)

    def get_cur_time(self) -> float:
        """Get's the current time in the simulation.

        Returns:
            float: the current time from start in seconds.
        """
        return self.cur_time

    def get_cfg(self) -> Tuple[CmdBoundaries, NoiseConfig]:
        """Get's the simulation configuration.

        Returns:
            Tuple[CmdBoundaries, NoiseConfig]: The boundary on command configuration and the noise configuration for the sim!
        """
        return copy(self.cmd_bounds), copy(self.noise_cfg)

    def reset(self) -> None:
        """Reset the simulation to the intial state."""
        self.cur_state = State(0.0, 0.0, 0.0)
        self.cur_time = 0.0

    def step(self, cmd: Command) -> State:
        """Given a command, step forward the simulation in time and return the new state of the robot.

        Args:
            cmd (Command): velocity commands for the robot.

        Returns:
            State: the state of the robot after taking the command and stepping time forward.
        """
        assert type(cmd) == Command, "cmd must be of type traj_lib.traj_sim.Command"

        linear_noise_x = random.gauss(mu=0.0, sigma=self.noise_cfg.linear_noise)
        linear_noise_y = random.gauss(mu=0.0, sigma=self.noise_cfg.linear_noise)
        angular_noise = random.gauss(mu=0.0, sigma=self.noise_cfg.angular_noise)

        vx_cmd = enforce_bounds(cmd.vx, self.vx_bound)
        vtheta_cmd = enforce_bounds(cmd.vtheta, self.vtheta_bound)

        # calculate the change in position given the robot's current heading
        dx = cmd.vx * self.dt * math.cos(self.cur_state.heading)
        dy = cmd.vx * self.dt * math.sin(self.cur_state.heading)
        dtheta = cmd.vtheta * self.dt

        new_x = self.cur_state.x + dx + linear_noise_x
        new_y = self.cur_state.y + dy + linear_noise_y
        new_heading = self.cur_state.heading + dtheta + angular_noise

        new_state = State(new_x, new_y, new_heading)
        self.cur_time += self.dt
        self.cur_time = round(self.cur_time, 5)
        self.cur_state = new_state

        return copy(self.cur_state)
