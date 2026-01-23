from abc import ABC, abstractmethod
from copy import copy
from dataclasses import dataclass
import math


@dataclass
class Point:
    x: float
    y: float


class TrajShape(ABC):
    def __init__(self, start: Point, seconds: float) -> None:
        self.start = start
        self.seconds = seconds

    @abstractmethod
    def get_goal_pt(self, t: float) -> Point:
        pass


class LineTraj(TrajShape):
    def __init__(self, start: Point, end: Point, seconds: float):
        super().__init__(start, seconds)

        self.end = end

    def get_goal_pt(self, t: float) -> Point:
        x_goal = self.start.x + (self.end.x - self.start.x) * t / self.seconds
        y_goal = self.start.y + (self.end.y - self.start.y) * t / self.seconds
        goal_pt = Point(x_goal, y_goal)
        return goal_pt


class CircleTraj(TrajShape):
    def __init__(self, start: Point, radius: float, seconds: float):
        super().__init__(start, seconds)
        self.radius = radius
        self.circle_ctr = Point(start.x - self.radius, start.y)

    def get_goal_pt(self, t: float) -> Point:
        cur_angle_around_circle = 2 * math.pi * t / self.seconds
        x_goal = self.circle_ctr.x + self.radius * math.cos(cur_angle_around_circle)
        y_goal = self.circle_ctr.y + self.radius * math.sin(cur_angle_around_circle)
        goal_pt = Point(x_goal, y_goal)
        return goal_pt


class SquareTraj(TrajShape):
    def __init__(self, start: Point, side_len: float, seconds: float):
        super().__init__(start, seconds)
        self.side_len = side_len
        self.time_per_side = seconds / 4

        # looking top down with x to the right and y to the up
        # do the right side from start up first then proceed counterclockwise along sides
        self.top_right_corner = copy(self.start)
        self.top_right_corner.y += self.side_len
        self.top_left_corner = copy(self.top_right_corner)
        self.top_left_corner.x -= self.side_len
        self.bottom_left_corner = copy(self.top_left_corner)
        self.bottom_left_corner.y -= self.side_len
        self.bottom_right_corner = copy(self.start)

        self.right_side = LineTraj(
            self.bottom_right_corner, self.top_right_corner, self.time_per_side
        )
        self.top_side = LineTraj(
            self.top_right_corner, self.top_left_corner, self.time_per_side
        )
        self.left_side = LineTraj(
            self.top_left_corner, self.bottom_left_corner, self.time_per_side
        )
        self.bottom_side = LineTraj(
            self.bottom_left_corner, self.bottom_right_corner, self.time_per_side
        )

    def get_goal_pt(self, t: float) -> Point:
        new_t = t
        while new_t >= self.time_per_side:
            new_t -= self.time_per_side
        goal = None
        if t < self.time_per_side:
            goal = self.right_side.get_goal_pt(new_t)
        elif t >= self.time_per_side and t < 2 * self.time_per_side:
            goal = self.top_side.get_goal_pt(new_t)
        elif t >= self.time_per_side * 2 and t < 3 * self.time_per_side:
            goal = self.left_side.get_goal_pt(new_t)
        elif t >= self.time_per_side * 3 and t < 4 * self.time_per_side:
            goal = self.bottom_side.get_goal_pt(new_t)
        else:
            goal = self.start
        return goal
