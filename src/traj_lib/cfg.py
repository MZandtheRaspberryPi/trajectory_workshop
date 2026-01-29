from dataclasses import dataclass
import math


@dataclass
class LowHighBound:
    low: float
    high: float


@dataclass
class CmdBoundaries:
    vx_bounds: LowHighBound
    vtheta_bounds: LowHighBound


VX_BOUNDS = LowHighBound(low=-0.5, high=0.5)
VTHETA_BOUNDS = LowHighBound(low=-math.pi / 2, high=math.pi / 2)

CMD_BOUNDS = CmdBoundaries(VX_BOUNDS, VTHETA_BOUNDS)


def enforce_bounds(x, bounds: LowHighBound) -> float:
    new_x = max(x, bounds.low)
    new_x = min(x, bounds.high)
    return new_x
