#!/usr/bin/python3

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

from typing import Tuple

WORLD_SIZE = 10.88   # world extent in world units, matches stock turtlesim (both axes)
SCREEN_SIZE = 500     # pygame window size in pixels (both axes)
_SCALE = SCREEN_SIZE / WORLD_SIZE  # pixels per world unit


def world_to_screen(x: float, y: float) -> Tuple[float, float]:
    """World coords (origin bottom-left, +y up) -> screen pixel coords (origin top-left, +y down)."""
    return x * _SCALE, SCREEN_SIZE - y * _SCALE


def screen_to_world(px: float, py: float) -> Tuple[float, float]:
    """Screen pixel coords (origin top-left, +y down) -> world coords (origin bottom-left, +y up)."""
    return px / _SCALE, (SCREEN_SIZE - py) / _SCALE


def clamp_to_world(x: float, y: float) -> Tuple[float, float]:
    return min(max(x, 0.0), WORLD_SIZE), min(max(y, 0.0), WORLD_SIZE)


def world_length_to_screen(length: float) -> float:
    return length * _SCALE


# Delivery drop-off zone: a fixed spot a carried parcel must be brought to.
DROPOFF_ZONE_POSE = (WORLD_SIZE - 1.5, WORLD_SIZE - 1.5)
DROPOFF_ZONE_RADIUS = 1.2
