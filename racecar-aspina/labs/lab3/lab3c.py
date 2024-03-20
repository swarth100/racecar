"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 3C - Depth Camera Wall Parking
"""

########################################################################################
# Imports
########################################################################################

import sys
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, Tuple

sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils

from lab3_utils import (
    get_closest_depth_at_position,
    get_closest_depth_coordinates_at_position,
)


########################################################################################
# Common Utilities
########################################################################################


class Mode(Enum):
    FORWARD = 0
    SEARCH = 1
    COMPLETE_STOP = 2
    FORCE_REVERSING = 3


@dataclass
class Direction:
    measurements: list[float] = field(default_factory=list)

    def add_measurement(self, measurement: float):
        self.measurements.append(measurement)

        # We add depths to the end of the list, and we pop front to cycle through
        if len(self.measurements) > NUM_DEPTH_MEASUREMENTS:
            self.measurements.pop(0)

    def is_forward(self, comp_measurement: float) -> bool:
        if len(self.measurements) != NUM_DEPTH_MEASUREMENTS:
            return True

        return comp_measurement < (sum(self.measurements) / len(self.measurements))

    def reset(self):
        self.measurements.clear()


########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# >> Variables
# The current speed of the car
speed = 0.0
# The current angle of the car's wheels
angle = 0.0

# Depth at which we wish to park our car
TARGET_DEPTH = 20
# Max detectable distance by the depth sensor
MAX_DISTANCE = 1_000
# Number of depth measurements to average
NUM_DEPTH_MEASUREMENTS: int = 10
# The direction tracker
direction_tracker: Direction = Direction()

# Midpoint of the camera
MID_POINT_X: int = rc.camera.get_width() // 2
MID_POINT_Y: int = rc.camera.get_height() // 2

STATE = Mode.FORWARD
TIME_COUNTER: float = 0

# Proportional Controllers
ANGLE_KP = 0.01
SPEED_KP = 200

########################################################################################
# Functions
########################################################################################


def start():
    """
    This function is run once every time the start button is pressed
    """
    global speed
    global angle

    # Initialize variables
    speed = 0
    angle = 0

    # Set initial driving speed and angle
    rc.drive.set_speed_angle(speed, angle)

    # Set update_slow to refresh every half second
    rc.set_update_slow_time(0.5)

    # Print start message
    print(">> Lab 3C - Depth Camera Wall Parking")


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global previous_depths
    global speed
    global angle
    global STATE
    global TIME_COUNTER

    # We always parse the image to compute the closest point
    depth_image = rc.camera.get_depth_image()

    mid_x: int = rc.camera.get_width() // 2
    dx: int = mid_x // 3 * 2
    mid_y: int = rc.camera.get_height() // 2

    # NOTE! Points are in (y-x coordinates!)
    mid_point: tuple[int, int] = (mid_y, mid_x)
    lhs_point: tuple[int, int] = (mid_y, mid_x - dx)
    rhs_point: tuple[int, int] = (mid_y, mid_x + dx)

    closest_point_lhs: tuple[int, int] = get_closest_depth_coordinates_at_position(
        depth_image, lhs_point, pixel_range_x=100, pixel_range_y=50
    )
    depth_lhs: float = depth_image[closest_point_lhs] or MAX_DISTANCE

    closest_point_rhs: tuple[int, int] = get_closest_depth_coordinates_at_position(
        depth_image, rhs_point, pixel_range_x=100, pixel_range_y=50
    )
    depth_rhs: float = depth_image[closest_point_rhs] or MAX_DISTANCE

    # We treat the measured depth as an average between the two measured depths
    depth: float = (depth_lhs + depth_rhs) / 2
    min_depth: float = min(depth_lhs, depth_rhs)
    max_depth: float = max(depth_lhs, depth_rhs)
    depth_delta = abs(max_depth - min_depth)
    direction_tracker.add_measurement(depth)

    # We do not need to track our direction if the obstacle is far away.
    # Direction tracking allows the car to know if it's going forward or reversing.
    if min_depth > TARGET_DEPTH * 4:
        direction_tracker.reset()

    if STATE == Mode.FORWARD:
        print(">> FORWARD")

        if depth < TARGET_DEPTH * 20:
            angle_error: float = depth_lhs - depth_rhs
            angle_change: float = ANGLE_KP * angle_error * rc.get_delta_time()

            # As we get closer to obstacles we wish to reduce our speed
            speed_limit = rc_utils.clamp(
                abs(depth / (TARGET_DEPTH * 8)), min=0.05, max=1
            )

            # If we still have forward-momentum allow us to reverse at full speed
            # We only check this on close depths
            if direction_tracker.is_forward(depth):
                min_speed = -1
                max_speed = speed_limit
            else:
                min_speed = -speed_limit
                max_speed = 0.5

                # If we reverse we wish to ensure the wheels point the opposite way
                print(">> (REVERSING)!")
                angle_change *= -10

            speed_error: float = (min_depth - TARGET_DEPTH) / TARGET_DEPTH

            # We attempt to handle cases where the wall is at a strong offset
            # Strong offsets occur when the LHS and RHS readings are significantly different
            # We must significantly reverse to re-approach the wall
            if (speed_error < 1) and (depth_delta > TARGET_DEPTH):
                STATE = Mode.FORCE_REVERSING
                # We reverse for fixed amounts of time
                TIME_COUNTER = 2.0

            speed_change: float = SPEED_KP * speed_error * rc.get_delta_time()
            speed = rc_utils.clamp(speed + speed_change, min=min_speed, max=max_speed)
            angle = rc_utils.clamp(angle + angle_change, min=-1, max=1)

            # Heuristic to stop the car rather than micro adjustments
            if (abs(speed) < 0.03) and (abs(speed_error) < 0.01):
                STATE = Mode.COMPLETE_STOP

        else:
            # Look for wall!
            STATE = Mode.SEARCH

    elif STATE == Mode.SEARCH:
        print(">> SEARCH")
        angle = 1
        speed = 1

        if depth < TARGET_DEPTH * 20:
            STATE = Mode.FORWARD

    elif STATE == Mode.COMPLETE_STOP:
        print(">> COMPLETE_STOP")
        angle = 0
        speed = 0

        if depth > TARGET_DEPTH * 2:
            STATE = Mode.FORWARD

    elif STATE == Mode.FORCE_REVERSING:
        print(">> FORCE_REVERSING")
        angle = 0
        speed = -1

        # We decrease the time counter to ensure we do not reverse forever
        TIME_COUNTER -= rc.get_delta_time()

        if TIME_COUNTER <= 0:
            TIME_COUNTER = 0
            STATE = Mode.FORWARD

    # Print the current speed and angle when the A button is held down
    if rc.controller.is_down(rc.controller.Button.A):
        print("Speed:", speed, "Angle:", angle)

    if rc.controller.is_down(rc.controller.Button.X):
        STATE = Mode.FORWARD

    if rc.controller.is_down(rc.controller.Button.Y):
        STATE = Mode.COMPLETE_STOP

    rc.drive.set_speed_angle(speed=speed, angle=angle)


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()
