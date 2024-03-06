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
# The last observed depths of the obstacle
previous_depths: list[float] = [MAX_DISTANCE]

# Midpoint of the camera
MID_POINT_X: int = rc.camera.get_width() // 2
MID_POINT_Y: int = rc.camera.get_height() // 2

STATE = Mode.FORWARD

# Proportional Controllers
ANGLE_KP = 0.5
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

    # We always parse the image to compute the closest point
    depth_image = rc.camera.get_depth_image()

    mid_x: int = rc.camera.get_width() // 2
    mid_y: int = rc.camera.get_height() // 2

    # NOTE! Points are in (y-x coordinates!)
    mid_point: tuple[int, int] = (mid_y, mid_x)

    closest_point: tuple[int, int] = get_closest_depth_coordinates_at_position(
        depth_image, mid_point, pixel_range_x=300, pixel_range_y=50
    )
    depth: float = depth_image[closest_point]

    if STATE == Mode.FORWARD:
        if depth < TARGET_DEPTH * 20:
            center_x: float = closest_point[1]
            angle_error = (center_x - MID_POINT_X) / MID_POINT_X
            angle_change: float = ANGLE_KP * angle_error * rc.get_delta_time()
            angle = rc_utils.clamp(angle + angle_change, min=-1, max=1)

            speed_limit = rc_utils.clamp(
                abs(depth / (TARGET_DEPTH * 8)), min=0.05, max=1
            )

            # If we still have forward-momentum allow us to reverse at full speed
            if depth <= (sum(previous_depths) / len(previous_depths)):
                min_speed = -0.5
                max_speed = speed_limit
            else:
                min_speed = -speed_limit
                max_speed = 0.5

                # If we reverse we wish to ensure the wheels point the opposite way
                angle *= -1

            speed_error = (depth - TARGET_DEPTH) / TARGET_DEPTH
            speed_change: float = SPEED_KP * speed_error * rc.get_delta_time()
            speed = rc_utils.clamp(speed + speed_change, min=min_speed, max=max_speed)

            # Heuristic to stop the car rather than micro adjustments
            if (abs(speed) < 0.05) and (abs(speed_error) < 0.05):
                speed = 0

            previous_depths.append(depth)
            if len(previous_depths) > 3:
                previous_depths.pop()

            print(f"{speed=}, {angle=}")

        else:
            # Look for cone!
            STATE = Mode.SEARCH

    elif STATE == Mode.SEARCH:
        angle = 1
        speed = 1

        if depth < TARGET_DEPTH * 20:
            STATE = Mode.FORWARD

    elif STATE == Mode.COMPLETE_STOP:
        angle = 0
        speed = 0

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
