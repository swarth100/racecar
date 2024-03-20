"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 3B - Depth Image Cone Parking
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

from lab3_utils import get_closest_depth_coordinates_at_position, Direction

########################################################################################
# Common Utilities
########################################################################################


class Mode(Enum):
    FORWARD = 0
    SEARCH = 1
    COMPLETE_STOP = 2
    FORCE_REVERSING = 3


########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# >> Constants
# Max observable distance
MAX_DISTANCE = 1_000
# Depth at which we wish to park our car
TARGET_DEPTH = 30
# The depth at which we consider an obstacle to be close or far
OBSTACLE_DEPTH = 2 * TARGET_DEPTH
FAR_OBSTACLE_DEPTH = 3 * TARGET_DEPTH
# The midpoint of the camera
MID_POINT_X: int = rc.camera.get_width() // 2

# >> Variables
# The current speed of the car
speed = 0.0
# The current angle of the car's wheels
angle = 0.0

# The direction tracker
direction_tracker: Direction = Direction()

# State the Racecar is in
STATE = Mode.FORWARD
TIME_COUNTER: float = 0

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
    print(">> Lab 3B - Depth Image Cone Parking")


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global previous_depth
    global speed
    global angle
    global STATE
    global TIME_COUNTER

    depth_image = rc.camera.get_depth_image()

    mid_x: int = rc.camera.get_width() // 2
    mid_y: int = rc.camera.get_height() // 2
    dx: int = mid_x // 6 * 5

    mid_point: tuple[int, int] = (mid_y, mid_x)
    low_point: tuple[int, int] = (mid_y + 105, mid_x)
    lhs_point: tuple[int, int] = (mid_y, mid_x - dx)
    rhs_point: tuple[int, int] = (mid_y, mid_x + dx)

    # NOTE! Points are in (y-x coordinates!)
    closest_point_mid: tuple[int, int] = get_closest_depth_coordinates_at_position(
        depth_image, mid_point, pixel_range_x=220, pixel_range_y=80
    )
    depth_mid: float = depth_image[closest_point_mid] or MAX_DISTANCE
    closest_point_low: tuple[int, int] = get_closest_depth_coordinates_at_position(
        depth_image, low_point, pixel_range_x=220, pixel_range_y=20
    )
    depth_low: float = depth_image[closest_point_low] or MAX_DISTANCE

    closest_point_lhs: tuple[int, int] = get_closest_depth_coordinates_at_position(
        depth_image, lhs_point, pixel_range_x=50, pixel_range_y=80
    )
    depth_lhs: float = depth_image[closest_point_lhs] or MAX_DISTANCE

    closest_point_rhs: tuple[int, int] = get_closest_depth_coordinates_at_position(
        depth_image, rhs_point, pixel_range_x=50, pixel_range_y=80
    )
    depth_rhs: float = depth_image[closest_point_rhs] or MAX_DISTANCE

    # We do not need to track our direction if the obstacle is far away.
    # Direction tracking allows the car to know if it's going forward or reversing.
    direction_tracker.add_measurement(depth_low)
    if depth_low > TARGET_DEPTH * 4:
        direction_tracker.reset()

    # Park the car 30 cm away from the closest cone
    if STATE == Mode.FORWARD:
        print(">> FORWARD")
        if depth_lhs < OBSTACLE_DEPTH:
            print(f"{depth_lhs=:.2f}, {depth_rhs=:.2f}")
            STATE = Mode.FORCE_REVERSING
            # We reverse for fixed amounts of time
            TIME_COUNTER = 0.75
            angle = 1

        elif depth_rhs < OBSTACLE_DEPTH:
            print(f"{depth_lhs=:.2f}, {depth_rhs=:.2f}")
            STATE = Mode.FORCE_REVERSING
            # We reverse for fixed amounts of time
            TIME_COUNTER = 0.75
            angle = -1

        elif (depth_mid < FAR_OBSTACLE_DEPTH) or (depth_low < OBSTACLE_DEPTH):
            center_x: float = closest_point_mid[1]
            angle_error = (center_x - MID_POINT_X) / MID_POINT_X
            angle_change: float = ANGLE_KP * angle_error * rc.get_delta_time()
            angle = rc_utils.clamp(angle + angle_change, min=-1, max=1)

            # As we get closer to obstacles we wish to reduce our speed
            speed_limit = rc_utils.clamp(
                abs(depth_low / (TARGET_DEPTH * 12)), min=0.05, max=1
            )
            speed_error = (depth_low - TARGET_DEPTH) / TARGET_DEPTH

            # If we still have forward momentum allow us to reverse at full speed
            if direction_tracker.is_forward(depth_low):
                min_speed = -1
                max_speed = speed_limit
            else:
                min_speed = -speed_limit
                max_speed = 0.5

                # If we reverse we wish to ensure the wheels point the opposite way
                print(">> (REVERSING)!")
                speed_error *= 100
                angle_change *= -10

            speed_change: float = SPEED_KP * speed_error * rc.get_delta_time()
            speed = rc_utils.clamp(speed + speed_change, min=min_speed, max=max_speed)

            # Heuristic to stop the car rather than micro adjustments
            if (abs(speed) < 0.01) and (abs(speed_error) < 0.01):
                STATE = Mode.COMPLETE_STOP
        else:
            # Look for cone!
            STATE = Mode.SEARCH

    elif STATE == Mode.SEARCH:
        print(">> SEARCH")
        angle = -1
        speed = 1

        if depth_mid < FAR_OBSTACLE_DEPTH:
            STATE = Mode.FORWARD
            angle = 0
            speed = 0.4

    elif STATE == Mode.COMPLETE_STOP:
        print(">> COMPLETE_STOP")
        angle = 0
        speed = 0

        if depth_mid > OBSTACLE_DEPTH:
            STATE = Mode.FORWARD

    elif STATE == Mode.FORCE_REVERSING:
        print(">> FORCE_REVERSING")
        speed = -1

        # We decrease the time counter to ensure we do not reverse forever
        TIME_COUNTER -= rc.get_delta_time()

        if TIME_COUNTER <= 0:
            TIME_COUNTER = 0
            angle = 0
            STATE = Mode.FORWARD

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
