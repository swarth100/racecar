"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 2A - Color Image Line Following
"""
import itertools

########################################################################################
# Imports
########################################################################################

import sys
from typing import Tuple, Optional

from simple_pid import PID
from typing_extensions import TypeAlias

import cv2 as cv
import numpy as np
from nptyping import NDArray

sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# >> Constants
# The smallest contour we will recognize as a valid contour
MIN_CONTOUR_AREA = 60

# A crop window for the floor directly in front of the car
CROP_FLOOR = ((360, 0), (rc.camera.get_height(), rc.camera.get_width()))

# Colors, stored as a pair (hsv_min, hsv_max)
THSV: TypeAlias = Tuple[Tuple[int, int, int], Tuple[int, int, int]]
BLUE: list[THSV] = [((90, 50, 50), (120, 255, 255))]
GREEN: list[THSV] = [((40, 50, 50), (80, 255, 255))]
RED: list[THSV] = [((0, 50, 50), (10, 255, 255)), ((160, 50, 50), (179, 255, 255))]

# >> Variables
speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels
contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour

line_priority_index: int = 0

angle_controller = PID(Kp=1, Ki=0.1, Kd=0.05, setpoint=0, output_limits=(-1, 1))
speed_controller = PID(Kp=1, Ki=0.05, Kd=0.02, setpoint=0, output_limits=(-0.85, 0.85))

########################################################################################
# Functions
########################################################################################


def get_hsv_ranges_and_colors() -> list[tuple[rc_utils.ColorBGR, list[THSV]]]:
    """
    Returns a tuple of lists of HSV ranges and colors to search for using itertools combinations based on the
    global index value (`line_priority_index`).
    """

    hsv_range_permutations: tuple[list[THSV]] = tuple(
        itertools.permutations([RED, GREEN, BLUE])
    )

    # noinspection PyTypeChecker
    hsv_color_permutations: tuple[list[rc_utils.ColorBGR]] = tuple(
        itertools.permutations(
            [
                rc_utils.ColorBGR.red,
                rc_utils.ColorBGR.green,
                rc_utils.ColorBGR.blue,
            ]
        )
    )

    return zip(
        hsv_color_permutations[line_priority_index],
        hsv_range_permutations[line_priority_index],
    )


def update_contour():
    """
    Finds contours in the current color image and uses them to update contour_center
    and contour_area
    """
    global contour_center
    global contour_area

    image = rc.camera.get_color_image()

    if image is None:
        contour_center = None
        contour_area = 0
    else:
        # TODO (challenge 1): Search for multiple tape colors with a priority order
        # (currently we only search for blue)

        # Crop the image to the floor directly in front of the car
        image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])

        contour: Optional[NDArray] = None

        # Rationale: we prioritize contour colors based on the current mode.
        # If at least one contour is found of the given color (of sufficiently large size), we use it and stop
        # searching for other contours.
        for color, hsv_options in get_hsv_ranges_and_colors():
            contours: list[NDArray] = []
            for hsv in hsv_options:
                contours.extend(rc_utils.find_contours(image, hsv[0], hsv[1]))

            # Filter contours by size to reduce noise
            contour: NDArray = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)

            if contour is not None:
                # Calculate contour information
                contour_center = rc_utils.get_contour_center(contour)
                contour_area = rc_utils.get_contour_area(contour)

                # Draw contour onto the image
                rc_utils.draw_contour(image, contour, color=color.value)
                rc_utils.draw_circle(image, contour_center)
                break
            else:
                contour_center = None
                contour_area = 0

        # Display the image to the screen
        rc.display.show_color_image(image)


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
    print(
        ">> Lab 2A - Color Image Line Following\n"
        "\n"
        "Controls:\n"
        "    Right trigger = accelerate forward\n"
        "    Left trigger = accelerate backward\n"
        "    A button = print current speed and angle\n"
        "    B button = print contour center and area"
    )


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global speed
    global angle
    global line_priority_index

    # Search for contours in the current color image
    update_contour()

    # Use the triggers to control the car's speed
    forwardSpeed = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    backSpeed = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    speed = forwardSpeed - backSpeed

    # Choose an angle based on contour_center
    # If we could not find a contour, keep the previous angle
    if contour_center is not None:
        # We leveraged an existing PID controller to achieve proportional control.
        # Proportional control is achieved over the angle and also over the speed.
        target = rc.camera.get_width() / 2
        error = target - contour_center[1]
        normalised = error / target
        angle = angle_controller(normalised)

        # Only apply speed proportional control if the car is being manually driven.
        if forwardSpeed or backSpeed:
            speed = speed * (1 - abs(speed_controller(normalised)))

    rc.drive.set_speed_angle(speed, angle)

    # Print the current speed and angle when the A button is held down
    if rc.controller.is_down(rc.controller.Button.A):
        print("Speed:", speed, "Angle:", angle)

    # Print the center and area of the largest contour when B is held down
    if rc.controller.is_down(rc.controller.Button.B):
        if contour_center is None:
            print("No contour found")
        else:
            print("Center:", contour_center, "Area:", contour_area)

    # Allow for switching of the color priority scheme by pressing X
    if rc.controller.was_pressed(rc.controller.Button.X):
        line_priority_index = (line_priority_index + 1) % 6

        colors: list[str] = [
            str(color.name.upper()) for color, _ in get_hsv_ranges_and_colors()
        ]
        print("Chaning line priority index to: ", colors)


def update_slow():
    """
    After start() is run, this function is run at a constant rate that is slower
    than update().  By default, update_slow() is run once per second
    """
    # Print a line of ascii text denoting the contour area and x-position
    if rc.camera.get_color_image() is None:
        # If no image is found, print all X's and don't display an image
        print("X" * 10 + " (No image) " + "X" * 10)
    else:
        # If an image is found but no contour is found, print all dashes
        if contour_center is None:
            print("-" * 32 + " : area = " + str(contour_area))

        # Otherwise, print a line of dashes with a | indicating the contour x-position
        else:
            s = ["-"] * 32
            s[int(contour_center[1] / 20)] = "|"
            print("".join(s) + " : area = " + str(contour_area))


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
