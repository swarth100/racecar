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

# >> Constants
# The smallest contour we will recognize as a valid contour
MIN_CONTOUR_AREA = 30

# The HSV range for the color orange, stored as (hsv_min, hsv_max)
ORANGE = ((10, 100, 100), (20, 255, 255))

# >> Variables
# The current speed of the car
speed = 0.0
# The current angle of the car's wheels
angle = 0.0
# The last observed depth of the obstacle
previous_depth = 99999
# The (pixel row, pixel column) of contour
contour_center: Optional[Tuple[int, int]] = None
# The area of contour
contour_area = 0

# Depth at which we wish to park our car
TARGET_DEPTH = 30

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
        # Find all the orange contours
        contours = rc_utils.find_contours(image, ORANGE[0], ORANGE[1])

        # Select the largest contour
        contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)

        if contour is not None:
            # Calculate contour information
            contour_center = rc_utils.get_contour_center(contour)
            contour_area = rc_utils.get_contour_area(contour)

            # Draw contour onto the image
            rc_utils.draw_contour(image, contour)
            rc_utils.draw_circle(image, contour_center)

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

    # Search for contours in the current color image
    update_contour()
    #         10        20
    #         -         X

    # TODO: Park the car 30 cm away from the closest orange cone
    if STATE == Mode.FORWARD:
        if contour_center is not None:
            center_x: float = contour_center[1]
            angle_error = (center_x - MID_POINT_X) / MID_POINT_X
            angle_change: float = ANGLE_KP * angle_error * rc.get_delta_time()
            angle = rc_utils.clamp(angle + angle_change, min=-1, max=1)

            depth_image = rc.camera.get_depth_image()
            depth: float = depth_image[contour_center]

            speed_limit = rc_utils.clamp(
                abs(depth / (TARGET_DEPTH * 6)), min=0.05, max=1
            )

            # If we still have forward momentum allow us to reverse at full speed
            if depth < previous_depth:
                min_speed = -0.5
                max_speed = speed_limit
            else:
                min_speed = -speed_limit
                max_speed = 0.5

            speed_error = (depth - TARGET_DEPTH) / TARGET_DEPTH
            speed_change: float = SPEED_KP * speed_error * rc.get_delta_time()
            speed = rc_utils.clamp(speed + speed_change, min=min_speed, max=max_speed)

            # Heuristic to stop the car rather than micro adjustments
            if (abs(speed) < 0.05) and (abs(speed_error) < 0.05):
                speed = 0

            previous_depth = depth

        else:
            # Look for cone!
            STATE = Mode.SEARCH

    elif STATE == Mode.SEARCH:
        angle = 1
        speed = 1

        if contour_center is not None:
            STATE = Mode.FORWARD

    elif STATE == Mode.COMPLETE_STOP:
        angle = 0
        speed = 0

    # Print the current speed and angle when the A button is held down
    if rc.controller.is_down(rc.controller.Button.A):
        print("Speed:", speed, "Angle:", angle)

    # Print the center and area of the largest contour when B is held down
    if rc.controller.is_down(rc.controller.Button.B):
        if contour_center is None:
            print("No contour found")
        else:
            print("Center:", contour_center, "Area:", contour_area)

    if rc.controller.is_down(rc.controller.Button.X):
        STATE = Mode.FORWARD

    if rc.controller.is_down(rc.controller.Button.Y):
        STATE = Mode.COMPLETE_STOP

    rc.drive.set_speed_angle(speed=speed, angle=angle)


def update_slow():
    """
    After start() is run, this function is run at a constant rate that is slower
    than update().  By default, update_slow() is run once per second
    """
    # Print a line of ascii text denoting the contour area and x position
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
