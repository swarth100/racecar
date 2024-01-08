"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 1 - Driving in Shapes
"""

########################################################################################
# Imports
########################################################################################

import sys
import time
from dataclasses import dataclass
from enum import IntEnum

sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Put any global variables here

########################################################################################
# Functions
########################################################################################


class Shape(IntEnum):
    NONE = 0
    CIRCLE = 1
    SQUARE = 2
    FIGURE_EIGHT = 3
    CUSTOM = 4


@dataclass
class DriveController:
    shape: Shape
    start_time: float

    @property
    def time_delta(self) -> float:
        return time.perf_counter() - self.start_time


_CTRL: DriveController = DriveController(Shape.NONE, 0.0)

# Quarter Turn Time (the time in seconds taken for a 90 degree turn at full speed)
QTT: float = 1.3


def start():
    """
    This function is run once every time the start button is pressed
    """
    # Begin at a full stop
    rc.drive.stop()

    # Print start message
    # TODO (main challenge): add a line explaining what the Y button does
    print(
        ">> Lab 1 - Driving in Shapes\n"
        "\n"
        "Controls:\n"
        "    Right trigger = accelerate forward\n"
        "    Left trigger = accelerate backward\n"
        "    Left joystick = turn front wheels\n"
        "    A button = drive in a circle\n"
        "    B button = drive in a square\n"
        "    X button = drive in a figure eight\n"
    )


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global _CTRL
    cur_time: float = time.perf_counter()

    # Rationale: The user clicks the button once but the car should keep driving in the given shape even
    # when the button is released. Clicking the button hence sets the shape to be driven in, for later continued use.
    if rc.controller.was_pressed(rc.controller.Button.A):
        # Drive in a circle when the A button is pressed
        print("Driving in a circle...")
        _CTRL = DriveController(shape=Shape.CIRCLE, start_time=cur_time)
    elif rc.controller.was_pressed(rc.controller.Button.B):
        # Drive in a square when the B button is pressed
        print("Driving in a square...")
        _CTRL = DriveController(shape=Shape.SQUARE, start_time=cur_time)
    elif rc.controller.was_pressed(rc.controller.Button.X):
        # Drive in a figure eight when the X button is pressed
        print("Driving in a figure eight...")
        _CTRL = DriveController(shape=Shape.FIGURE_EIGHT, start_time=cur_time)

    # Rationale: We divide the figure to be drawn into quartiles.
    # For each quartile we determine the driving behavior as a function of time.
    if _CTRL.shape == Shape.CIRCLE:
        rc.drive.set_speed_angle(1, 1)
    elif _CTRL.shape == Shape.SQUARE:
        is_straight: bool = 0 <= _CTRL.time_delta % (2 * QTT) <= QTT
        if is_straight:
            rc.drive.set_speed_angle(1, 0)
        else:
            # Otherwise, turn right
            rc.drive.set_speed_angle(1, 1)
    elif _CTRL.shape == Shape.FIGURE_EIGHT:
        is_right_turn: bool = (2 * QTT) <= _CTRL.time_delta % (8 * QTT) <= (6 * QTT)
        if is_right_turn:
            rc.drive.set_speed_angle(1, 1)
        else:
            # Otherwise, turn left
            rc.drive.set_speed_angle(1, -1)


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()
