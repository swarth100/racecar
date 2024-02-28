"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 3A - Depth Camera Safety Stop
"""

########################################################################################
# Imports
########################################################################################

import sys
from enum import Enum

from nptyping import NDArray

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global utilities
########################################################################################


def get_closest_depth_coordinates_at_position(
    depth_image: NDArray,
    position: tuple[int, int],
    pixel_range_x: int = 20,
    pixel_range_y: int = 20,
) -> tuple[int, int]:
    (mid_y, mid_x) = position
    top_left: tuple[int, int] = (mid_y - pixel_range_y, mid_x - pixel_range_x)
    bottom_right: tuple[int, int] = (mid_y + pixel_range_y, mid_x + pixel_range_x)

    # IMPORTANT: We crop as otherwise the ground is closest!
    cropped_depth_image = rc_utils.crop(
        depth_image,
        top_left_inclusive=top_left,
        bottom_right_exclusive=bottom_right,
    )
    closest_point: tuple[int, int] = rc_utils.get_closest_pixel(cropped_depth_image)

    # We offset to give global coordinates!
    return closest_point[0] + top_left[0], closest_point[1] + top_left[1]


def get_closest_depth_at_position(
    depth_image: NDArray,
    position: tuple[int, int],
    pixel_range_x: int = 20,
    pixel_range_y: int = 20,
) -> float:
    closest_point = get_closest_depth_coordinates_at_position(
        depth_image, position, pixel_range_x, pixel_range_y
    )
    return depth_image[closest_point]


def get_center_obstacle_depth(rc) -> tuple[float, float]:
    depth_image = rc.camera.get_depth_image()

    mid_x: int = rc.camera.get_width() // 2
    mid_y: int = rc.camera.get_height() // 2

    # We identify the obstacle we are trying to avoid and we point towards it
    obstacle_coords = get_closest_depth_coordinates_at_position(
        depth_image,
        (mid_y - 20, mid_x),
        pixel_range_x=(mid_x - 1),
        pixel_range_y=40,
    )
    depth: float = depth_image[obstacle_coords]

    center_error_x = ((mid_x - obstacle_coords[1]) / mid_x) * 2

    return depth, center_error_x


class Obstacle:
    def __init__(self, rc: racecar_core.Racecar):
        depth_image = rc.camera.get_depth_image()

        mid_x: int = rc.camera.get_width() // 2
        mid_y: int = rc.camera.get_height() // 2
        offset_x: int = mid_x // 2
        offset_y: int = mid_y // 2

        # NOTE! Points are in (y-x coordinates!)

        mid_point: tuple[int, int] = (mid_y, mid_x)
        left_point: tuple[int, int] = (mid_y, mid_x - offset_x)
        right_point: tuple[int, int] = (mid_y, mid_x + offset_x)
        top_point: tuple[int, int] = (mid_y - offset_y, mid_x)
        bottom_point: tuple[int, int] = (mid_y + offset_y, mid_x)

        mid_distance: float = get_closest_depth_at_position(
            depth_image, mid_point, pixel_range_x=120, pixel_range_y=50
        )
        left_distance: float = get_closest_depth_at_position(
            depth_image, left_point, pixel_range_x=60
        )
        right_distance: float = get_closest_depth_at_position(
            depth_image, right_point, pixel_range_x=60
        )
        top_distance: float = get_closest_depth_at_position(depth_image, top_point)
        bottom_distance: float = get_closest_depth_at_position(
            depth_image, bottom_point
        )

        # In case where there is no data we must ensure it gets assigned the max!
        self._mid_distance = mid_distance or MAX_DISTANCE
        self._left_distance = left_distance or MAX_DISTANCE
        self._right_distance = right_distance or MAX_DISTANCE
        self._top_distance = top_distance or MAX_DISTANCE
        self._bottom_distance = bottom_distance or MAX_DISTANCE

        print("B Point", bottom_point, "Bottom Distance:", self._bottom_distance)

    @property
    def is_left_obstacle(self) -> bool:
        return self._left_distance < MIN_DISTANCE * 2

    @property
    def is_right_obstacle(self) -> bool:
        return self._right_distance < MIN_DISTANCE * 2

    @property
    def is_front_obstacle(self) -> bool:
        return self._mid_distance < MIN_DISTANCE * 4

    @property
    def is_general_obstacle(self) -> bool:
        return (
            (self._mid_distance < MIN_DISTANCE * 8)
            or (self._left_distance < MIN_DISTANCE * 8)
            or (self._right_distance < MIN_DISTANCE * 8)
        )

    @property
    def is_ramp_up(self) -> bool:
        return (
            self._bottom_distance < MIN_DISTANCE * 4
            and self._mid_distance < MIN_DISTANCE * 4
            and self._left_distance < MIN_DISTANCE * 4
            and self._right_distance < MIN_DISTANCE * 4
            and self._top_distance > MIN_DISTANCE * 8
        )

    @property
    def is_cliff(self) -> bool:
        return self._bottom_distance > MIN_DISTANCE * 4

    def __str__(self):
        return f"Mid: {self._mid_distance}, Left: {self._left_distance}, Right: {self._right_distance}, Top: {self._top_distance}, Bottom: {self._bottom_distance}"

    def __repr__(self):
        return f"Is Left: {self.is_left_obstacle}, Is Right: {self.is_right_obstacle}, Is General: {self.is_general_obstacle}, Is Front: {self.is_front_obstacle}, Is Ramp Up: {self.is_ramp_up}"


class Mode(Enum):
    FORWARD = 0
    OBSTACLE_STOP = 1
    VOID_STOP = 2
    RAMP = 3
    RAMP_STEEP = 4


########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

STATE: Mode = Mode.FORWARD

# Distance in centimeters
MIN_DISTANCE = 15
MAX_DISTANCE = 1_000

########################################################################################
# Functions
########################################################################################


def start():
    """
    This function is run once every time the start button is pressed
    """
    # Have the car begin at a stop
    rc.drive.stop()

    # Print start message
    print(
        ">> Lab 3A - Depth Camera Safety Stop\n"
        "\n"
        "Controls:\n"
        "    Right trigger = accelerate forward\n"
        "    Right bumper = override safety stop\n"
        "    Left trigger = accelerate backward\n"
        "    Left joystick = turn front wheels\n"
        "    A button = print current speed and angle\n"
        "    B button = print the distance at the center of the depth image"
    )


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global STATE

    # Use the triggers to control the car's speed
    rt = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    lt = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    speed = rt - lt

    # Use the left joystick to control the angle of the front wheels
    angle = rc.controller.get_joystick(rc.controller.Joystick.LEFT)[0]
    obstacle: Obstacle = Obstacle(rc)

    if STATE == Mode.FORWARD:
        if obstacle.is_general_obstacle:
            print(">> FORWARD SLOW")
            speed *= 0.25
        else:
            print(">> FORWARD FAST")

        print("-----------------")
        print(obstacle.__str__())
        print(obstacle.__repr__())

        if obstacle.is_ramp_up:
            # Drive up ramps
            STATE = Mode.RAMP
        elif obstacle.is_front_obstacle:
            # Prevent forward movement if the car is about to hit something.
            STATE = Mode.OBSTACLE_STOP
        elif obstacle.is_cliff:
            STATE = Mode.VOID_STOP
        elif obstacle.is_left_obstacle and not obstacle.is_right_obstacle:
            # Avoid left obstacle (but not if inbetween walls)
            rc.drive.set_speed_angle(speed=speed, angle=0.5 if speed > 0 else 1)
        elif obstacle.is_right_obstacle and not obstacle.is_left_obstacle:
            # Avoid right obstacle (but not if inbetween walls)
            rc.drive.set_speed_angle(speed=speed, angle=-0.5 if speed > 0 else -1)
        else:
            # If no obstacle then apply current speed/angle
            rc.drive.set_speed_angle(speed=speed, angle=angle)

    elif STATE == Mode.OBSTACLE_STOP:
        print(">> OBSTACLE_STOP")
        depth, center_error_x = get_center_obstacle_depth(rc)

        print("Depth:", depth, "Center Error X", center_error_x)

        # Reset to FORWARD Mode when no obstacle is visible
        if not obstacle.is_front_obstacle:
            STATE = Mode.FORWARD
            return

        if obstacle.is_ramp_up:
            STATE = Mode.RAMP
            return

        speed_error = (MIN_DISTANCE - depth) / MIN_DISTANCE

        # Hitting the obstacle is worse that being too far away.
        # We worsen the error if we are too close
        if speed_error > 0:
            speed_error += 0.9

        speed = rc_utils.clamp(-speed_error, min=-1, max=0.05)

        # Heuristic to stop the car rather than micro adjustments
        if (abs(speed) < 0.1) and (abs(speed_error) < 0.05):
            speed = 0

        angle_error: float = center_error_x * 2
        if abs(angle_error) > 0.95:
            angle = rc_utils.clamp(angle, min=-1, max=1)

        rc.drive.set_speed_angle(speed=speed, angle=angle)

    elif STATE == Mode.VOID_STOP:
        print(">> VOID_STOP")

        depth_image = rc.camera.get_depth_image()

        mid_x: int = rc.camera.get_width() // 2
        mid_y: int = rc.camera.get_height() // 2

        depth: float = get_closest_depth_at_position(
            depth_image,
            (mid_y + (mid_y // 2), mid_x),
            pixel_range_x=50,
            pixel_range_y=200,
        )

        if depth < MIN_DISTANCE * 4:
            rc.drive.set_speed_angle(speed=-1, angle=0)
        else:
            rc.drive.stop()

    elif STATE == Mode.RAMP:
        print(">> RAMP")

        rc.drive.set_speed_angle(speed=1, angle=0)

        # On a ramp we must simply go straight until when we see the ramp
        if (not obstacle.is_front_obstacle) and obstacle.is_cliff:
            STATE = Mode.RAMP_STEEP

    elif STATE == Mode.RAMP_STEEP:
        print(">> RAMP_STEEP")
        if not obstacle.is_cliff:
            STATE = Mode.FORWARD

    # Print the current speed and angle when the A button is held down
    if rc.controller.is_down(rc.controller.Button.A):
        print("Speed:", speed, "Angle:", angle)

    # Print the depth image center distance when the B button is held down
    if rc.controller.is_down(rc.controller.Button.B):
        print("Center distance:", center_distance)

    # TODO (stretch goal): Prevent forward movement if the car is about to drive off a
    # ledge.  ONLY TEST THIS IN THE SIMULATION, DO NOT TEST THIS WITH A REAL CAR.


def update_slow():
    """
    This function is run once every frame, but is run slowly
    """
    global center_distance
    depth_image = rc.camera.get_depth_image()
    center_distance = rc_utils.get_depth_image_center_distance(depth_image)

    # Display the current depth image
    # rc.display.show_depth_image(depth_image)


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start=start, update=update, update_slow=update_slow)
    rc.go()
