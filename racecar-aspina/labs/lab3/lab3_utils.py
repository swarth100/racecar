import sys
from dataclasses import dataclass, field

from nptyping import NDArray

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils


def get_closest_depth_coordinates_at_position(
    depth_image: NDArray,
    position: tuple[int, int],
    pixel_range_x: int = 20,
    pixel_range_y: int = 20,
) -> tuple[int, int]:
    """
    Returns the coordinates of the closest depth pixel in the vicinity of the position.

    :param depth_image: The depth image to search in.
    :param position: The position to search around (passed as (Y, X).
    :param pixel_range_x: The range in the x-axis to search around.
    :param pixel_range_y: The range in the y-axis to search around.
    :return: The coordinates of the closest depth pixel.
    """
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
    """
    Returns the depth of the closest depth pixel in the vicinity of the position.

    :param depth_image: The depth image to search in.
    :param position: The position to search around (passed as (Y, X).
    :param pixel_range_x: The range in the x-axis to search around.
    :param pixel_range_y: The range in the y-axis to search around.
    :return: The depth of the closest depth pixel.
    """
    closest_point = get_closest_depth_coordinates_at_position(
        depth_image, position, pixel_range_x, pixel_range_y
    )
    return depth_image[closest_point]


# -------------------------------------------------------------------------------------------------------------------- #

# Number of depth measurements to average for determining direction
NUM_DIRECTION_MEASUREMENTS: int = 10


@dataclass
class Direction:
    measurements: list[float] = field(default_factory=list)

    def add_measurement(self, measurement: float):
        self.measurements.append(measurement)

        # We add depths to the end of the list, and we pop front to cycle through
        if len(self.measurements) > NUM_DIRECTION_MEASUREMENTS:
            self.measurements.pop(0)

    def is_forward(self, comp_measurement: float) -> bool:
        if len(self.measurements) != NUM_DIRECTION_MEASUREMENTS:
            return True

        return comp_measurement < (sum(self.measurements) / len(self.measurements))

    def reset(self):
        self.measurements.clear()
