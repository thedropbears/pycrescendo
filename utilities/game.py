import typing

import robotpy_apriltag
import wpilib
from wpimath.geometry import Pose3d, Translation3d

apriltag_layout = robotpy_apriltag.loadAprilTagLayoutField(
    robotpy_apriltag.AprilTagField.k2024Crescendo
)

TagId = typing.Literal[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16]

get_fiducial_pose = typing.cast(
    typing.Callable[[TagId], Pose3d],
    apriltag_layout.getTagPose,
)

RED_SPEAKER_POSE = get_fiducial_pose(4)
BLUE_SPEAKER_POSE = get_fiducial_pose(7)


# This will default to the blue alliance if a proper link to the driver station has not yet been established
def is_red() -> bool:
    return wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed


def get_goal_speaker_position() -> Translation3d:
    if is_red():
        return RED_SPEAKER_POSE.translation()

    return BLUE_SPEAKER_POSE.translation()
