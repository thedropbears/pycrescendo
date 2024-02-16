import math

from wpimath.geometry import Rotation2d, Translation2d, Pose2d

from utilities.game import RED_SPEAKER_POSE, field_flip_pose2d


class Path:
    waypoints: list[Translation2d]
    final_heading: Rotation2d

    def __init__(self, waypoints: list[Translation2d]):
        self.waypoints = waypoints
        self.final_heading = Rotation2d(0)


class NotePositions:
    # These are the 3 close notes, named for the nearest element
    amp = Translation2d(13.645, 7.00045)
    speaker = Translation2d(13.645, 5.55265)
    podium = Translation2d(13.645, 4.1057)

    # 1 is always the closest to the amp side
    Centre1 = Translation2d(8.2956, 7.4585)
    Centre2 = Translation2d(8.2956, 5.7821)
    Centre3 = Translation2d(8.2956, 4.1057)
    Centre4 = Translation2d(8.2956, 2.4293)
    Centre5 = Translation2d(8.2956, 0.75286)

    # The podium note is very close to the stage leg so we need different positions to avoid collisions
    # Directions are in the field coordinate system for red side
    podium_NW = podium + Translation2d(0.5, 0.5)
    podium_NE = podium + Translation2d(0.5, -0.5)
    podium_N = podium + Translation2d(0.5, 0)


class ShootingPositions:
    close_straight = Translation2d(15, RED_SPEAKER_POSE.y)
    amp_speaker_bounce = Translation2d(
        14.7, (NotePositions.amp.y + NotePositions.speaker.y) / 2
    )
    source_side = Translation2d(14.7, 2.8)


class TeamPoses:
    RED_TEST_POSE = Pose2d(15.1, 5.5, math.pi)
    BLUE_TEST_POSE = field_flip_pose2d(RED_TEST_POSE)
    BLUE_PODIUM = Pose2d(Translation2d(2.992, 4.08455), Rotation2d(math.pi))
    RED_PODIUM = field_flip_pose2d(BLUE_PODIUM)
class PathPositions:
    stage_transition_N = Translation2d(11.4, 4.5)
    stage_transition_S = Translation2d(11.4, 3.74)
    stage_transition_S_entry = Translation2d(13.0, 2.5)
