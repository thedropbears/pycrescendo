from dataclasses import dataclass
from wpimath.geometry import Rotation2d, Translation2d, Pose2d


@dataclass
class Path:
    waypoints: list[Translation2d]
    final_heading: Rotation2d


def Make_Path(
    current_pose: Pose2d, waypoints: list[Translation2d], final_heading: Rotation2d
) -> Path:
    """This is to make a path but take into account the stage legs and avoid them, using "pathfinding" (My own less-rich algorithms) to do so quickly"""

    return Path(waypoints, final_heading)


@dataclass
class NotePaths:
    # All paths assume RED alliance
    # They will automatically be flipped if we are blue
    pick_up_path: Path
    shoot_path: Path
