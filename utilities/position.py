from math import pi

from dataclasses import dataclass
from wpimath.geometry import Rotation2d, Translation2d, Pose2d

# Is this redundant? -> Using Pose2d instead
# @dataclass
# class NodePosition:
#     translation: Translation2d
#     heading: Rotation2d


@dataclass
class Path:
    waypoints: list[Translation2d]
    final_heading: Rotation2d

    def copy(self):
        return Path(
            [Translation2d(i.x, i.y) for i in self.waypoints],
            Rotation2d(self.final_heading.radians()),
        )

    def __add__(self, other):
        return Path(self.waypoints + other.waypoints, self.final_heading)

    def __iter__(self):
        return iter(self.waypoints)

    def __getitem__(self, __i: int | slice):
        return self.waypoints[__i]

    def __setitem__(self, __key: int, __value: Translation2d):
        self.waypoints[__key] = __value

    def __delitem__(self, __key: int | slice):
        del self.waypoints[__key]

    def __len__(self):
        return len(self.waypoints)


@dataclass
class NotePaths:
    # All paths assume RED alliance
    # They will automatically be flipped if we are blue
    pick_up_path: Path
    shoot_path: Path
    pickup_offset: Translation2d


# StageHeight = 8.210550308227539
StageWidth = 16.541748046875


class NotePoses:
    # Order from the driver station 1-3
    # 1 is always the closest to the side of driver station 1
    Stage1 = Pose2d(Translation2d(13.645, 7.00045), Rotation2d(0))
    Stage2 = Pose2d(Translation2d(13.645, 5.55265), Rotation2d(0))
    Stage3 = Pose2d(Translation2d(13.645, 4.1057), Rotation2d(0))
    Centre1 = Pose2d(Translation2d(8.2956, 7.4585), Rotation2d(0))
    Centre2 = Pose2d(Translation2d(8.2956, 5.7821), Rotation2d(0))
    Centre3 = Pose2d(Translation2d(8.2956, 4.1057), Rotation2d(0))
    Centre4 = Pose2d(Translation2d(8.2956, 2.4293), Rotation2d(0))
    Centre5 = Pose2d(Translation2d(8.2956, 0.75286), Rotation2d(0))


# =(-90+x*(360/3))%360

StageLegs: list[Pose2d] = [
    # Red stage legs
    Pose2d(Translation2d(5.652, 5.4015), Rotation2d(60 * pi / 180)),
    Pose2d(Translation2d(5.652, 2.8215), Rotation2d(300 * pi / 180)),
    Pose2d(Translation2d(3.367, 4.08455), Rotation2d(pi)),  # 180
    # Blue stage legs
    Pose2d(Translation2d(StageWidth - 5.652, 5.4015), Rotation2d(120 * pi / 180)),
    Pose2d(Translation2d(StageWidth - 5.652, 2.8215), Rotation2d(240 * pi / 180)),
    Pose2d(Translation2d(StageWidth - 3.367, 4.08455), Rotation2d(0)),  # 180
]


class ShootingPoses:
    Pos1 = Pose2d(Translation2d(10, 5.45585), Rotation2d(0))
