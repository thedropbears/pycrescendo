from math import pi
from dataclasses import dataclass
from wpimath.geometry import Rotation2d, Translation2d


@dataclass
class NodePosition:
    translation: Translation2d
    heading: Rotation2d


# StageHeight = 8.210550308227539
StageWidth = 16.541748046875


class NotePositions:
    # Order from the driver station 1-3
    # 1 is always the closest to the side of driver station 1
    Stage1 = NodePosition(Translation2d(13.645, 7.00045), Rotation2d(0))
    Stage2 = NodePosition(Translation2d(13.645, 5.55265), Rotation2d(0))
    Stage3 = NodePosition(Translation2d(13.645, 4.1057), Rotation2d(0))
    Centre1 = NodePosition(Translation2d(8.2956, 7.4585), Rotation2d(0))
    Centre2 = NodePosition(Translation2d(8.2956, 5.7821), Rotation2d(0))
    Centre3 = NodePosition(Translation2d(8.2956, 4.1057), Rotation2d(0))
    Centre4 = NodePosition(Translation2d(8.2956, 2.4293), Rotation2d(0))
    Centre5 = NodePosition(Translation2d(8.2956, 0.75286), Rotation2d(0))


# =(-90+x*(360/3))%360

StageLegs = [
    # Red stage legs
    NodePosition(Translation2d(5.652, 5.4015), Rotation2d(60 * pi / 180)),
    NodePosition(Translation2d(5.652, 2.8215), Rotation2d(300 * pi / 180)),
    NodePosition(Translation2d(3.367, 4.08455), Rotation2d(pi)),  # 180
    # Blue stage legs
    NodePosition(Translation2d(StageWidth - 5.652, 5.4015), Rotation2d(120 * pi / 180)),
    NodePosition(Translation2d(StageWidth - 5.652, 2.8215), Rotation2d(240 * pi / 180)),
    NodePosition(Translation2d(StageWidth - 3.367, 4.08455), Rotation2d(0)),  # 180
]


class ShootingPositions:
    Pos1 = NodePosition(Translation2d(10, 5.45585), Rotation2d(0))
