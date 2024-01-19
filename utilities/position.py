from dataclasses import dataclass
from wpimath.geometry import Rotation2d, Translation2d


@dataclass
class NodePosition:
    translation: Translation2d
    heading: Rotation2d


class NotePositions:
    # Order from the driver station 1-3
    # 1 is always the closest to the side of driver station 1
    Stage1 = NodePosition(Translation2d(13.69568, 6.90365), Rotation2d(0))
    Stage2 = NodePosition(Translation2d(13.69568, 5.45585), Rotation2d(0))
    Stage3 = NodePosition(Translation2d(13.69568, 4.00805), Rotation2d(0))
    Centre1 = NodePosition(Translation2d(8.270875, 7.36085), Rotation2d(0))
    Centre2 = NodePosition(Translation2d(8.270875, 5.68445), Rotation2d(0))
    Centre3 = NodePosition(Translation2d(8.270875, 4.00805), Rotation2d(0))
    Centre4 = NodePosition(Translation2d(8.270875, 2.33165), Rotation2d(0))
    Centre5 = NodePosition(Translation2d(8.270875, 0.65525), Rotation2d(0))


class ShootingPositions:
    Pos1 = NodePosition(Translation2d(10, 5.45585), Rotation2d(0))