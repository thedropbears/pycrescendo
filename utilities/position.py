from dataclasses import dataclass
from wpimath.geometry import Rotation2d, Translation2d


@dataclass
class NodePosition:
    translation: Translation2d
    heading: Rotation2d


class NotePosition:
    # Order from the driver station 1-3
    # 1 is always the closest to the side of driver station 1
    NoteStage1 = NodePosition(Translation2d(13.69568, 6.90365), Rotation2d(0))
    NoteStage2 = NodePosition(Translation2d(13.69568, 5.45585), Rotation2d(0))
    NoteStage3 = NodePosition(Translation2d(13.69568, 4.00805), Rotation2d(0))
    NoteCentre1 = NodePosition(Translation2d(8.270875, 7.36085), Rotation2d(0))
    NoteCentre2 = NodePosition(Translation2d(8.270875, 5.68445), Rotation2d(0))
    NoteCentre3 = NodePosition(Translation2d(8.270875, 4.00805), Rotation2d(0))
    NoteCentre4 = NodePosition(Translation2d(8.270875, 2.33165), Rotation2d(0))
    NoteCentre5 = NodePosition(Translation2d(8.270875, 0.65525), Rotation2d(0))


class ShootPosition:
    ShootPos1 = NodePosition(Translation2d(10, 5.45585), Rotation2d(0))
