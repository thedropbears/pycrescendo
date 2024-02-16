from utilities.position import NotePositions, Path, ShootingPoses
from autonomous.base import AutoBase


class All3Notes(AutoBase):
    MODE_NAME = "All notes in our half"

    def __init__(self) -> None:
        self.note_paths = [
            Path([NotePositions.Stage3NW]),
            Path([NotePositions.Stage2]),
            Path([NotePositions.Stage1]),
        ]
        self.shoot_paths = [
            Path([ShootingPoses.CloseStraight]),
            Path([ShootingPoses.CloseStraight]),
            Path([ShootingPoses.CloseStraight]),
        ]
