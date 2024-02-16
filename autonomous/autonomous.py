from utilities.position import NotePositions, Path, ShootingPositions, PathPositions
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
            Path([ShootingPositions.CloseStraight]),
            Path([ShootingPositions.CloseStraight]),
            Path([ShootingPositions.CloseStraight]),
        ]


class Amp1(AutoBase):
    # The "top" or north notes of the field
    MODE_NAME = "3 notes: internal, amp and centre 1"

    def __init__(self) -> None:
        self.note_paths = [
            Path([NotePositions.Stage1]),
            Path([NotePositions.Centre1]),
        ]
        self.shoot_paths = [
            Path([NotePositions.Stage1]),
            Path([NotePositions.Stage1]),
        ]


class Speaker3(AutoBase):
    # Moving through the stage
    MODE_NAME = "3 notes: internal, speaker and centre 3"

    def __init__(self) -> None:
        self.note_paths = [
            Path([NotePositions.Stage2]),
            Path([PathPositions.Stage2Centre3Transition, NotePositions.Centre3]),
        ]
        self.shoot_paths = [
            Path([NotePositions.Stage2]),
            Path([PathPositions.Stage2Centre3Transition, NotePositions.Stage2]),
        ]
