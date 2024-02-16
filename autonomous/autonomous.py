from utilities.position import NotePositions, Path, ShootingPositions, PathPositions
from autonomous.base import AutoBase


class CloseNotes(AutoBase):
    MODE_NAME = "4 notes: internal and 3 close"

    def __init__(self) -> None:
        self.note_paths = [
            Path([NotePositions.Stage3NW]),
            Path([NotePositions.Stage2]),
            Path([NotePositions.Stage1]),
        ]
        self.shoot_paths = [
            Path([ShootingPositions.CloseStraight]),
            Path([ShootingPositions.BetweenStage1AndStage2]),
            Path([NotePositions.Stage1]),
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
            Path([PathPositions.StageTransitionUpper, NotePositions.Centre3]),
        ]
        self.shoot_paths = [
            Path([NotePositions.Stage2]),
            Path([PathPositions.StageTransitionUpper, NotePositions.Stage2]),
        ]


class Middle3(AutoBase):
    # Moving to and shooting south middle notes of the field
    MODE_NAME = "3 notes: internal, center 3, center 5"

    def __init__(self) -> None:
        self.note_paths = [
            Path(
                [
                    PathPositions.StageTransitionLowerEntry,
                    PathPositions.StageTransitionLower,
                    NotePositions.Centre3,
                ]
            ),
            Path([NotePositions.Centre5]),
        ]

        self.shoot_paths = [
            Path(
                [
                    PathPositions.StageTransitionLower,
                    PathPositions.StageTransitionLowerEntry,
                    ShootingPositions.SourceSide,
                ]
            ),
            Path([ShootingPositions.SourceSide]),
        ]
