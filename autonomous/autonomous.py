from utilities.position import NotePositions, Path, ShootingPositions, PathPositions
from autonomous.base import AutoBase


class PodiumSpeakerAmp(AutoBase):
    MODE_NAME = "4 notes: internal, podium, speaker, amp"

    def __init__(self) -> None:
        self.note_paths = [
            Path([NotePositions.podium_NW]),
            Path([NotePositions.speaker]),
            Path([NotePositions.amp]),
        ]
        self.shoot_paths = [
            Path([ShootingPositions.close_straight]),
            Path([ShootingPositions.amp_speaker_bounce]),
            Path([NotePositions.amp]),
        ]


class AmpCentre1(AutoBase):
    # The "top" or north notes of the field
    MODE_NAME = "3 notes: internal, amp, centre 1"

    def __init__(self) -> None:
        self.note_paths = [
            Path([NotePositions.amp]),
            Path([NotePositions.Centre1]),
        ]
        self.shoot_paths = [
            Path([NotePositions.amp]),
            Path([NotePositions.amp]),
        ]


class SpeakerCentre3(AutoBase):
    # Moving through the stage
    MODE_NAME = "3 notes: internal, speaker, centre 3"

    def __init__(self) -> None:
        self.note_paths = [
            Path([NotePositions.speaker]),
            Path([PathPositions.stage_transition_N, NotePositions.Centre3]),
        ]
        self.shoot_paths = [
            Path([NotePositions.speaker]),
            Path([PathPositions.stage_transition_N, NotePositions.speaker]),
        ]


class Centre3Centre5(AutoBase):
    # Stay in the south of the field to avoid interfering with allies using the close notes
    MODE_NAME = "3 notes: internal, center 3, center 5"

    def __init__(self) -> None:
        self.note_paths = [
            Path(
                [
                    PathPositions.stage_transition_S_entry,
                    PathPositions.stage_transition_S,
                    NotePositions.Centre3,
                ]
            ),
            Path([NotePositions.Centre5]),
        ]

        self.shoot_paths = [
            Path(
                [
                    PathPositions.stage_transition_S,
                    PathPositions.stage_transition_S_entry,
                    ShootingPositions.source_side,
                ]
            ),
            Path([ShootingPositions.source_side]),
        ]
