import math

from utilities.position import (
    NotePositions,
    Path,
    ShootingPositions,
    PathPositions,
)
from utilities import game
from autonomous.base import AutoBase
from wpimath.geometry import Pose2d, Translation2d, Rotation2d


def rotation_to_red_speaker(position: Translation2d) -> Rotation2d:
    t = game.RED_SPEAKER_POSE.toPose2d().translation() - position
    return t.angle() + Rotation2d(math.pi)


class PodiumSpeakerAmpTopcentre(AutoBase):
    MODE_NAME = "5 notes: internal, podium, speaker, amp, top centre"

    def __init__(self) -> None:
        note_paths = [
            Path([NotePositions.podium_NW], face_target=False),
            Path([NotePositions.speaker], face_target=False),
            Path([NotePositions.amp], face_target=False),
            Path([PathPositions.avoid_wall, NotePositions.Centre1], face_target=False),
        ]
        shoot_paths = [
            Path([ShootingPositions.close_straight], face_target=True),
            Path([ShootingPositions.amp_speaker_bounce], face_target=True),
            Path([NotePositions.amp], face_target=True),
            Path([PathPositions.avoid_wall, NotePositions.amp], face_target=True),
        ]
        super().__init__(note_paths, shoot_paths)


class PodiumSpeakerAmp(AutoBase):
    MODE_NAME = "4 notes: internal, podium, speaker, amp"

    def __init__(self) -> None:
        note_paths = [
            Path([NotePositions.podium_NW], face_target=False),
            Path([NotePositions.speaker], face_target=False),
            Path([NotePositions.amp], face_target=False),
        ]
        shoot_paths = [
            Path([ShootingPositions.close_straight], face_target=True),
            Path([ShootingPositions.amp_speaker_bounce], face_target=True),
            Path([NotePositions.amp], face_target=True),
        ]
        # Start pose only needs to be on the correct half of the field,
        # so choose the podium as a reference point
        start_pose = Pose2d(
            NotePositions.podium, rotation_to_red_speaker(NotePositions.podium)
        )
        super().__init__(note_paths, shoot_paths, start_pose)


class AmpCentre1(AutoBase):
    # The "top" or north notes of the field
    MODE_NAME = "3 notes: internal, amp, centre 1"

    def __init__(self) -> None:
        note_paths = [
            Path([NotePositions.amp], face_target=False),
            Path([PathPositions.avoid_wall, NotePositions.Centre1], face_target=False),
        ]
        shoot_paths = [
            Path([NotePositions.amp], face_target=True),
            Path([PathPositions.avoid_wall, NotePositions.amp], face_target=True),
        ]
        # Start pose only needs to be on the correct half of the field,
        # so choose the amp as a reference point
        start_pose = Pose2d(
            NotePositions.amp, rotation_to_red_speaker(NotePositions.amp)
        )
        super().__init__(note_paths, shoot_paths, start_pose)


class SpeakerCentre3(AutoBase):
    # Moving through the stage
    MODE_NAME = "3 notes: internal, speaker, centre 3"

    def __init__(self) -> None:
        note_paths = [
            Path([NotePositions.speaker], face_target=False),
            Path(
                [PathPositions.stage_transition_N, NotePositions.Centre3],
                face_target=False,
            ),
        ]
        shoot_paths = [
            Path([NotePositions.speaker], face_target=True),
            Path(
                [PathPositions.stage_transition_N, NotePositions.speaker],
                face_target=True,
            ),
        ]
        # Start pose only needs to be on the correct half of the field,
        # so choose the speaker as a reference point
        start_pose = Pose2d(
            NotePositions.speaker, rotation_to_red_speaker(NotePositions.speaker)
        )
        super().__init__(note_paths, shoot_paths, start_pose)


class Centre3Centre5(AutoBase):
    # Stay in the south of the field to avoid interfering with allies using the close notes
    MODE_NAME = "3 notes: internal, center 3, center 5"
    # Not yet tested
    DISABLED = True

    def __init__(self) -> None:
        note_paths = [
            Path(
                [
                    PathPositions.stage_transition_S_entry,
                    PathPositions.stage_transition_S,
                    NotePositions.Centre3,
                ],
                face_target=False,
            ),
            Path([NotePositions.Centre5], face_target=False),
        ]

        shoot_paths = [
            Path(
                [
                    PathPositions.stage_transition_S,
                    PathPositions.stage_transition_S_entry,
                    ShootingPositions.source_side,
                ],
                face_target=True,
            ),
            Path([ShootingPositions.source_side], face_target=True),
        ]
        sim_start_pos = Translation2d(15.4, 2.94)
        rotation = rotation_to_red_speaker(sim_start_pos)
        sim_start_pose = Pose2d(sim_start_pos, rotation)
        super().__init__(note_paths, shoot_paths, sim_start_pose)
