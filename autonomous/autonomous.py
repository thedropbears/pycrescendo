import math

from utilities.position import (
    NotePositions,
    Path,
    ShootingPositions,
    PathPositions,
    TeamPoses,
)
from utilities import game
from autonomous.base import AutoBase
from wpimath.geometry import Pose2d, Translation2d, Rotation2d


def rotation_to_red_speaker(position: Translation2d) -> Rotation2d:
    t = game.RED_SPEAKER_POSE.toPose2d().translation() - position
    return t.angle() + Rotation2d(math.pi)


class PodiumSpeakerAmpCentre1(AutoBase):
    MODE_NAME = "5 notes: podium, speaker, amp, centre 1"

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
    MODE_NAME = "4 notes: podium, speaker, amp"

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
        # so choose the subwoofer
        start_pose = Pose2d(
            TeamPoses.RED_TEST_POSE.translation(),
            rotation_to_red_speaker(TeamPoses.RED_TEST_POSE.translation()),
        )
        super().__init__(note_paths, shoot_paths, start_pose)


class AmpCentre1(AutoBase):
    # The "top" or north notes of the field
    MODE_NAME = "3 notes: amp, centre 1"

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
        # so choose the subwoofer
        start_pose = Pose2d(
            TeamPoses.RED_TEST_POSE.translation(),
            rotation_to_red_speaker(TeamPoses.RED_TEST_POSE.translation()),
        )
        super().__init__(note_paths, shoot_paths, start_pose)


class AmpCentre1Centre2(AutoBase):
    # The "top" or north notes of the field
    MODE_NAME = "4 notes: amp, centre 1, centre 2"

    def __init__(self) -> None:
        note_paths = [
            Path([NotePositions.amp], face_target=False),
            Path([PathPositions.avoid_wall, NotePositions.Centre1], face_target=False),
            Path([PathPositions.avoid_wall, NotePositions.Centre2], face_target=False),
        ]
        shoot_paths = [
            Path([NotePositions.amp], face_target=True),
            Path([PathPositions.avoid_wall, NotePositions.amp], face_target=True),
            Path([PathPositions.avoid_wall, NotePositions.amp], face_target=True),
        ]
        # Start pose only needs to be on the correct half of the field,
        # so choose the subwoofer
        start_pose = Pose2d(
            TeamPoses.RED_TEST_POSE.translation(),
            rotation_to_red_speaker(TeamPoses.RED_TEST_POSE.translation()),
        )
        super().__init__(note_paths, shoot_paths, start_pose)


class SpeakerCentre3(AutoBase):
    # Moving through the stage
    MODE_NAME = "3 notes: speaker, centre 3"

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
        # so choose the subwoofer
        start_pose = Pose2d(
            TeamPoses.RED_TEST_POSE.translation(),
            rotation_to_red_speaker(TeamPoses.RED_TEST_POSE.translation()),
        )
        super().__init__(note_paths, shoot_paths, start_pose)


class SpeakerCentre3Centre4(AutoBase):
    # Moving through the stage
    MODE_NAME = "4 notes: speaker, centre 3, centre 4"

    def __init__(self) -> None:
        note_paths = [
            Path([NotePositions.speaker], face_target=False),
            Path(
                [PathPositions.stage_transition_N, NotePositions.Centre3],
                face_target=False,
            ),
            Path(
                [PathPositions.stage_transition_N, NotePositions.Centre4],
                face_target=False,
            ),
        ]
        shoot_paths = [
            Path([NotePositions.speaker], face_target=True),
            Path(
                [PathPositions.stage_transition_N, NotePositions.speaker],
                face_target=True,
            ),
            Path(
                [PathPositions.stage_transition_N, NotePositions.speaker],
                face_target=True,
            ),
        ]
        # Start pose only needs to be on the correct half of the field,
        # so choose the subwoofer
        start_pose = Pose2d(
            TeamPoses.RED_TEST_POSE.translation(),
            rotation_to_red_speaker(TeamPoses.RED_TEST_POSE.translation()),
        )
        super().__init__(note_paths, shoot_paths, start_pose)


class Centre5Centre4(AutoBase):
    # Stay in the south of the field to avoid interfering with allies using the close notes
    MODE_NAME = "3 notes: centre 5, centre 4"

    def __init__(self) -> None:
        note_paths = [
            Path(
                [PathPositions.avoid_stage_S, NotePositions.Centre5], face_target=False
            ),
            Path(
                [
                    PathPositions.avoid_stage_S,
                    NotePositions.Centre4,
                ],
                face_target=False,
            ),
        ]

        shoot_paths = [
            Path(
                [PathPositions.avoid_stage_S, ShootingPositions.source_side],
                face_target=True,
            ),
            Path(
                [
                    PathPositions.avoid_stage_S,
                    ShootingPositions.source_side,
                ],
                face_target=True,
            ),
        ]
        sim_start_pos = ShootingPositions.source_side
        rotation = rotation_to_red_speaker(sim_start_pos)
        sim_start_pose = Pose2d(sim_start_pos, rotation)
        super().__init__(note_paths, shoot_paths, sim_start_pose)


class Centre5Centre4Centre3(AutoBase):
    # Stay in the south of the field to avoid interfering with allies using the close notes
    MODE_NAME = "4 notes: centre 5, centre 4, centre 3"

    def __init__(self) -> None:
        note_paths = [
            Path(
                [PathPositions.avoid_stage_S, NotePositions.Centre5], face_target=False
            ),
            Path(
                [
                    PathPositions.avoid_stage_S,
                    NotePositions.Centre4,
                ],
                face_target=False,
            ),
            Path(
                [
                    PathPositions.stage_transition_S_entry,
                    PathPositions.stage_transition_S,
                    NotePositions.Centre3,
                ],
                face_target=False,
            ),
        ]

        shoot_paths = [
            Path(
                [PathPositions.avoid_stage_S, ShootingPositions.source_side],
                face_target=True,
            ),
            Path(
                [
                    PathPositions.avoid_stage_S,
                    ShootingPositions.source_side,
                ],
                face_target=True,
            ),
            Path(
                [
                    PathPositions.stage_transition_S,
                    PathPositions.stage_transition_S_entry,
                    ShootingPositions.source_side,
                ],
                face_target=True,
            ),
        ]
        sim_start_pos = ShootingPositions.source_side
        rotation = rotation_to_red_speaker(sim_start_pos)
        sim_start_pose = Pose2d(sim_start_pos, rotation)
        super().__init__(note_paths, shoot_paths, sim_start_pose)
