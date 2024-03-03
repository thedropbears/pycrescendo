from utilities.position import NotePositions, Path, ShootingPositions, PathPositions
from autonomous.base import AutoBase, rotation_to_red_speaker
from wpimath.geometry import Pose2d, Translation2d


class PodiumSpeakerAmp(AutoBase):
    MODE_NAME = "4 notes: internal, podium, speaker, amp"

    def __init__(self) -> None:
        note_paths = [
            Path([NotePositions.podium_NW]),
            Path([NotePositions.speaker]),
            Path([NotePositions.amp]),
        ]
        shoot_paths = [
            Path([ShootingPositions.close_straight]),
            Path([ShootingPositions.amp_speaker_bounce]),
            Path([NotePositions.amp]),
        ]
        super().__init__(note_paths, shoot_paths)


class AmpCentre1(AutoBase):
    # The "top" or north notes of the field
    MODE_NAME = "3 notes: internal, amp, centre 1"

    def __init__(self) -> None:
        note_paths = [
            Path([NotePositions.amp]),
            Path([PathPositions.avoid_wall, NotePositions.Centre1]),
        ]
        shoot_paths = [
            Path([NotePositions.amp]),
            Path([PathPositions.avoid_wall, NotePositions.amp]),
        ]
        super().__init__(note_paths, shoot_paths)


class SpeakerCentre3(AutoBase):
    # Moving through the stage
    MODE_NAME = "3 notes: internal, speaker, centre 3"

    def __init__(self) -> None:
        note_paths = [
            Path([NotePositions.speaker]),
            Path([PathPositions.stage_transition_N, NotePositions.Centre3]),
        ]
        shoot_paths = [
            Path([NotePositions.speaker]),
            Path([PathPositions.stage_transition_N, NotePositions.speaker]),
        ]
        super().__init__(note_paths, shoot_paths)


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
                ]
            ),
            Path([NotePositions.Centre5]),
        ]

        shoot_paths = [
            Path(
                [
                    PathPositions.stage_transition_S,
                    PathPositions.stage_transition_S_entry,
                    ShootingPositions.source_side,
                ]
            ),
            Path([ShootingPositions.source_side]),
        ]
        sim_start_pos = Translation2d(15.4, 2.94)
        rotation = rotation_to_red_speaker(sim_start_pos)
        sim_start_pose = Pose2d(sim_start_pos, rotation)
        super().__init__(note_paths, shoot_paths, sim_start_pose)
