from wpimath.geometry import Rotation2d, Translation2d
from utilities.position import NotePoses, Path, NotePaths, ShootingPoses

from autonomous.base import AutoBase, rotation_to_red_speaker, combine


class All3Notes(AutoBase):
    MODE_NAME = "All notes in our half"

    def setup(self) -> None:
        self.note_paths = [
            NotePaths(
                pick_up_path=Path(
                    [NotePoses.Stage1.translation()],
                    Rotation2d(),
                ),
                shoot_path=Path(
                    [NotePoses.Stage1.translation()],
                    rotation_to_red_speaker(NotePoses.Stage1.translation()),
                ),
                pickup_offset=Translation2d(1, -0.6),
            ),
            NotePaths(
                pick_up_path=Path(
                    [NotePoses.Stage2.translation()],
                    Rotation2d(),
                ),
                shoot_path=Path(
                    [NotePoses.Stage2.translation()],
                    rotation_to_red_speaker(NotePoses.Stage2.translation()),
                ),
                pickup_offset=Translation2d(0, 1),
            ),
            NotePaths(
                pick_up_path=Path(
                    [
                        NotePoses.Stage3.translation() + Translation2d(0.3, 0.1)
                    ],  # otherwise it'll hit the stage leg
                    Rotation2d(),
                ),
                shoot_path=Path(
                    [
                        NotePoses.Stage3.translation() + Translation2d(1, 0)
                    ],  # drive it back out to avoid the stage leg when turning
                    rotation_to_red_speaker(NotePoses.Stage3.translation()),
                ),
                pickup_offset=Translation2d(0.9, 0.4),
            ),
        ]


class NearFarTop(AutoBase):
    MODE_NAME = "Near and far top notes"

    def setup(self) -> None:
        self.note_paths = [
            NotePaths(
                pick_up_path=Path(
                    [NotePoses.Stage1.translation()],
                    Rotation2d(),
                ),
                shoot_path=Path(
                    [NotePoses.Stage1.translation()],
                    rotation_to_red_speaker(NotePoses.Stage1.translation()),
                ),
                pickup_offset=Translation2d(1, -0.6),
            ),
            NotePaths(
                pick_up_path=Path(
                    [NotePoses.Centre1.translation()],
                    Rotation2d(),
                ),
                shoot_path=Path(
                    [NotePoses.Stage1.translation()],
                    rotation_to_red_speaker(NotePoses.Stage1.translation()),
                ),
                pickup_offset=Translation2d(2, 0),
            ),
        ]


class CentreNote(AutoBase):
    MODE_NAME = "Centre note"

    def setup(self) -> None:
        to = Path(
            [
                ShootingPoses.Pos2.translation(),
                Translation2d(11.312, 4.11),  # under stage 1
                NotePoses.Centre3.translation(),
            ],
            rotation_to_red_speaker(ShootingPoses.Pos2.translation()),
        )
        self.note_paths = [
            NotePaths(
                pick_up_path=to,
                shoot_path=to.copy().inverse(),
                pickup_offset=Translation2d(1, 0),
            )
        ]


class Middle3(AutoBase):
    MODE_NAME = "Centre 3 notes"

    def setup(self) -> None:
        to = Path(
            [
                ShootingPoses.Pos2.translation(),
                Translation2d(11.312, 4.11),  # under stage 1
            ],
            rotation_to_red_speaker(ShootingPoses.Pos2.translation()),
        )

        def make_path(note, offset):
            return NotePaths(
                pick_up_path=to + note,
                shoot_path=to.copy().inverse(),
                pickup_offset=Translation2d(*offset),
            )

        self.note_paths = [
            make_path(NotePoses.Centre3, (1, 0)),
            make_path(NotePoses.Centre2, (0, -1)),
            make_path(NotePoses.Centre4, (0, 1)),
        ]


class CrossField(AutoBase):
    MODE_NAME = "Cross field"

    def setup(self) -> None:
        cross_field = Path(
            [
                ShootingPoses.Pos2.translation(),
                Translation2d(11.312, 4.11),  # under stage 1
                Translation2d(5.785, 4.129),  # Start of stage 2
                Translation2d(4.3210, 3.207),  # End of stage 2
                Translation2d(1.513, 1.437),
            ],
            rotation_to_red_speaker(ShootingPoses.Pos2.translation()),
        )
        self.note_paths = [
            NotePaths(
                pick_up_path=cross_field,
                shoot_path=cross_field.copy().inverse(),
                pickup_offset=Translation2d(0.5, 0.5),
            )
        ]


class Front2Note(AutoBase):
    MODE_NAME = "Front of speaker 2 note"

    def setup(self) -> None:
        self.note_paths = [
            NotePaths(
                pick_up_path=Path(
                    [NotePoses.Stage2.translation()],
                    Rotation2d(),
                ),
                shoot_path=Path(
                    [NotePoses.Stage2.translation()],
                    rotation_to_red_speaker(NotePoses.Stage2.translation()),
                ),
                pickup_offset=Translation2d(1, 0),
            )
        ]


AllMids = combine("All middle notes", Front2Note, Middle3)
