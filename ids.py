import enum


@enum.unique
class TalonIds(enum.IntEnum):
    drive_1 = 1
    steer_1 = 5

    drive_2 = 2
    steer_2 = 6

    drive_3 = 3
    steer_3 = 7

    drive_4 = 4
    steer_4 = 8

    shooter_flywheel = 9

    intake = 11


@enum.unique
class CancoderIds(enum.IntEnum):
    swerve_1 = 1
    swerve_2 = 2
    swerve_3 = 3
    swerve_4 = 4


@enum.unique
class SparkMaxIds(enum.IntEnum):
    shooter_injector = 1
    shooter_inclinator = 2  # TODO Change to the correct ID
    climber = 3


@enum.unique
class DioChannels(enum.IntEnum):
    inclinator_encoder = 1  # TODO Change to the correct ID
    climber_limit_switch = 2
