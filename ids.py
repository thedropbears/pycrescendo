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

    shooter_flywheel_left = 9
    shooter_flywheel_right = 10

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
    shooter_inclinator = 2
    climber = 3
    intake_deploy_l = 4
    intake_deploy_r = 5


@enum.unique
class DioChannels(enum.IntEnum):
    inclinator_encoder = 1
    injector_break_beam = 2
    climber_deploy_switch = 3
    climber_retract_switch = 4


@enum.unique
class PwmChannels(enum.IntEnum):
    led_strip = 0
