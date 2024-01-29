import wpilib
from magicbot import feedback
from rev import CANSparkMax
from ids import SparkMaxIds, DioChannels


class ClimberComponent:
    GEAR_RATIO = 1 / 48  # using a Neo with 4:1 4:1 3:1 ratio

    def __init__(self) -> None:
        self.climbing_motor = CANSparkMax(
            SparkMaxIds.climber, CANSparkMax.MotorType.kBrushless
        )
        self.deploy_limit_switch = wpilib.DigitalInput(
            DioChannels.climber_deploy_switch
        )
        self.retract_limit_switch = wpilib.DigitalInput(
            DioChannels.climber_retract_switch
        )
        self.speed = 0

    @feedback
    def has_climb_finished(self):
        return not self.retract_limit_switch.get()

    @feedback
    def has_deploy_finished(self):
        return not self.deploy_limit_switch.get()

    def deploy(self) -> None:
        if self.has_deploy_finished():
            self.speed = 0.0
        else:
            self.speed = 1

    def retract(self) -> None:
        if self.has_climb_finished():
            self.speed = 0.0
        else:
            self.speed = -1

    def execute(self) -> None:
        self.climbing_motor.set(self.speed)
        self.speed = 0
