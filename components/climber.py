import wpilib
from magicbot import feedback, will_reset_to
from rev import CANSparkMax
from ids import SparkMaxIds, DioChannels


class ClimberComponent:
    GEAR_RATIO = 1 / 48  # using a Neo with 4:1 4:1 3:1 ratio
    speed = will_reset_to(0.0)

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
        self.deployed = False

    @feedback
    def has_climb_finished(self):
        return not self.retract_limit_switch.get()

    @feedback
    def has_deploy_finished(self):
        return not self.deploy_limit_switch.get()

    def deploy(self) -> None:
        self.deployed = True
        if self.has_deploy_finished():
            self.speed = 0.0
        else:
            self.speed = 0.5

    def retract(self) -> None:
        self.deployed = False
        if self.has_climb_finished():
            self.speed = 0.0
        else:
            self.speed = -0.5

    def execute(self) -> None:
        self.climbing_motor.set(self.speed)
