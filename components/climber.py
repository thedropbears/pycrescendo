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
        self.deployed = False
        self.stopped = True

    @feedback
    def has_climb_finished(self):
        return not self.retract_limit_switch.get()

    @feedback
    def has_deploy_finished(self):
        return not self.deploy_limit_switch.get()

    def stop(self):
        self.stopped = True

    def deploy(self) -> None:
        self.deployed = True
        self.stopped = False

    def retract(self) -> None:
        self.deployed = False
        self.stopped = False

    def execute(self) -> None:
        if self.stopped:
            self.climbing_motor.set(0)
        else:
            if self.deployed:
                # Deploy the climber
                if self.has_deploy_finished():
                    self.climbing_motor.set(0)
                else:
                    self.climbing_motor.set(0.5)
            else:
                if self.has_climb_finished():
                    self.climbing_motor.set(0)
                else:
                    self.climbing_motor.set(-0.5)
