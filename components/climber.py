from rev import CANSparkMax, SparkLimitSwitch
from ids import SparkMaxIds


class ClimberComponent:
    # TODO get real values
    CURRENT_STALL_LEVELS = 10
    CLIMB_COMPLETE_ROTATION = 20  # the number of rotations needed to complete the climb as measured on the shaft
    GEAR_RATIO = 1  # using a Neo with 4:1 4:1 3:1 ratio (numbers havent been tuned)

    def __init__(self) -> None:
        self.climbing_motor = CANSparkMax(
            SparkMaxIds.climber, CANSparkMax.MotorType.kBrushless
        )
        self.limit_switch = self.climbing_motor.getForwardLimitSwitch(
            SparkLimitSwitch.Type.kNormallyOpen
        )
        self.encoder = self.climbing_motor.getEncoder()
        self.encoder.setPositionConversionFactor(self.GEAR_RATIO)
        self.deployed = False

    def has_climb_finished(self):
        if self.encoder.getPosition() >= self.CLIMB_COMPLETE_ROTATION:
            return self.climbing_motor.getOutputCurrent() >= self.CURRENT_STALL_LEVELS
        else:
            return False

    def has_deploy_finished(self):
        return self.limit_switch.get()

    def deploy(self) -> None:
        self.deployed = True

    def retract(self) -> None:
        self.deployed = False

    def execute(self) -> None:
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
