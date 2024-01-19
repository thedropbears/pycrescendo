import wpilib
from rev import CANSparkMax
from ids import SparkMaxIds


class ClimberComponent:
    def __init__(self) -> None:
        self.deployed = False

    def deploy(self) -> None:
        self.deployed = True

    def execute(self) -> None:
        if self.deployed:

            def __init__(self) -> None:
                self.deployed = False

    def deploy(self) -> None:
        self.deployed = True

    def execute(self) -> None:
        if self.deployed:
            # Deploy the climber
            self.climbingmotor = CANSparkMax(
                SparkMaxIds.climber, CANSparkMax.MotorType.kBrushless
            )  # using a Neo with 4:1 4:1 3:1 ratio
            self.limitswitch = wpilib.LimitSwitch(2)

            def motorCurrent():
                return CANSparkMax.getOutputCurrent

            TriggeringLevels = 100  # TriggeringLevels Placeholder for the current levels needed to stop
            limitswitchHit = False  # placeholder for limit switch variable
            if limitswitchHit == False:
                self.drive.motor(0.5, 0)
            elif limitswitchHit == True & motorCurrent() <= TriggeringLevels:
                self.drive.motor(-0.5, 0)
            elif limitswitchHit == True & motorCurrent() >= TriggeringLevels:
                self.drive.motor(0, 0)
            else:
                pass
        else:
            pass

