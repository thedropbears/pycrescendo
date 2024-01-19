from magicbot import tunable, feedback
from rev import CANSparkMax
from ids import SparkMaxIds, TalonIds, DioChannels
import phoenix6
from phoenix6.controls import VoltageOut
import phoenix6.hardware
from wpilib import DutyCycleEncoder


class ShooterComponent:
    flywheel_speed = tunable(0.0)
    inject_speed = tunable(0.0)

    # TODO Figure that out
    MAX_INCLINE_ANGLE = 45
    MIN_INCLINE_ANGLE = 10

    def __init__(self) -> None:
        self.inclinator = CANSparkMax(
            SparkMaxIds.shooter_inclinator, CANSparkMax.MotorType.kBrushless
        )
        self.inclinator_encoder = DutyCycleEncoder(DioChannels.inclinator_encoder)
        self.flywheel = phoenix6.hardware.TalonFX(TalonIds.shooter_flywheel)
        self.injector = CANSparkMax(
            SparkMaxIds.shooter_injector, CANSparkMax.MotorType.kBrushless
        )
        self.injector.setInverted(True)

        self.should_inject = False

    def shoot(self) -> None:
        self.should_inject = True

    @feedback
    def is_ready(self) -> bool:
        return True

    @feedback
    def at_inclination(self) -> bool:
        pass

    def execute(self) -> None:
        """This gets called at the end of the control loop"""
        flywheel_request = VoltageOut(12.0 * self.flywheel_speed)
        if self.should_inject:
            self.injector.set(self.inject_speed)
        else:
            self.injector.set(0.0)

        self.flywheel.set_control(flywheel_request)
        self.should_inject = False
