from magicbot import tunable, feedback
from rev import CANSparkMax
from ids import SparkMaxIds, TalonIds, DioChannels
import phoenix6
from phoenix6.controls import VoltageOut
import phoenix6.hardware
from wpilib import DutyCycleEncoder
from wpimath.controller import ProfiledPIDControllerRadians
from wpimath.trajectory import TrapezoidProfileRadians
import math
from utilities.functions import clamp


class ShooterComponent:
    flywheel_speed = tunable(0.0)
    inject_speed = tunable(0.0)

    # TODO Figure that out
    MAX_INCLINE_ANGLE = 45
    MIN_INCLINE_ANGLE = 10
    INCLINATOR_TOLERANCE = math.radians(5)

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

        self.inclinator_controller = ProfiledPIDControllerRadians(
            1, 0, 0, TrapezoidProfileRadians.Constraints(2, 2)
        )
        self.inclinator_controller.setTolerance(ShooterComponent.INCLINATOR_TOLERANCE)

        self.should_inject = False

    def set_inclination(self, angle: float) -> None:
        self.inclinator_controller.setGoal(
            clamp(
                angle,
                ShooterComponent.MIN_INCLINE_ANGLE,
                ShooterComponent.MAX_INCLINE_ANGLE,
            )
        )

    def shoot(self) -> None:
        self.should_inject = True

    @feedback
    def is_ready(self) -> bool:
        return True

    @feedback
    def at_inclination(self) -> bool:
        return self.inclinator_controller.atGoal()

    def execute(self) -> None:
        """This gets called at the end of the control loop"""
        flywheel_request = VoltageOut(12.0 * self.flywheel_speed)
        inclinator_speed = self.inclinator_controller.calculate(
            self.inclinator_encoder.get() * math.tau
        )
        self.inclinator.set(inclinator_speed)

        if self.should_inject and self.at_inclination():
            self.injector.set(self.inject_speed)
        else:
            self.injector.set(0.0)

        self.flywheel.set_control(flywheel_request)
        self.should_inject = False
