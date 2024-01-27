from magicbot import tunable, feedback
from rev import CANSparkMax
from ids import SparkMaxIds, TalonIds, DioChannels

from phoenix6.controls import VelocityVoltage
from phoenix6.hardware import TalonFX
from phoenix6.configs import MotorOutputConfigs, Slot0Configs, FeedbackConfigs
from phoenix6.signals import NeutralModeValue
from wpilib import DigitalInput, DutyCycle

from wpimath.controller import ProfiledPIDControllerRadians
from wpimath.trajectory import TrapezoidProfileRadians
import math
from utilities.functions import clamp


class ShooterComponent:
    FLYWHEEL_GEAR_RATIO = 24.0 / 18.0
    desired_flywheel_speed = tunable(0.0)
    inject_speed = tunable(0.0)

    MAX_INCLINE_ANGLE = 0.973  # ~55 degrees
    MIN_INCLINE_ANGLE = math.radians(20)
    INCLINATOR_TOLERANCE = math.radians(1)

    INCLINATOR_OFFSET = 0.822 * math.tau - math.radians(20)
    INCLINATOR_SCALE_FACTOR = math.tau  # rev -> radians

    def __init__(self) -> None:
        self.inclinator = CANSparkMax(
            SparkMaxIds.shooter_inclinator, CANSparkMax.MotorType.kBrushless
        )
        self.inclinator_encoder = DutyCycle(
            DigitalInput(DioChannels.inclinator_encoder)
        )
        self.flywheel = TalonFX(TalonIds.shooter_flywheel)

        flywheel_config = self.flywheel.configurator
        flywheel_motor_config = MotorOutputConfigs()
        flywheel_motor_config.neutral_mode = NeutralModeValue.COAST

        flywheel_pid = (
            Slot0Configs()
            .with_k_p(0.3514)
            .with_k_i(0)
            .with_k_d(0)
            .with_k_s(0.19469)
            .with_k_v(0.15649)
            .with_k_a(0.017639)
        )

        flywheel_gear_ratio = FeedbackConfigs().with_sensor_to_mechanism_ratio(
            self.FLYWHEEL_GEAR_RATIO
        )

        flywheel_config.apply(flywheel_motor_config)
        flywheel_config.apply(flywheel_pid)
        flywheel_config.apply(flywheel_gear_ratio)

        self.injector = CANSparkMax(
            SparkMaxIds.shooter_injector, CANSparkMax.MotorType.kBrushless
        )
        self.injector.setInverted(False)

        self.inclinator_controller = ProfiledPIDControllerRadians(
            3.0, 0, 0, TrapezoidProfileRadians.Constraints(math.pi, math.pi)
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
    def encoder_raw(self):
        return self.inclinator_encoder.getOutput()

    @feedback
    def at_inclination(self) -> bool:
        return self.inclinator_controller.atGoal()

    @feedback
    def inclination_angle(self) -> float:
        return (
            self.inclinator_encoder.getOutput() * self.INCLINATOR_SCALE_FACTOR
            - self.INCLINATOR_OFFSET
        )

    @feedback
    def actual_flywheel_speed(self) -> float:
        return self.flywheel.get_velocity().value

    def execute(self) -> None:
        """This gets called at the end of the control loop"""

        inclinator_speed = self.inclinator_controller.calculate(
            self.inclination_angle()
        )
        self.inclinator.set(inclinator_speed)

        if self.should_inject and self.at_inclination():
            self.injector.set(self.inject_speed)
        else:
            self.injector.set(0.0)

        flywheel_request = VelocityVoltage(self.desired_flywheel_speed)
        self.flywheel.set_control(flywheel_request)
        self.should_inject = False
