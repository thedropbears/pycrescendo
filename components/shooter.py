from magicbot import tunable, feedback
from rev import CANSparkMax
from ids import SparkMaxIds, TalonIds, DioChannels
import phoenix6
from phoenix6.controls import VelocityVoltage
import phoenix6.hardware
from wpilib import DutyCycleEncoder
from wpimath.controller import ProfiledPIDControllerRadians
from wpimath.trajectory import TrapezoidProfileRadians
import math
from utilities.functions import clamp


class ShooterComponent:
    MAX_FLYWHEEL_SPEED = 6380 / 60
    FLYWHEEL_GEAR_RATIO = 24.0 / 18.0
    flywheel_speed = tunable(0.0)
    inject_speed = tunable(0.0)

    MAX_INCLINE_ANGLE = math.radians(25)
    MIN_INCLINE_ANGLE = math.radians(0)
    INCLINATOR_TOLERANCE = math.radians(5)

    INCLINATOR_OFFSET = 0.6632

    def __init__(self) -> None:
        self.inclinator = CANSparkMax(
            SparkMaxIds.shooter_inclinator, CANSparkMax.MotorType.kBrushless
        )
        self.inclinator.setInverted(True)
        self.inclinator_encoder = DutyCycleEncoder(DioChannels.inclinator_encoder)
        self.inclinator_encoder.setPositionOffset(self.INCLINATOR_OFFSET)
        # invert encoder and map to radians
        self.inclinator_encoder.setDistancePerRotation(-math.tau)
        self.flywheel = phoenix6.hardware.TalonFX(TalonIds.shooter_flywheel)

        flywheel_config = self.flywheel.configurator
        flywheel_motor_config = phoenix6.configs.MotorOutputConfigs()
        flywheel_motor_config.neutral_mode = phoenix6.signals.NeutralModeValue.COAST

        flywheel_pid = (
            phoenix6.configs.Slot0Configs()
            .with_k_p(3.516)
            .with_k_i(0)
            .with_k_d(0)
            .with_k_s(0.19469)
            .with_k_v(0.15649)
            .with_k_a(0.017639)
        )

        flywheel_gear_ratio = (
            phoenix6.configs.FeedbackConfigs().with_sensor_to_mechanism_ratio(
                self.FLYWHEEL_GEAR_RATIO
            )
        )

        flywheel_config.apply(flywheel_motor_config)
        flywheel_config.apply(flywheel_pid)
        flywheel_config.apply(flywheel_gear_ratio)

        self.injector = CANSparkMax(
            SparkMaxIds.shooter_injector, CANSparkMax.MotorType.kBrushless
        )
        self.injector.setInverted(False)

        self.inclinator_controller = ProfiledPIDControllerRadians(
            0.8, 0, 0, TrapezoidProfileRadians.Constraints(2, 2)
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

    @feedback
    def inclination_angle(self) -> float:
        return self.inclinator_encoder.getDistance()

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

        flywheel_request = VelocityVoltage(
            self.flywheel_speed * ShooterComponent.MAX_FLYWHEEL_SPEED
        )
        self.flywheel.set_control(flywheel_request)
        self.should_inject = False
