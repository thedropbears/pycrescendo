import math
import numpy as np
from magicbot import tunable, feedback
from rev import CANSparkMax
from ids import SparkMaxIds, TalonIds, DioChannels

from phoenix6.controls import VelocityVoltage, Follower, NeutralOut
from phoenix6.hardware import TalonFX
from phoenix6.configs import (
    MotorOutputConfigs,
    Slot0Configs,
    FeedbackConfigs,
    ClosedLoopRampsConfigs,
)
from phoenix6.signals import NeutralModeValue
from wpilib import DigitalInput, DutyCycle, SmartDashboard
from wpimath.controller import PIDController

from utilities.functions import clamp


class ShooterComponent:
    FLYWHEEL_GEAR_RATIO = 1 / (22.0 / 18.0)
    FLYWHEEL_TOLERANCE = 1  # rps

    FLYWHEEL_SHOOTING_SPEED = 75
    FLYWHEEL_JETTISON_SPEED = 20
    FLYWHEEL_RAMP_TIME = 1

    MAX_INCLINE_ANGLE = 1.045  # ~60 degrees
    MIN_INCLINE_ANGLE = 0.354  # ~20 degrees
    INCLINATOR_TOLERANCE = math.radians(1)
    INCLINATOR_OFFSET = 2.544 - MIN_INCLINE_ANGLE
    INCLINATOR_SCALE_FACTOR = math.tau  # rps -> radians
    INCLINATOR_GEAR_RATIO = 18 / 24 * 26 / 300
    INCLINATOR_POSITION_CONVERSION_FACTOR = (
        INCLINATOR_GEAR_RATIO * math.tau
    )  # motor rotations -> mech rads
    INCLINATOR_VELOCITY_CONVERSION_FACTOR = (
        INCLINATOR_POSITION_CONVERSION_FACTOR / 60
    )  # rpm -> radians/s
    INCLINATOR_JETTISON_ANGLE = 1.03

    # Add extra point outside our range to ramp speed down to zero
    FLYWHEEL_DISTANCE_LOOKUP = (0, 1.3, 2.0, 3.0, 4.0, 5.0, 7.0)
    FLYWHEEL_SPEED_LOOKUP = (
        54,
        54,
        60,
        64,
        65,
        65,
        0,
    )
    FLYWHEEL_ANGLE_LOOKUP = (
        0.93,
        0.93,
        0.77,
        0.57,
        0.49,
        0.46,
        MIN_INCLINE_ANGLE,
    )

    desired_inclinator_angle = tunable((MAX_INCLINE_ANGLE + MIN_INCLINE_ANGLE) / 2)
    desired_flywheel_speed = tunable(0.0)

    def __init__(self) -> None:
        self.locked = False
        self.inclinator = CANSparkMax(
            SparkMaxIds.shooter_inclinator, CANSparkMax.MotorType.kBrushless
        )
        self.inclinator.setInverted(True)
        self.inclinator.setIdleMode(CANSparkMax.IdleMode.kBrake)
        self.absolute_inclinator_encoder = DutyCycle(
            DigitalInput(DioChannels.inclinator_encoder)
        )
        self.inclinator_encoder = self.inclinator.getEncoder()
        self.inclinator_encoder.setPositionConversionFactor(
            self.INCLINATOR_POSITION_CONVERSION_FACTOR
        )
        self.inclinator_encoder.setVelocityConversionFactor(
            self.INCLINATOR_VELOCITY_CONVERSION_FACTOR
        )
        self.flywheel_left = TalonFX(TalonIds.shooter_flywheel_left)
        self.flywheel_right = TalonFX(TalonIds.shooter_flywheel_right)
        self.flywheel_right.set_control(Follower(TalonIds.shooter_flywheel_left, True))

        flywheel_left_config = self.flywheel_left.configurator
        flywheel_right_config = self.flywheel_right.configurator
        flywheel_motor_config = MotorOutputConfigs()
        flywheel_motor_config.neutral_mode = NeutralModeValue.COAST

        flywheel_pid = (
            Slot0Configs()
            .with_k_p(0.17487)
            .with_k_i(0)
            .with_k_d(0)
            .with_k_s(0.32193)
            .with_k_v(0.096182)
            .with_k_a(0.0096935)
        )

        flywheel_gear_ratio = FeedbackConfigs().with_sensor_to_mechanism_ratio(
            self.FLYWHEEL_GEAR_RATIO
        )

        flywheel_left_closed_loop_ramp_config = (
            ClosedLoopRampsConfigs().with_voltage_closed_loop_ramp_period(
                self.FLYWHEEL_RAMP_TIME
            )
        )

        flywheel_left_config.apply(flywheel_left_closed_loop_ramp_config)
        flywheel_left_config.apply(flywheel_motor_config)
        flywheel_left_config.apply(flywheel_pid)
        flywheel_left_config.apply(flywheel_gear_ratio)

        flywheel_right_config.apply(flywheel_motor_config)
        flywheel_right_config.apply(flywheel_pid)
        flywheel_right_config.apply(flywheel_gear_ratio)

        self.inclinator_controller = PIDController(3.0, 0, 0)
        self.inclinator_controller.setTolerance(ShooterComponent.INCLINATOR_TOLERANCE)
        SmartDashboard.putData(self.inclinator_controller)

    range = tunable(0.0)

    @feedback
    def get_applied_output(self) -> float:
        return self.inclinator.getAppliedOutput()

    def on_enable(self) -> None:
        self.inclinator_controller.reset()

    def lock(self) -> None:
        self.locked = True

    def unlock(self) -> None:
        self.locked = False

    @feedback
    def is_ready(self) -> bool:
        """Is the shooter ready to fire?"""
        return self._flywheels_at_speed() and self._at_inclination()

    @feedback
    def _at_inclination(self) -> bool:
        """Is the inclinator close to the correct angle?"""
        return (
            abs(self.desired_inclinator_angle - self._inclination_angle())
            < self.INCLINATOR_TOLERANCE
        )

    @feedback
    def _flywheels_at_speed(self) -> bool:
        """Are the flywheels close to thier target speed"""
        return (
            abs(self.desired_flywheel_speed - self.flywheel_left.get_velocity().value)
            < self.FLYWHEEL_TOLERANCE
        )

    @feedback
    def _inclination_angle(self) -> float:
        """Get the angle of the mechanism in radians measured positive upwards from zero parellel to the ground."""
        return self._raw_inclination_angle() - self.INCLINATOR_OFFSET

    @feedback
    def _raw_inclination_angle(self) -> float:
        return (
            self.absolute_inclinator_encoder.getOutput() * self.INCLINATOR_SCALE_FACTOR
        )

    def is_range_in_bounds(self, range) -> bool:
        return (
            self.FLYWHEEL_DISTANCE_LOOKUP[0]
            < range
            <= self.FLYWHEEL_DISTANCE_LOOKUP[-2]
        )

    @feedback
    def _flywheel_velocity(self) -> float:
        return self.flywheel_left.get_velocity().value

    def set_range(self, range: float) -> None:
        self.range = range
        self.desired_inclinator_angle = float(
            np.interp(range, self.FLYWHEEL_DISTANCE_LOOKUP, self.FLYWHEEL_ANGLE_LOOKUP)
        )
        self.desired_flywheel_speed = float(
            np.interp(
                range,
                self.FLYWHEEL_DISTANCE_LOOKUP,
                self.FLYWHEEL_SPEED_LOOKUP,
                right=0.0,
            )
        )

    def coast_down(self) -> None:
        self.desired_flywheel_speed = 0

    def prepare_to_jettison(self) -> None:
        self.desired_inclinator_angle = self.INCLINATOR_JETTISON_ANGLE
        self.desired_flywheel_speed = self.FLYWHEEL_JETTISON_SPEED

    def execute(self) -> None:
        """This gets called at the end of the control loop"""
        inclinator_speed = self.inclinator_controller.calculate(
            self._inclination_angle(),
            clamp(
                self.desired_inclinator_angle,
                ShooterComponent.MIN_INCLINE_ANGLE,
                ShooterComponent.MAX_INCLINE_ANGLE,
            ),
        )
        self.inclinator.set(inclinator_speed)

        # stop the flywheels while climbing or permenantly after a real climb
        if self.locked:
            self.desired_flywheel_speed = 0

        if self.desired_flywheel_speed == 0:
            self.flywheel_left.set_control(NeutralOut())
        else:
            self.flywheel_left.set_control(VelocityVoltage(self.desired_flywheel_speed))
