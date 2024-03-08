import math
from enum import Enum
import rev
import time

from magicbot import tunable, feedback
from rev import CANSparkMax
from phoenix6.configs import MotorOutputConfigs, FeedbackConfigs, config_groups
from phoenix6.controls import VoltageOut
from phoenix6.hardware import TalonFX
from wpilib import DigitalInput
from wpimath.controller import ArmFeedforward
from wpimath.trajectory import TrapezoidProfile

from ids import TalonIds, SparkMaxIds, DioChannels


class IntakeComponent:
    motor_speed = tunable(1.0)
    inject_intake_speed = tunable(0.3)
    inject_shoot_speed = tunable(1.0)

    INTAKE_GEAR_RATIO = 2
    DEPLOY_GEAR_RATIO = (1 / 5) * (1 / 3) * (24 / 72)
    MOTOR_REV_TO_SHAFT_RADIANS = DEPLOY_GEAR_RATIO * math.tau
    MOTOR_RPM_TO_SHAFT_RAD_PER_SEC = MOTOR_REV_TO_SHAFT_RADIANS / 60

    SHAFT_REV_RETRACT_HARD_LIMIT = 1.778579
    SHAFT_REV_DEPLOY_HARD_LIMIT = 0.0
    SHAFT_REV_HOVER_POINT = SHAFT_REV_DEPLOY_HARD_LIMIT + math.radians(15)

    ALLOWABLE_ERROR = 0.01

    RETRACTED_STATE = TrapezoidProfile.State(SHAFT_REV_RETRACT_HARD_LIMIT, 0.0)
    DEPLOYED_STATE = TrapezoidProfile.State(SHAFT_REV_DEPLOY_HARD_LIMIT, 0.0)
    HOVER_STATE = TrapezoidProfile.State(SHAFT_REV_HOVER_POINT, 0.0)
    INTAKE_STALL_VELOCITY = 1  # rot/s below which we consider mechanism stalled
    INTAKE_RUNNING_VELOCITY = 3  # rot/s above which stall detection is enabled

    class Direction(Enum):
        BACKWARD = -1
        STOPPED = 0
        FORWARD = 1

    def __init__(self) -> None:
        self.motor = TalonFX(TalonIds.intake)

        self.deploy_motor_l = CANSparkMax(
            SparkMaxIds.intake_deploy_l, CANSparkMax.MotorType.kBrushless
        )
        self.deploy_motor_r = CANSparkMax(
            SparkMaxIds.intake_deploy_r, CANSparkMax.MotorType.kBrushless
        )
        self.deploy_motor_l.setIdleMode(CANSparkMax.IdleMode.kBrake)
        self.deploy_motor_r.setIdleMode(CANSparkMax.IdleMode.kBrake)

        # running the controller on the rio rather than on the motor controller
        # to allow access to the velocity setpoint for feedforward
        arm_constraints = TrapezoidProfile.Constraints(
            maxVelocity=6.0, maxAcceleration=math.pi
        )

        self.arm_profile = TrapezoidProfile(arm_constraints)

        # Recalc with some modifications
        # ratio step up 0.016666667
        # COM 0.3m
        # Arm Mass 4kg
        # start ang 0 deg
        # stop ang 78 deg
        # kG was then lowered after testing
        self.feed_forward_calculator = ArmFeedforward(kS=0.0, kG=0.16, kV=1.17, kA=0.02)

        self.pid_controller = self.deploy_motor_l.getPIDController()
        self.deploy_encoder = self.deploy_motor_l.getEncoder()
        self.deploy_encoder.setVelocityConversionFactor(
            self.MOTOR_RPM_TO_SHAFT_RAD_PER_SEC
        )
        self.deploy_motor_l.setInverted(False)
        self.deploy_encoder.setPositionConversionFactor(self.MOTOR_REV_TO_SHAFT_RADIANS)

        # Retract PID Controller
        self.retract_pid_slot = 0
        self.pid_controller.setFF(
            1 / (5700.0 * self.MOTOR_RPM_TO_SHAFT_RAD_PER_SEC), self.retract_pid_slot
        )
        self.pid_controller.setP(0.2, self.retract_pid_slot)
        self.pid_controller.setI(0, self.retract_pid_slot)
        self.pid_controller.setD(0.4, self.retract_pid_slot)
        self.pid_controller.setOutputRange(-1, 1, self.retract_pid_slot)

        # Deploy PID Controller
        self.deploy_pid_slot = 1
        self.pid_controller.setFF(
            1 / (5700.0 * self.MOTOR_RPM_TO_SHAFT_RAD_PER_SEC), self.deploy_pid_slot
        )
        self.pid_controller.setP(0.6, self.deploy_pid_slot)
        self.pid_controller.setI(0, self.deploy_pid_slot)
        self.pid_controller.setD(0.6, self.deploy_pid_slot)
        self.pid_controller.setOutputRange(-1, 1, self.deploy_pid_slot)

        self.pid_slot = self.retract_pid_slot

        self.direction = self.Direction.STOPPED

        # Intake should begin raised...
        self.target_deployment_state = TrapezoidProfile.State(
            self.SHAFT_REV_RETRACT_HARD_LIMIT, 0.0
        )
        self.last_setpoint_update_time = time.monotonic()

        self.deploy_encoder.setPosition(self.target_deployment_state.position)

        self.deploy_limit_switch = self.deploy_motor_l.getReverseLimitSwitch(
            rev.SparkLimitSwitch.Type.kNormallyOpen
        )
        self.retract_limit_switch = self.deploy_motor_l.getForwardLimitSwitch(
            rev.SparkLimitSwitch.Type.kNormallyOpen
        )

        self.deploy_motor_l.setSoftLimit(
            CANSparkMax.SoftLimitDirection.kForward, self.SHAFT_REV_DEPLOY_HARD_LIMIT
        )
        self.deploy_motor_l.setSoftLimit(
            CANSparkMax.SoftLimitDirection.kReverse, self.SHAFT_REV_RETRACT_HARD_LIMIT
        )

        motor_configurator = self.motor.configurator
        intake_gear_ratio = FeedbackConfigs().with_sensor_to_mechanism_ratio(
            self.INTAKE_GEAR_RATIO
        )
        motor_config = MotorOutputConfigs()
        motor_config.inverted = config_groups.InvertedValue.CLOCKWISE_POSITIVE

        motor_configurator.apply(motor_config)
        motor_configurator.apply(intake_gear_ratio)

        self.deploy_motor_r.follow(self.deploy_motor_l, True)

        self.injector = rev.CANSparkMax(
            SparkMaxIds.shooter_injector, rev.CANSparkMax.MotorType.kBrushless
        )
        self.injector.setInverted(False)

        self.break_beam = DigitalInput(DioChannels.injector_break_beam)

        self.desired_injector_speed = 0.0
        self.has_indexed = False
        self.stall_detection_enabled = False

    @feedback
    def _at_retract_hard_limit(self) -> bool:
        return self.retract_limit_switch.get()

    @feedback
    def _at_deploy_hard_limit(self) -> bool:
        return self.deploy_limit_switch.get()

    def deploy(self) -> None:
        if self.target_deployment_state is not self.DEPLOYED_STATE:
            self.last_setpoint_update_time = time.monotonic()
            self.target_deployment_state = self.DEPLOYED_STATE
            self.pid_slot = self.deploy_pid_slot

    def retract(self) -> None:
        if self.target_deployment_state is not self.RETRACTED_STATE:
            self.last_setpoint_update_time = time.monotonic()
            self.target_deployment_state = self.RETRACTED_STATE
            self.pid_slot = self.retract_pid_slot

    def hover(self) -> None:
        # hover a bit off the ground to facilitate outtaking
        if self.target_deployment_state is not self.HOVER_STATE:
            self.last_setpoint_update_time = time.monotonic()
            self.target_deployment_state = self.HOVER_STATE
            self.pid_slot = self.retract_pid_slot

    def intake(self) -> None:
        self.direction = self.Direction.FORWARD
        self.desired_injector_speed = (
            0.0 if self.has_note() else self.inject_intake_speed
        )

    def backdrive_intake(self) -> None:
        self.direction = self.Direction.BACKWARD

    def backdrive_injector(self) -> None:
        self.desired_injector_speed = -self.inject_intake_speed

    def feed_shooter(self) -> None:
        self.desired_injector_speed = self.inject_shoot_speed

    def has_intake_stalled(self) -> bool:
        return (
            self.motor.get_velocity().value < self.INTAKE_STALL_VELOCITY
            and self.direction is not self.Direction.STOPPED
            and self.stall_detection_enabled
        )

    @feedback
    def is_fully_retracted(self) -> bool:
        return self._at_retract_hard_limit() or (
            abs(self.SHAFT_REV_RETRACT_HARD_LIMIT - self.deploy_encoder.getPosition())
            < self.ALLOWABLE_ERROR
        )

    @feedback
    def is_fully_deployed(self) -> bool:
        return self._at_deploy_hard_limit() or (
            abs(self.SHAFT_REV_DEPLOY_HARD_LIMIT - self.deploy_encoder.getPosition())
            < self.ALLOWABLE_ERROR
        )

    @feedback
    def deploy_current_position(self) -> float:
        return self.deploy_encoder.getPosition()

    def maybe_reindex_deployment_encoder(self) -> None:
        if self._at_retract_hard_limit():
            self.deploy_encoder.setPosition(self.SHAFT_REV_RETRACT_HARD_LIMIT)
            self.has_indexed = True

        if self._at_deploy_hard_limit():
            self.deploy_encoder.setPosition(self.SHAFT_REV_DEPLOY_HARD_LIMIT)
            self.has_indexed = True

    @feedback
    def has_note(self) -> bool:
        return not self.break_beam.get()

    def execute(self) -> None:
        if not self.has_indexed:
            self.maybe_reindex_deployment_encoder()

        # stall detection gating
        if self.direction is self.Direction.STOPPED:
            self.stall_detection_enabled = False
        elif self.motor.get_velocity().value > self.INTAKE_RUNNING_VELOCITY:
            self.stall_detection_enabled = True

        intake_request = VoltageOut(self.direction.value * self.motor_speed * 12.0)

        self.motor.set_control(intake_request)

        desired_state = self.arm_profile.calculate(
            time.monotonic() - self.last_setpoint_update_time,
            TrapezoidProfile.State(
                self.deploy_encoder.getPosition(), self.deploy_encoder.getVelocity()
            ),
            self.target_deployment_state,
        )

        ff = self.feed_forward_calculator.calculate(
            desired_state.position, desired_state.velocity
        )
        if (
            self.target_deployment_state is self.DEPLOYED_STATE
            or self.target_deployment_state is self.HOVER_STATE
        ):
            self.pid_controller.setReference(
                desired_state.position,
                CANSparkMax.ControlType.kPosition,
                pidSlot=self.pid_slot,
            )
        elif self.target_deployment_state is self.RETRACTED_STATE:
            self.pid_controller.setReference(
                desired_state.position,
                CANSparkMax.ControlType.kPosition,
                pidSlot=self.pid_slot,
                arbFeedforward=ff,
            )

        self.injector.set(self.desired_injector_speed)

        self.direction = self.Direction.STOPPED
        self.desired_injector_speed = 0.0
