import math
from enum import Enum
import rev

from magicbot import tunable, feedback
from rev import CANSparkMax
from phoenix6.configs import MotorOutputConfigs, config_groups
from phoenix6.controls import VoltageOut
from phoenix6.hardware import TalonFX

from ids import TalonIds, SparkMaxIds


class IntakeComponent:
    motor_speed = tunable(1.0)

    GEAR_RATIO = (1 / 5) * (1 / 4) * (24 / 76)
    MOTOR_REV_TO_SHAFT_RADIANS = GEAR_RATIO * math.tau
    MOTOR_RPM_TO_SHAFT_RAD_PER_SEC = MOTOR_REV_TO_SHAFT_RADIANS / 60

    SHAFT_REV_RETRACT_HARD_LIMIT = 0.0
    SHAFT_REV_DEPLOY_HARD_LIMIT = 1.8675022996

    ALLOWABLE_ERROR = 0.01

    # TODO REMOVE THIS WHEN WE HAVE REMADE THE MECHANISM AND ARENT AS AFRAID OF BREAKING IT
    SAFETY_SCALE = 0.4

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

        self.pid_controller = self.deploy_motor_l.getPIDController()
        self.deploy_encoder = self.deploy_motor_l.getEncoder()
        self.deploy_encoder.setVelocityConversionFactor(
            self.MOTOR_RPM_TO_SHAFT_RAD_PER_SEC
        )
        self.deploy_encoder.setPositionConversionFactor(self.MOTOR_REV_TO_SHAFT_RADIANS)

        # Retract PID Controller
        self.retract_pid_slot = 0
        self.pid_controller.setFF(
            1 / (5700.0 * self.MOTOR_RPM_TO_SHAFT_RAD_PER_SEC), self.retract_pid_slot
        )
        self.pid_controller.setP(0.08, self.retract_pid_slot)
        self.pid_controller.setI(0, self.retract_pid_slot)
        self.pid_controller.setD(0.4, self.retract_pid_slot)
        self.pid_controller.setOutputRange(-1, 1, self.retract_pid_slot)

        self.pid_controller.setSmartMotionMaxVelocity(
            self.SAFETY_SCALE * 6, self.retract_pid_slot
        )  # rad/s
        self.pid_controller.setSmartMotionMinOutputVelocity(
            0, self.retract_pid_slot
        )  # rad/s
        self.pid_controller.setSmartMotionMaxAccel(
            self.SAFETY_SCALE * math.pi, self.retract_pid_slot
        )  # rad/s^2
        self.pid_controller.setSmartMotionAllowedClosedLoopError(
            self.ALLOWABLE_ERROR, self.retract_pid_slot
        )  # Max allowed error

        # Deploy PID Controller
        self.deploy_pid_slot = 1
        self.pid_controller.setFF(
            1 / (5700.0 * self.MOTOR_RPM_TO_SHAFT_RAD_PER_SEC), self.deploy_pid_slot
        )
        self.pid_controller.setP(0.06, self.deploy_pid_slot)
        self.pid_controller.setI(0, self.deploy_pid_slot)
        self.pid_controller.setD(0.4, self.deploy_pid_slot)
        self.pid_controller.setOutputRange(-1, 1, self.deploy_pid_slot)

        self.pid_controller.setSmartMotionMaxVelocity(
            self.SAFETY_SCALE * 6, self.deploy_pid_slot
        )  # rad/s
        self.pid_controller.setSmartMotionMinOutputVelocity(
            0, self.deploy_pid_slot
        )  # rad/s
        self.pid_controller.setSmartMotionMaxAccel(
            self.SAFETY_SCALE * math.pi, self.deploy_pid_slot
        )  # rad/s^2
        self.pid_controller.setSmartMotionAllowedClosedLoopError(
            self.ALLOWABLE_ERROR, self.deploy_pid_slot
        )  # Max allowed error

        self.pid_slot = self.retract_pid_slot

        self.direction = self.Direction.STOPPED

        # Intake should begin raised...
        self.deploy_setpoint = self.SHAFT_REV_RETRACT_HARD_LIMIT
        self.deploy_encoder.setPosition(self.deploy_setpoint)

        self.deploy_limit_switch = self.deploy_motor_l.getForwardLimitSwitch(
            rev.SparkLimitSwitch.Type.kNormallyOpen
        )
        self.retract_limit_switch = self.deploy_motor_l.getReverseLimitSwitch(
            rev.SparkLimitSwitch.Type.kNormallyOpen
        )

        self.deploy_motor_l.setSoftLimit(
            CANSparkMax.SoftLimitDirection.kForward, self.SHAFT_REV_DEPLOY_HARD_LIMIT
        )
        self.deploy_motor_l.setSoftLimit(
            CANSparkMax.SoftLimitDirection.kReverse, self.SHAFT_REV_RETRACT_HARD_LIMIT
        )

        motor_configurator = self.motor.configurator
        motor_config = MotorOutputConfigs()
        motor_config.inverted = config_groups.InvertedValue.CLOCKWISE_POSITIVE

        motor_configurator.apply(motor_config)

        self.deploy_motor_r.follow(self.deploy_motor_l, True)

    def _at_retract_hard_limit(self) -> bool:
        return self.retract_limit_switch.get()

    def _at_deploy_hard_limit(self) -> bool:
        return self.deploy_limit_switch.get()

    def deploy(self) -> None:
        self.deploy_setpoint = self.SHAFT_REV_DEPLOY_HARD_LIMIT
        self.pid_slot = self.deploy_pid_slot

    def retract(self) -> None:
        self.deploy_setpoint = self.SHAFT_REV_RETRACT_HARD_LIMIT
        self.pid_slot = self.retract_pid_slot

    def intake(self) -> None:
        self.direction = self.Direction.FORWARD

    def outtake(self) -> None:
        self.direction = self.Direction.BACKWARD

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

        if self._at_deploy_hard_limit():
            self.deploy_encoder.setPosition(self.SHAFT_REV_DEPLOY_HARD_LIMIT)

    def execute(self) -> None:
        self.maybe_reindex_deployment_encoder()

        intake_request = VoltageOut(self.direction.value * self.motor_speed * 12.0)

        self.motor.set_control(intake_request)

        self.pid_controller.setReference(
            self.deploy_setpoint,
            CANSparkMax.ControlType.kSmartMotion,
            pidSlot=self.pid_slot,
        )

        self.direction = self.Direction.STOPPED
