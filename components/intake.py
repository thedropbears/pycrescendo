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
    motor_speed = tunable(0.4)

    GEAR_RATIO = (1 / 5) * (1 / 5) * (18 / 72)
    MOTOR_REV_TO_SHAFT_RADIANS = GEAR_RATIO * math.tau
    MOTOR_RPM_TO_SHAFT_RAD_PER_SEC = MOTOR_REV_TO_SHAFT_RADIANS / 60

    SHAFT_REV_RETRACT_HARD_LIMIT = 0.0
    SHAFT_REV_DEPLOY_HARD_LIMIT = 1.8675022996

    SHAFT_REV_RETRACT_SOFT_LIMIT = SHAFT_REV_RETRACT_HARD_LIMIT + math.radians(5)
    SHAFT_REV_DEPLOY_SOFT_LIMIT = SHAFT_REV_DEPLOY_HARD_LIMIT - math.radians(5)

    ALLOWABLE_ERROR = 0.01

    class Direction(Enum):
        BACKWARD = -1
        STOPPED = 0
        FORWARD = 1

    def __init__(self) -> None:
        self.motor = TalonFX(TalonIds.intake)

        # TODO check motor direction
        self.deploy_motor = CANSparkMax(
            SparkMaxIds.intake_deploy, CANSparkMax.MotorType.kBrushless
        )
        self.pid_controller = self.deploy_motor.getPIDController()
        self.deploy_encoder = self.deploy_motor.getEncoder()
        self.deploy_encoder.setVelocityConversionFactor(
            self.MOTOR_RPM_TO_SHAFT_RAD_PER_SEC
        )
        self.deploy_encoder.setPositionConversionFactor(self.MOTOR_REV_TO_SHAFT_RADIANS)

        self.pid_controller.setP(0.05)
        self.pid_controller.setI(0)
        self.pid_controller.setD(0)
        self.pid_controller.setOutputRange(-1, 1)

        slot = 0
        self.pid_controller.setSmartMotionMaxVelocity(6, slot)  # rad/s
        self.pid_controller.setSmartMotionMinOutputVelocity(0, slot)  # rad/s
        self.pid_controller.setSmartMotionMaxAccel(6, slot)  # rad/s^2
        self.pid_controller.setSmartMotionAllowedClosedLoopError(
            self.ALLOWABLE_ERROR, slot
        )  # Max allowed error

        self.direction = self.Direction.STOPPED

        # Intake should begin raised...
        self.deploy_setpoint = self.SHAFT_REV_RETRACT_SOFT_LIMIT
        self.deploy_encoder.setPosition(self.deploy_setpoint)

        self.deploy_limit_switch = self.deploy_motor.getForwardLimitSwitch(
            rev.SparkLimitSwitch.Type.kNormallyOpen
        )
        self.retract_limit_switch = self.deploy_motor.getReverseLimitSwitch(
            rev.SparkLimitSwitch.Type.kNormallyOpen
        )

        self.deploy_motor.setSoftLimit(
            CANSparkMax.SoftLimitDirection.kForward, self.SHAFT_REV_DEPLOY_SOFT_LIMIT
        )
        self.deploy_motor.setSoftLimit(
            CANSparkMax.SoftLimitDirection.kReverse, self.SHAFT_REV_RETRACT_SOFT_LIMIT
        )

        motor_configurator = self.motor.configurator
        motor_config = MotorOutputConfigs()
        motor_config.inverted = config_groups.InvertedValue.COUNTER_CLOCKWISE_POSITIVE

        motor_configurator.apply(motor_config)

    def _at_retract_hard_limit(self) -> bool:
        return self.retract_limit_switch.get()

    def _at_deploy_hard_limit(self) -> bool:
        return self.deploy_limit_switch.get()

    def deploy(self) -> None:
        self.deploy_setpoint = self.SHAFT_REV_DEPLOY_SOFT_LIMIT

    def retract(self) -> None:
        self.deploy_setpoint = self.SHAFT_REV_RETRACT_SOFT_LIMIT

    def intake(self) -> None:
        self.direction = self.Direction.FORWARD

    def outtake(self) -> None:
        self.direction = self.Direction.BACKWARD

    @feedback
    def is_note_present(self) -> bool:
        return False

    @feedback
    def is_fully_retracted(self) -> bool:
        return self._at_retract_hard_limit() or (
            abs(self.SHAFT_REV_RETRACT_SOFT_LIMIT - self.deploy_encoder.getPosition())
            < self.ALLOWABLE_ERROR
        )

    @feedback
    def is_fully_deployed(self) -> bool:
        return self._at_deploy_hard_limit() or (
            abs(self.SHAFT_REV_DEPLOY_SOFT_LIMIT - self.deploy_encoder.getPosition())
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
            self.deploy_setpoint, CANSparkMax.ControlType.kSmartMotion
        )

        self.direction = self.Direction.STOPPED
