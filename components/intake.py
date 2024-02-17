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

    GEAR_RATIO = 1 / 50
    MOTOR_REV_TO_SHAFT_RADIANS = GEAR_RATIO * math.tau
    MOTOR_RPM_TO_SHAFT_RAD_PER_SEC = MOTOR_REV_TO_SHAFT_RADIANS / 60

    SHAFT_REV_TOP_LIMIT = 0.0
    SHAFT_REV_BOTTOM_LIMIT = 2.115336

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

        self.deploying = False
        self.deploy_setpoint = 0.0

        # Intake should begin raised...
        self.retract()
        self.deploy_encoder.setPosition(self.deploy_setpoint)
        self.set_soft_limit_state(False)

        self.deploy_limit_switch = self.deploy_motor.getForwardLimitSwitch(
            rev.SparkLimitSwitch.Type.kNormallyOpen
        )
        self.retract_limit_switch = self.deploy_motor.getReverseLimitSwitch(
            rev.SparkLimitSwitch.Type.kNormallyOpen
        )

        self.deploy_motor.setSoftLimit(
            CANSparkMax.SoftLimitDirection.kForward, self.SHAFT_REV_BOTTOM_LIMIT
        )
        self.deploy_motor.setSoftLimit(
            CANSparkMax.SoftLimitDirection.kReverse, self.SHAFT_REV_TOP_LIMIT
        )

        motor_configurator = self.motor.configurator
        motor_config = MotorOutputConfigs()
        motor_config.inverted = config_groups.InvertedValue.COUNTER_CLOCKWISE_POSITIVE

        motor_configurator.apply(motor_config)

    def set_soft_limit_state(self, state: bool) -> None:
        self.encoder_limit_enabled = state
        self.deploy_motor.enableSoftLimit(
            CANSparkMax.SoftLimitDirection.kForward, state
        )
        self.deploy_motor.enableSoftLimit(
            CANSparkMax.SoftLimitDirection.kReverse, state
        )

    def retract_hard_limit(self) -> bool:
        return self.retract_limit_switch.get()

    def deploy_hard_limit(self) -> bool:
        return self.deploy_limit_switch.get()

    def deploy(self) -> None:
        self.deploying = True
        self.deploy_setpoint = self.SHAFT_REV_BOTTOM_LIMIT

    def retract(self) -> None:
        self.deploying = True
        self.deploy_setpoint = self.SHAFT_REV_TOP_LIMIT

    def stop_deploy(self) -> None:
        self.deploying = False

    def intake(self) -> None:
        self.direction = self.Direction.FORWARD

    def outtake(self) -> None:
        self.direction = self.Direction.BACKWARD

    @feedback
    def is_note_present(self) -> bool:
        return False

    @feedback
    def is_fully_retracted(self) -> bool:
        return self.retract_hard_limit() or (
            self.encoder_limit_enabled
            and abs(self.SHAFT_REV_TOP_LIMIT - self.deploy_encoder.getPosition())
            < self.ALLOWABLE_ERROR
        )

    @feedback
    def is_fully_deployed(self) -> bool:
        return self.deploy_hard_limit() or (
            self.encoder_limit_enabled
            and abs(self.SHAFT_REV_BOTTOM_LIMIT - self.deploy_encoder.getPosition())
            < self.ALLOWABLE_ERROR
        )

    @feedback
    def deploy_current_position(self) -> float:
        return self.deploy_encoder.getPosition()

    def try_initialise_limits(self) -> None:
        if not self.encoder_limit_enabled:
            if self.is_fully_retracted():
                self.set_soft_limit_state(True)
                self.deploy_encoder.setPosition(self.SHAFT_REV_TOP_LIMIT)

            if self.is_fully_deployed():
                self.set_soft_limit_state(True)
                self.deploy_encoder.setPosition(self.SHAFT_REV_BOTTOM_LIMIT)

    def execute(self) -> None:
        self.try_initialise_limits()

        intake_request = VoltageOut(self.direction.value * self.motor_speed * 12.0)

        self.motor.set_control(intake_request)

        if self.deploying:
            self.pid_controller.setReference(
                self.deploy_setpoint, CANSparkMax.ControlType.kSmartMotion
            )
        else:
            self.deploy_motor.set(0)
        self.direction = self.Direction.STOPPED
