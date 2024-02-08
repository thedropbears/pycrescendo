from enum import Enum

from magicbot import tunable, feedback
from phoenix6.configs import MotorOutputConfigs, config_groups
from phoenix6.controls import VoltageOut
from phoenix6.hardware import TalonFX

from wpilib import DigitalInput

from math import tau

from rev import CANSparkMax

from ids import TalonIds, SparkMaxIds, DioChannels


class IntakeComponent:
    motor_speed = tunable(0.4)

    GEAR_RATIO = 24 / 48  # 24 / 54 / 48
    MOTOR_RPM_TO_SHAFT_RAD_PER_SEC = tau / 60 * GEAR_RATIO

    SHAFT_REV_TOP_LIMIT = 0
    SHAFT_REV_BOTTOM_LIMIT = 2.004

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
        self.deploy_encoder.setPositionConversionFactor(
            self.MOTOR_RPM_TO_SHAFT_RAD_PER_SEC
        )

        self.pid_controller.setP(0.1)
        self.pid_controller.setI(0)
        self.pid_controller.setD(0)
        self.pid_controller.setOutputRange(-1, 1)

        slot = 0
        self.pid_controller.setSmartMotionMaxVelocity(1, slot)  # rad/s
        self.pid_controller.setSmartMotionMinOutputVelocity(0, slot)  # rad/s
        self.pid_controller.setSmartMotionMaxAccel(0.5, slot)  # rad/s^2
        self.pid_controller.setSmartMotionAllowedClosedLoopError(
            0.1, slot
        )  # Max allowed error

        self.direction = self.Direction.STOPPED

        self.deploy_limit_switch = DigitalInput(DioChannels.intake_deploy_switch)
        self.retract_limit_switch = DigitalInput(DioChannels.intake_retract_switch)

        self.deploy_motor.setSoftLimit(
            CANSparkMax.SoftLimitDirection.kForward, self.SHAFT_REV_TOP_LIMIT
        )
        self.deploy_motor.setSoftLimit(
            CANSparkMax.SoftLimitDirection.kReverse, self.SHAFT_REV_BOTTOM_LIMIT
        )
        # Intake should begin raised...
        self.deploy_setpoint = self.SHAFT_REV_TOP_LIMIT
        self.deploy_encoder.setPosition(self.deploy_setpoint)
        self.set_soft_limit_state(False)

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

    def deploy(self) -> None:
        self.deploy_setpoint = self.SHAFT_REV_BOTTOM_LIMIT

    def retract(self) -> None:
        self.deploy_setpoint = self.SHAFT_REV_TOP_LIMIT

    def intake(self) -> None:
        self.direction = self.Direction.FORWARD

    def outtake(self) -> None:
        self.direction = self.Direction.BACKWARD

    @feedback
    def is_note_present(self) -> bool:
        return False

    @feedback
    def is_fully_retracted(self) -> bool:
        return not self.retract_limit_switch.get() or (
            self.encoder_limit_enabled
            and self.deploy_motor.getFault(CANSparkMax.FaultID.kSoftLimitRev)
        )

    @feedback
    def is_fully_deployed(self) -> bool:
        return not self.deploy_limit_switch.get() or (
            self.encoder_limit_enabled
            and self.deploy_motor.getFault(CANSparkMax.FaultID.kSoftLimitFwd)
        )

    def execute(self) -> None:
        if not self.encoder_limit_enabled:
            if self.is_fully_retracted():
                self.set_soft_limit_state(True)
                self.deploy_encoder.setPosition(self.SHAFT_REV_BOTTOM_LIMIT)

            if self.is_fully_deployed():
                self.set_soft_limit_state(True)
                self.deploy_encoder.setPosition(self.SHAFT_REV_TOP_LIMIT)

        intake_request = VoltageOut(self.direction.value * self.motor_speed * 12.0)

        self.motor.set_control(intake_request)

        self.pid_controller.setReference(
            self.deploy_setpoint, CANSparkMax.ControlType.kSmartMotion
        )
        self.direction = self.Direction.STOPPED
