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

    LIMIT_BOTTOM = 0
    LIMIT_TOP = 10  # TODO: replace with correct number

    class Direction(Enum):
        BACKWARD = -1
        STOPPED = 0
        FORWARD = 1

    def __init__(self) -> None:
        self.motor = TalonFX(TalonIds.intake)

        self.deploy_motor = CANSparkMax(
            SparkMaxIds.intake_deploy, CANSparkMax.MotorType.kBrushless
        )
        self.pid_controller = self.deploy_motor.getPIDController()
        self.encoder = self.deploy_motor.getEncoder()
        RPMTORotsPerSec = tau * 60
        self.encoder.setVelocityConversionFactor(RPMTORotsPerSec)
        self.encoder.setPositionConversionFactor(RPMTORotsPerSec)
        # pv = self.encoder.getPosition()

        self.pid_controller.setP(0.1)
        self.pid_controller.setI(0)
        self.pid_controller.setD(0)
        self.pid_controller.setOutputRange(-1, 1)

        slot = 0
        self.pid_controller.setSmartMotionMaxVelocity(1, slot)  # RPM
        self.pid_controller.setSmartMotionMinOutputVelocity(0, slot)  # RPM
        self.pid_controller.setSmartMotionMaxAccel(0.5, slot)  # RPM^2
        self.pid_controller.setSmartMotionAllowedClosedLoopError(
            0, slot
        )  # Max allowed error

        self.direction = self.Direction.STOPPED
        self.deploy_dir = self.Direction.STOPPED

        self.deploy_limit_switch = DigitalInput(DioChannels.intake_deploy_switch)
        self.retract_limit_switch = DigitalInput(DioChannels.intake_retract_switch)

        self.deploy_motor.setSoftLimit(
            CANSparkMax.SoftLimitDirection.kForward, self.SHAFT_REV_TOP_LIMIT
        )
        self.deploy_motor.setSoftLimit(
            CANSparkMax.SoftLimitDirection.kReverse, self.SHAFT_REV_BOTTOM_LIMIT
        )
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
        self.deploy_dir = self.Direction.FORWARD

    def retract(self) -> None:
        self.deploy_dir = self.Direction.BACKWARD

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
                self.enable_soft_limits(True)
                self.climb_encoder.setPosition(self.LIMIT_BOTTOM)

            if self.is_fully_deployed():
                self.enable_soft_limits(True)
                self.climb_encoder.setPosition(self.LIMIT_TOP)

        intake_request = VoltageOut(self.direction.value * self.motor_speed * 12.0)

        self.motor.set_control(intake_request)

        self.pid_controller.setReference(
            self.deploy_dir, CANSparkMax.ControlType.kSmartMotion
        )
        self.direction = self.Direction.STOPPED
