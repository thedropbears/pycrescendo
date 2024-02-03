from enum import Enum

from magicbot import tunable, feedback
from phoenix6.configs import MotorOutputConfigs, config_groups
from phoenix6.controls import VoltageOut
from phoenix6.hardware import TalonFX

from rev import CANSparkMax

from ids import TalonIds, SparkMaxIds


class IntakeComponent:
    motor_speed = tunable(0.4)

    class Direction(Enum):
        BACKWARD = -1
        STOPPED = 0
        FORWARD = 1

    def __init__(self) -> None:
        self.motor = TalonFX(TalonIds.intake)
        self.deploy_motor = CANSparkMax(SparkMaxIds.intake_deploy)
        # pid = self.deploy_motor.getPIDController()
        # pid.setP(0.1)
        # pid.setI(0.0)
        # pid.setD(0.0)
        self.direction = self.Direction.STOPPED

        motor_configurator = self.motor.configurator
        motor_config = MotorOutputConfigs()
        motor_config.inverted = config_groups.InvertedValue.COUNTER_CLOCKWISE_POSITIVE

        motor_configurator.apply(motor_config)

    def deploy(self) -> None:
        self.deploy_motor.set(self.Direction.FORWARD)

    def retract(self) -> None:
        self.deploy_motor.set(self.Direction.FORWARD)

    def intake(self) -> None:
        self.direction = self.Direction.FORWARD

    def outtake(self) -> None:
        self.direction = self.Direction.BACKWARD

    @feedback
    def is_note_present(self) -> bool:
        return False

    @feedback
    def is_fully_retracted(self) -> bool:
        return True

    @feedback
    def is_fully_deployed(self) -> bool:
        return True

    def execute(self) -> None:
        intake_request = VoltageOut(self.direction.value * self.motor_speed * 12.0)

        self.motor.set_control(intake_request)
        self.direction = self.Direction.STOPPED
