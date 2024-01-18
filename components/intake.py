from enum import Enum

from magicbot import tunable, feedback
from phoenix6.configs import MotorOutputConfigs, config_groups
from phoenix6.controls import VoltageOut
from phoenix6.hardware import TalonFX

from ids import TalonIds


class Intake:
    motor_speed = tunable(1.0)

    class Direction(Enum):
        BACKWARD = -1
        STOPPED = 0
        FORWARD = 1

    def __init__(self) -> None:
        self.motor = TalonFX(TalonIds.intake)
        self.direction = self.Direction.STOPPED

        motor_configurator = self.motor.configurator
        motor_config = MotorOutputConfigs()
        motor_config.inverted = config_groups.InvertedValue.CLOCKWISE_POSITIVE

        motor_configurator.apply(motor_config)

    def deploy(self) -> None:
        pass

    def retract(self) -> None:
        pass

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
