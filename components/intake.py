from enum import Enum

from magicbot import tunable, feedback
from phoenix6.configs import MotorOutputConfigs, config_groups
from phoenix6.controls import VoltageOut
from phoenix6.hardware import TalonFX

from ids import TalonIds


class IntakeComponent:
    intake_motor_speed = tunable(0.4)

    class Direction(Enum):
        BACKWARD = -1
        STOPPED = 0
        FORWARD = 1

    def __init__(self) -> None:
        self.intake_motor = TalonFX(TalonIds.intake)
        self.direction = self.Direction.STOPPED

        intake_motor_configurator = self.intake_motor.configurator
        intake_motor_config = MotorOutputConfigs()
        intake_motor_config.inverted = (
            config_groups.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )

        intake_motor_configurator.apply(intake_motor_config)

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
        intake_request = VoltageOut(
            self.direction.value * self.intake_motor_speed * 12.0
        )

        self.intake_motor.set_control(intake_request)
        self.direction = self.Direction.STOPPED
