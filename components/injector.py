import enum
import magicbot
import rev
import wpilib

import ids


class Mode(enum.Enum):
    IDLE = enum.auto()
    INTAKE = enum.auto()
    SHOOT = enum.auto()


class InjectorComponent:
    intake_speed = magicbot.tunable(0.5)
    shoot_speed = magicbot.tunable(1.0)

    def __init__(self) -> None:
        self.injector = rev.CANSparkMax(
            ids.SparkMaxIds.shooter_injector, rev.CANSparkMax.MotorType.kBrushless
        )
        self.injector.setInverted(False)

        self.break_beam = wpilib.DigitalInput(ids.DioChannels.injector_break_beam)

        self._mode = Mode.IDLE

    def has_note(self) -> bool:
        return not self.break_beam.get()

    def intake(self) -> None:
        self._mode = Mode.INTAKE

    def shoot(self) -> None:
        self._mode = Mode.SHOOT

    def execute(self) -> None:
        match self._mode:
            case Mode.IDLE:
                speed = 0.0
            case Mode.INTAKE:
                speed = 0.0 if self.has_note() else self.intake_speed
            case Mode.SHOOT:
                speed = self.shoot_speed

        self.injector.set(speed)
