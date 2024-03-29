import wpilib
from enum import Enum
from magicbot import feedback
from rev import CANSparkMax
from ids import SparkMaxIds, DioChannels

from components.led import LightStrip


class Climber:
    GEAR_RATIO = 1 / 48  # using a Neo with 4:1 4:1 3:1 ratio
    SHAFT_REV_TOP_LIMIT = 7.110515
    SHAFT_REV_BOTTOM_LIMIT = 0

    status_lights: LightStrip

    class POSITION(Enum):
        RETRACTED = 0
        DEPLOYING = 1
        DEPLOYED = 2

    def __init__(self) -> None:
        self.climbing_motor = CANSparkMax(
            SparkMaxIds.climber, CANSparkMax.MotorType.kBrushless
        )
        self.deploy_limit_switch = wpilib.DigitalInput(
            DioChannels.climber_deploy_switch
        )
        self.retract_limit_switch = wpilib.DigitalInput(
            DioChannels.climber_retract_switch
        )
        self.speed = 0.0
        self.climb_encoder = self.climbing_motor.getEncoder()
        self.climb_encoder.setPositionConversionFactor(self.GEAR_RATIO)

        self.last_position = self.POSITION.RETRACTED

        self.seen_deploy_limit_switch = False

    def on_disable(self) -> None:
        self.seen_deploy_limit_switch = False

    @feedback
    def has_climb_finished(self) -> bool:
        return not self.retract_limit_switch.get()

    @feedback
    def has_deploy_finished(self) -> bool:
        return not self.deploy_limit_switch.get()

    def should_lock_mechanisms(self) -> bool:
        # Climbs in the last 20 seconds are real climbs...
        return self.seen_deploy_limit_switch

    def deploy(self) -> None:
        if self.has_deploy_finished():
            self.speed = 0.0
        else:
            self.speed = 1.0

    def retract(self) -> None:
        if self.has_climb_finished():
            self.speed = 0.0
        else:
            self.speed = -1.0

    def execute(self) -> None:
        if self.has_climb_finished():
            self.climb_encoder.setPosition(self.SHAFT_REV_BOTTOM_LIMIT)

        if self.has_deploy_finished():
            self.climb_encoder.setPosition(self.SHAFT_REV_TOP_LIMIT)

        if self.has_climb_finished():
            if self.last_position is not self.POSITION.RETRACTED:
                self.status_lights.climbing_arm_retracted()
                self.last_position = self.POSITION.RETRACTED
            if wpilib.DriverStation.getMatchTime() > 20:
                # reset in case of accidental climb
                self.seen_deploy_limit_switch = False
        elif self.has_deploy_finished():
            self.seen_deploy_limit_switch = True
            if self.last_position is not self.POSITION.DEPLOYED:
                self.status_lights.climbing_arm_fully_extended()
                self.last_position = self.POSITION.DEPLOYED
        elif self.last_position is not self.POSITION.DEPLOYING:
            self.status_lights.climbing_arm_extending()
            self.last_position = self.POSITION.DEPLOYING

        self.climbing_motor.set(self.speed)
        self.speed = 0.0
