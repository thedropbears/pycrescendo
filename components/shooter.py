from magicbot import tunable
from rev import CANSparkMax
from ids import SparkMaxIds, TalonIds
import phoenix6
from phoenix6.configs import (
    config_groups,
    MotorOutputConfigs,
)
from phoenix6.controls import VoltageOut
import phoenix6.hardware


class Shooter:
    flywheel_speed = tunable(0.0)
    inject_speed = tunable(0.0)

    def __init__(self):
        self.deployed = False
        self.top_flywheel = phoenix6.hardware.TalonFX(TalonIds.top_flywheel)
        self.bottom_flywheel = phoenix6.hardware.TalonFX(TalonIds.bottom_flywheel)
        self.inject_flywheel = CANSparkMax(
            SparkMaxIds.inject_flywheel, CANSparkMax.MotorType.kBrushless
        )

        # Configure bottom flywheel motor
        bottom_config = self.bottom_flywheel.configurator
        bottom_flywheel_config = MotorOutputConfigs()
        bottom_flywheel_config.inverted = config_groups.InvertedValue.CLOCKWISE_POSITIVE

        bottom_config.apply(bottom_flywheel_config)

        self.inject_flywheel.setInverted(True)

        self.should_inject = False

    def shoot(self):
        """causes shooter motors to spin"""
        self.deployed = True

    def inject(self):
        self.should_inject = True

    def is_ready(self):
        return True

    def execute(self):
        """This gets called at the end of the control loop"""
        voltsge_request = VoltageOut(12.0 * self.flywheel_speed)
        if self.should_inject:
            self.inject_flywheel.set(self.inject_speed)
        else:
            self.inject_flywheel.set(0.0)

        self.top_flywheel.set_control(voltsge_request)
        self.bottom_flywheel.set_control(voltsge_request)
        self.should_inject = False
