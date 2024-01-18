from magicbot import tunable
from rev import CANSparkMax
from ids import SparkMaxIds, TalonIds
import phoenix6
from phoenix6.controls import VoltageOut
import phoenix6.hardware


class ShooterComponent:
    flywheel_speed = tunable(0.0)
    inject_speed = tunable(0.0)

    def __init__(self):
        self.flywheel = phoenix6.hardware.TalonFX(TalonIds.shooter_flywheel)
        self.injector = CANSparkMax(
            SparkMaxIds.shooter_injector, CANSparkMax.MotorType.kBrushless
        )
        self.injector.setInverted(True)

        self.should_inject = False

    def shoot(self):
        self.should_inject = True

    def is_ready(self):
        return True

    def execute(self):
        """This gets called at the end of the control loop"""
        flywheel_request = VoltageOut(12.0 * self.flywheel_speed)
        if self.should_inject:
            self.injector.set(self.inject_speed)
        else:
            self.injector.set(0.0)

        self.flywheel.set_control(flywheel_request)
        self.should_inject = False
