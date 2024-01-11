from magicbot import tunable
import wpilib


class Intake:
    shooter_motor: wpilib.Talon
    shoot_speed = tunable(1.0)  # speed is tunable via NetworkTables

    def __init__(self):
        self.deployed = False

    def deploy(self):
        self.deployed = True

    def is_ready(self):
        return True

    def execute(self):
        if self.deployed:
            self.shooter_motor.set(self.shoot_speed)
        else:
            self.shooter_motor.set(0)

        self.deployed = False
