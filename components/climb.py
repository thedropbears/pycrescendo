from magicbot import tunable
import wpilib


class Climber:
    shooter_motor: wpilib.Talon
    shoot_speed = tunable(1.0)  # speed is tunable via NetworkTables

    def __init__(self):
        self.deployed = False

    def deploy(self):
        """Causes the climber to do it's thing"""
        self.deployed = True

    def is_ready(self):
        return True

    def execute(self):
        """This gets called at the end of the control loop"""
        if self.deployed:
            self.shooter_motor.set(self.shoot_speed)
        else:
            self.shooter_motor.set(0)

        self.deployed = False
