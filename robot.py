#!/usr/bin/env python3

import wpilib
import wpilib.event
import magicbot

from components.chassis import Chassis
from controllers.movement import Movement

from utilities.scalers import rescale_js


class MyRobot(magicbot.MagicRobot):
    # Controllers
    movement: Movement

    # Components
    chassis: Chassis

    max_speed = magicbot.tunable(Chassis.max_wheel_speed * 0.95)

    def createObjects(self) -> None:
        self.data_log = wpilib.DataLogManager.getLog()

        self.gamepad = wpilib.XboxController(0)

        self.rumble_timer = wpilib.Timer()
        self.rumble_timer.start()
        self.rumble_duration = 0.0

        self.field = wpilib.Field2d()
        wpilib.SmartDashboard.putData(self.field)
        self.target_node = self.field.getObject("target_node")

        self.last_dpad = -1

    def rumble_for(self, intensity: float, duration: float):
        self.rumble_duration = duration
        self.rumble_timer.reset()
        self.gamepad.setRumble(wpilib.XboxController.RumbleType.kBothRumble, intensity)

    def short_rumble(self):
        self.rumble_for(0.4, 0.1)

    def long_rumble(self):
        self.rumble_for(0.8, 0.3)

    def teleopInit(self) -> None:
        pass

    def teleopPeriodic(self) -> None:
        # Driving
        spin_rate = 4
        drive_x = -rescale_js(self.gamepad.getLeftY(), 0.1) * self.max_speed
        drive_y = -rescale_js(self.gamepad.getLeftX(), 0.1) * self.max_speed
        drive_z = -rescale_js(self.gamepad.getRightX(), 0.1, exponential=2) * spin_rate
        local_driving = self.gamepad.getBButton()
        self.movement.set_input(vx=drive_x, vy=drive_y, vz=drive_z, local=local_driving)

        # stop rumble after time
        if self.rumble_timer.hasElapsed(self.rumble_duration):
            self.gamepad.setRumble(wpilib.XboxController.RumbleType.kBothRumble, 0)

    def testInit(self) -> None:
        pass

    def testPeriodic(self) -> None:

        # Cancel any running controllers
        if self.gamepad.getBackButtonPressed():
            self.cancel_controllers()

        self.chassis.update_odometry()

    def cancel_controllers(self):
        self.movement.done()

    def disabledPeriodic(self) -> None:
        self.chassis.update_odometry()

    def autonomousInit(self) -> None:
        pass


if __name__ == "__main__":
    wpilib.run(MyRobot)
