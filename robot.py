#!/usr/bin/env python3

import wpilib
import wpilib.event
import magicbot
from magicbot import tunable

from components.chassis import ChassisComponent
from components.shooter import ShooterComponent
from components.intake import IntakeComponent
from components.climber import ClimberComponent

from controllers.shooter import Shooter

import math

from utilities.scalers import rescale_js


class MyRobot(magicbot.MagicRobot):
    # Controllers
    shooter: Shooter

    # Components
    chassis: ChassisComponent
    shooter_component: ShooterComponent
    intake: IntakeComponent
    climber_component: ClimberComponent

    max_speed = magicbot.tunable(ChassisComponent.max_wheel_speed * 0.95)
    inclination_angle = tunable(0.0)

    def createObjects(self) -> None:
        self.data_log = wpilib.DataLogManager.getLog()

        self.gamepad = wpilib.XboxController(0)

        self.rumble_timer = wpilib.Timer()
        self.rumble_timer.start()
        self.rumble_duration = 0.0

        self.field = wpilib.Field2d()
        wpilib.SmartDashboard.putData(self.field)

    def rumble_for(self, intensity: float, duration: float):
        self.rumble_duration = duration
        self.rumble_timer.reset()
        self.gamepad.setRumble(wpilib.XboxController.RumbleType.kBothRumble, intensity)

    def short_rumble(self):
        self.rumble_for(0.4, 0.1)

    def long_rumble(self):
        self.rumble_for(0.8, 0.3)

    def teleopInit(self) -> None:
        self.dpad_angle = 0.0

    def teleopPeriodic(self) -> None:
        # Driving
        spin_rate = 4
        drive_x = -rescale_js(self.gamepad.getLeftY(), 0.1) * self.max_speed
        drive_y = -rescale_js(self.gamepad.getLeftX(), 0.1) * self.max_speed
        drive_z = -rescale_js(self.gamepad.getRightX(), 0.1, exponential=2) * spin_rate
        local_driving = self.gamepad.getYButton()
        driver_inputs = (drive_x, drive_y, drive_z)
        if local_driving:
            self.chassis.drive_local(*driver_inputs)
        else:
            self.chassis.drive_field(*driver_inputs)

        # give rotational access to the driver
        if drive_z != 0:
            self.chassis.stop_snapping()

        if self.gamepad.getAButtonPressed():
            self.chassis.snap_to_heading(math.radians(self.dpad_angle))

        # stop rumble after time
        if self.rumble_timer.hasElapsed(self.rumble_duration):
            self.gamepad.setRumble(wpilib.XboxController.RumbleType.kBothRumble, 0)

    def testInit(self) -> None:
        pass

    def testPeriodic(self) -> None:
        # injecting
        if self.gamepad.getBButton():
            self.shooter.shoot()

        if self.gamepad.getXButton():
            self.intake.intake()

        if self.gamepad.getAButton():
            self.shooter_component.set_inclination(math.radians(self.inclination_angle))

        if self.gamepad.getLeftBumper():
            self.climber_component.deploy()

        if self.gamepad.getRightBumper():
            self.climber_component.retract()

        # Cancel any running controllers
        if self.gamepad.getBackButtonPressed():
            self.cancel_controllers()

        self.shooter.execute()

        self.intake.execute()
        self.shooter_component.execute()

        self.chassis.update_odometry()

    def cancel_controllers(self):
        pass

    def disabledPeriodic(self) -> None:
        self.chassis.update_odometry()

    def autonomousInit(self) -> None:
        pass


if __name__ == "__main__":
    wpilib.run(MyRobot)
