#!/usr/bin/env python3

import wpilib
import wpilib.event
import magicbot

from wpimath.geometry import Quaternion, Rotation3d, Translation3d, Pose2d, Rotation2d
from components.chassis import Chassis
from controllers.movement import Movement

from utilities.scalers import rescale_js


class MyRobot(magicbot.MagicRobot):
    # Components
    chassis: Chassis

    movement: Movement

    max_speed = magicbot.tunable(Chassis.max_wheel_speed * 0.95)

    def createObjects(self) -> None:
        self.data_log = wpilib.DataLogManager.getLog()

        self.gamepad = wpilib.XboxController(0)
        self.joystick = wpilib.Joystick(1)

        self.event_loop = wpilib.event.EventLoop()
        # Right trigger events
        self.right_trigger_down_full = self.gamepad.rightTrigger(
            0.95, self.event_loop
        ).rising()
        self.right_trigger_down_half = self.gamepad.rightTrigger(0.05, self.event_loop)
        self.right_trigger_up = self.gamepad.rightTrigger(
            0.95, self.event_loop
        ).falling()

        # Left trigger events
        self.left_trigger_down_full = self.gamepad.leftTrigger(
            0.95, self.event_loop
        ).rising()
        self.left_trigger_down_half = self.gamepad.leftTrigger(
            0.05, self.event_loop
        ).rising()
        self.left_trigger_up = self.gamepad.leftTrigger(0.95, self.event_loop).falling()

        self.rumble_timer = wpilib.Timer()
        self.rumble_timer.start()
        self.rumble_duration = 0.0

        self.field = wpilib.Field2d()
        wpilib.SmartDashboard.putData(self.field)
        self.target_node = self.field.getObject("target_node")

        self.port_localizer_name = "cam_port"
        self.port_localizer_pos = Translation3d(-0.35001, 0.06583, 0.25)
        self.port_localizer_rot = Rotation3d(
            Quaternion(
                0.0850897952914238,
                0.21561633050441742,
                -0.9725809097290039,
                0.018864024430513382,
            )
        )

        self.starboard_localizer_name = "cam_starboard"
        self.starboard_localizer_pos = Translation3d(-0.35001, -0.06583, 0.247)
        self.starboard_localizer_rot = Rotation3d(
            Quaternion(
                0.08508981764316559,
                -0.21561576426029205,
                -0.9725810289382935,
                -0.01886390522122383,
            )
        )
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
        self.event_loop.poll()
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
