#!/usr/bin/env python3
import math
import wpilib
import wpilib.event
from wpimath.geometry import Rotation3d, Translation3d
import magicbot
from magicbot import tunable

from components.chassis import ChassisComponent
from components.vision import VisualLocalizer
from components.shooter import ShooterComponent
from components.intake import IntakeComponent
from components.climber import ClimberComponent
from components.led import LightStrip

from controllers.note import NoteManager
from controllers.shooter import Shooter
from controllers.climber import Climber

from utilities.game import is_red


from utilities.scalers import rescale_js


class MyRobot(magicbot.MagicRobot):
    # Controllers
    note_manager: NoteManager
    shooter: Shooter
    climber: Climber

    # Components
    chassis: ChassisComponent
    shooter_component: ShooterComponent
    intake: IntakeComponent
    climber_component: ClimberComponent
    lights: LightStrip

    max_speed = magicbot.tunable(4)  # m/s
    lower_max_speed = magicbot.tunable(2)  # m/s
    max_spin_rate = magicbot.tunable(4)  # m/s
    lower_max_spin_rate = magicbot.tunable(2)  # m/s
    inclination_angle = tunable(0.0)
    vision_port: VisualLocalizer
    vision_starboard: VisualLocalizer

    def createObjects(self) -> None:
        self.data_log = wpilib.DataLogManager.getLog()

        self.gamepad = wpilib.XboxController(0)

        self.field = wpilib.Field2d()
        wpilib.SmartDashboard.putData(self.field)

        self.lights_strip_length = 144  # TODO Change to correct length

        self.vision_port_name = "ardu_cam_port"
        self.vision_port_pos = Translation3d(0.005, 0.161, 0.503)
        self.vision_port_rot = Rotation3d(
            0, -math.radians(20), math.radians(180) - math.radians(90 - 71.252763)
        )

        self.vision_starboard_name = "ardu_cam_starboard"
        self.vision_starboard_pos = Translation3d(0.005, 0.221, 0.503)
        self.vision_starboard_rot = Rotation3d(
            0, -math.radians(20), math.radians(180) + math.radians(90 - 71.252763)
        )

    def teleopInit(self) -> None:
        pass

    def teleopPeriodic(self) -> None:
        # Set max speed
        max_speed = self.max_speed
        max_spin_rate = self.max_spin_rate
        if self.gamepad.getXButton():
            max_speed = self.lower_max_speed
            max_spin_rate = self.lower_max_spin_rate

        # Driving
        drive_x = -rescale_js(self.gamepad.getLeftY(), 0.1) * max_speed
        drive_y = -rescale_js(self.gamepad.getLeftX(), 0.1) * max_speed
        drive_z = (
            -rescale_js(self.gamepad.getRightX(), 0.1, exponential=2) * max_spin_rate
        )
        local_driving = self.gamepad.getYButton()

        if local_driving:
            self.chassis.drive_local(drive_x, drive_y, drive_z)
        else:
            if is_red():
                drive_x = -drive_x
                drive_y = -drive_y
            self.chassis.drive_field(drive_x, drive_y, drive_z)

        # Give rotational access to the driver
        if drive_z != 0:
            self.chassis.stop_snapping()

        dpad = self.gamepad.getPOV()
        if dpad != -1:
            if is_red():
                self.chassis.snap_to_heading(-math.radians(dpad) + math.pi)
            else:
                self.chassis.snap_to_heading(-math.radians(dpad))

        # Set current robot direction to forward
        if self.gamepad.getBButtonPressed():
            self.chassis.reset_yaw()

        # Reset Odometry
        if self.gamepad.getStartButtonPressed():
            self.chassis.reset_odometry()

        # Reverse intake and shoot shooter
        if self.gamepad.getBackButton():
            # TODO add this capability to note manager
            pass

        # Climbing arm controls. Toggles!
        if self.gamepad.getRightBumperPressed():
            self.climber.try_toggle()

        # Intake
        if self.gamepad.getLeftTriggerAxis() > 0.5:
            self.note_manager.try_intake()

        # Cancel intaking
        if self.gamepad.getLeftBumper():
            self.note_manager.cancel_intake()

        # Shoot
        if self.gamepad.getRightTriggerAxis() > 0.5:
            self.note_manager.try_shoot()

    def testInit(self) -> None:
        pass

    def testPeriodic(self) -> None:
        # moving arm
        if self.gamepad.getAButton():
            self.intake.deploy()
        elif self.gamepad.getYButton():
            self.intake.retract()

        # injecting
        if self.gamepad.getBButton():
            self.intake.inject()

        if self.gamepad.getXButton():
            self.intake.intake()

        # Climbing arm controls
        if self.gamepad.getLeftBumper():
            self.climber_component.deploy()

        if self.gamepad.getRightBumper():
            self.climber_component.retract()

        # Cancel any running controllers
        if self.gamepad.getBackButtonPressed():
            self.cancel_controllers()

        self.intake.execute()
        self.shooter_component.execute()
        self.climber_component.execute()

        self.chassis.update_odometry()

        self.vision_port.execute()
        self.vision_starboard.execute()

    def cancel_controllers(self):
        self.climber.stop()

    def disabledPeriodic(self) -> None:
        self.chassis.update_alliance()
        self.chassis.update_odometry()

        self.intake.maybe_reindex_deployment_encoder()
        self.lights.execute()
        self.vision_port.execute()
        self.vision_starboard.execute()

    def autonomousInit(self) -> None:
        pass


if __name__ == "__main__":
    wpilib.run(MyRobot)
