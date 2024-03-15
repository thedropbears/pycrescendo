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
from components.climber import Climber
from components.led import LightStrip

from controllers.note import NoteManager
from controllers.intake import Intake
from controllers.shooter import Shooter

from autonomous.base import AutoBase

from utilities.game import is_red
from utilities.scalers import rescale_js
from utilities.functions import clamp
from utilities.position import on_same_side_of_stage, y_close_to_stage


class MyRobot(magicbot.MagicRobot):
    # Controllers
    note_manager: NoteManager
    shooter: Shooter
    intake: Intake

    # Components
    chassis: ChassisComponent
    climber: Climber
    shooter_component: ShooterComponent
    intake_component: IntakeComponent

    status_lights: LightStrip

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

        # side: (28*3)*2 + front: (30*3) - 2 (R.I.P)
        self.status_lights_strip_length = (28 * 3) * 2 + (30 * 3) - 2

        self.vision_port_name = "ardu_cam_port"
        self.vision_port_pos = Translation3d(0.005, 0.221, 0.503)
        self.vision_port_rot = Rotation3d(
            0, -math.radians(20), math.radians(180) - math.radians(90 - 71.252763)
        )

        self.vision_starboard_name = "ardu_cam_starboard"
        self.vision_starboard_pos = Translation3d(0.005, 0.161, 0.503)
        self.vision_starboard_rot = Rotation3d(
            0, -math.radians(20), math.radians(180) + math.radians(90 - 71.252763)
        )

    def teleopInit(self) -> None:
        pass

    def teleopPeriodic(self) -> None:
        if self.climber.should_lock_mechanisms():
            self.shooter_component.lock()
            self.intake_component.lock()
        else:
            self.shooter_component.unlock()
            self.intake_component.unlock()

        # Set max speed
        max_speed = self.max_speed
        max_spin_rate = self.max_spin_rate
        if self.gamepad.getXButton():
            max_speed = self.lower_max_speed
            max_spin_rate = self.lower_max_spin_rate

        # Driving
        drive_x = -rescale_js(self.gamepad.getLeftY(), 0.05, 2.5) * max_speed
        drive_y = -rescale_js(self.gamepad.getLeftX(), 0.05, 2.5) * max_speed
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
        # dpad upwards
        if dpad in (0, 45, 315):
            self.climber.deploy()
        elif dpad in (135, 180, 235):
            self.climber.retract()

        # Set current robot direction to forward
        if self.gamepad.getBButtonPressed():
            self.chassis.reset_yaw()

        # Reset Odometry
        if self.gamepad.getStartButtonPressed():
            self.chassis.reset_odometry()

        # Reverse intake and shoot shooter
        if self.gamepad.getBackButton():
            self.note_manager.jettison()

        # Intake
        if self.gamepad.getLeftTriggerAxis() > 0.5:
            self.note_manager.try_intake()

        # Cancel intaking
        if self.gamepad.getLeftBumper():
            self.note_manager.try_cancel_intake()

        # Shoot
        if self.gamepad.getRightTriggerAxis() > 0.5:
            self.note_manager.try_shoot()

    def testInit(self) -> None:
        pass

    def testPeriodic(self) -> None:
        # moving arm
        if self.gamepad.getAButton():
            self.intake_component.deploy()
        elif self.gamepad.getYButton():
            self.intake_component.retract()

        dpad = self.gamepad.getPOV()
        if dpad != -1:
            if is_red():
                self.chassis.snap_to_heading(-math.radians(dpad) + math.pi)
            else:
                self.chassis.snap_to_heading(-math.radians(dpad))
        else:
            self.chassis.stop_snapping()
            self.chassis.drive_local(0, 0, 0)

        # injecting
        if self.gamepad.getBButton():
            self.intake_component.feed_shooter()

        if self.gamepad.getXButton():
            self.intake_component.intake()

        if self.gamepad.getLeftBumperPressed():
            self.shooter_component.desired_inclinator_angle = clamp(
                self.shooter_component.desired_inclinator_angle + 0.01,
                self.shooter_component.MIN_INCLINE_ANGLE,
                self.shooter_component.MAX_INCLINE_ANGLE,
            )

        if self.gamepad.getRightBumperPressed():
            self.shooter_component.desired_inclinator_angle = clamp(
                self.shooter_component.desired_inclinator_angle - 0.01,
                self.shooter_component.MIN_INCLINE_ANGLE,
                self.shooter_component.MAX_INCLINE_ANGLE,
            )

        self.intake_component.execute()
        self.shooter_component.execute()
        self.climber.execute()
        self.chassis.execute()

        self.chassis.update_odometry()

        self.status_lights.execute()
        self.vision_port.execute()
        self.vision_starboard.execute()

    def disabledPeriodic(self) -> None:
        self.chassis.update_alliance()
        self.chassis.update_odometry()

        self.intake_component.maybe_reindex_deployment_encoder()
        self.status_lights.execute()
        self.vision_port.execute()
        self.vision_starboard.execute()

        # check if we can see targets
        if (
            not self.vision_port.sees_target()
            and not self.vision_starboard.sees_target()
        ):
            self.status_lights.no_vision()
        else:
            # check we start on the correct side of the stage
            selected_auto = self._automodes.chooser.getSelected()
            if isinstance(selected_auto, AutoBase):
                intended_start_pose = selected_auto.get_starting_pose()
                current_pose = self.chassis.get_pose()
                if intended_start_pose is not None:
                    if on_same_side_of_stage(intended_start_pose, current_pose):
                        if y_close_to_stage(current_pose):
                            self.status_lights.too_close_to_stage()
                        else:
                            self.status_lights.rainbow()
                    else:
                        self.status_lights.invalid_start()
                else:
                    self.status_lights.missing_start_pose()
            else:
                self.status_lights.missing_start_pose()

    def autonomousInit(self) -> None:
        pass


if __name__ == "__main__":
    wpilib.run(MyRobot)
