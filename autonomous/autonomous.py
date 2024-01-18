from magicbot.state_machine import AutonomousStateMachine, state
from wpimath.trajectory import (
    TrajectoryConfig,
    Trajectory,
    TrajectoryGenerator,
    TrapezoidProfileRadians,
)
from wpimath.trajectory.constraint import (
    CentripetalAccelerationConstraint,
    RectangularRegionConstraint,
    MaxVelocityConstraint,
)
from wpimath.controller import (
    HolonomicDriveController,
    PIDController,
    ProfiledPIDControllerRadians,
)
from wpilib import Field2d
from wpimath.spline import Spline3
from components.chassis import Chassis
from components.intake import Intake

import utilities.game as game

# Add controllers for intake and shooter when available

from wpimath.geometry import Pose2d, Translation2d
import math

from dataclasses import dataclass


@dataclass
class NotePaths:
    # All paths assume RED alliance
    # They will automatically be flipped if we are blue
    pick_up_path: list[Pose2d]
    shoot_path: list[Pose2d]


class AutoBase(AutonomousStateMachine):
    chassis: Chassis
    intake: Intake
    field: Field2d
    # Add controllers for intake and shooter when available

    POSITION_TOLERANCE = 0.025

    # If the robot is close to the goal but still not enough, making the robot reverse to
    # approach the control vector is unnecessary; this constant scales the derivative of
    # the goal derivative according to the translation distance.
    # The closer the robot gets to the goal, the small the derivative is.
    END_CONTROL_SCALER = 1
    ANGLE_TOLERANCE = math.radians(2)
    MAX_VEL = 1
    MAX_ACCEL = 0.5

    def __init__(self) -> None:
        self.note_paths: list[NotePaths] = []

        x_controller = PIDController(2.5, 0, 0)
        y_controller = PIDController(2.5, 0, 0)
        heading_controller = ProfiledPIDControllerRadians(
            3, 0, 0, TrapezoidProfileRadians.Constraints(2, 2)
        )
        heading_controller.enableContinuousInput(math.pi, -math.pi)

        self.drive_controller = HolonomicDriveController(
            x_controller, y_controller, heading_controller
        )
        # Since robot is stationary from one action to another, point the control vector at the goal to avoid the robot taking unnecessary turns before moving towards the goal
        self.kD = 0.3

    @state(first=True)
    def initialise(self) -> None:
        # Make a working copy of the NotePaths so that we can pop
        # This isn't necessary but makes testing better because we can re-run auto routines
        self.note_paths_working_copy = list(self.note_paths)

        # We always start ready to shoot, so fire straight away
        self.next_state("shoot_note")

    @state
    def shoot_note(self, initial_call: bool) -> None:
        if initial_call:
            # Call the shooter state machine
            pass

        if True:
            # This needs to check if the state machine has finished firing
            if len(self.note_paths_working_copy) == 0:
                # Just shot the last note
                self.done()
            else:
                self.next_state("drive_to_pick_up")

    @state
    def drive_to_pick_up(self, state_tm: float, initial_call: bool) -> None:
        if initial_call:
            self.trajectory = self.calculate_trajectory(
                self.note_paths_working_copy[0].pick_up_path
            )
            # Also deploy the intake

        # Do some driving...
        self.drive_on_trajectory(state_tm)

        if self.is_at_goal() or self.intake.is_note_present():
            # Check if we have a note collected
            self.next_state("drive_to_shoot")

    @state
    def drive_to_shoot(self, state_tm: float, initial_call: bool) -> None:
        if initial_call:
            self.trajectory = self.calculate_trajectory(
                self.note_paths_working_copy[0].shoot_path
            )

        # Do some driving...
        self.drive_on_trajectory(state_tm)

        if self.is_at_goal():
            # If we are in position, remove this note from the list and shoot it
            self.note_paths_working_copy.pop(0)
            self.next_state("shoot_note")

    def drive_on_trajectory(self, state_tm: float):
        target_state = self.trajectory.sample(
            state_tm
        )  # Grabbing the target position at the current point in time from the trajectory.

        # Calculating the speeds required to get to the target position.
        chassis_speed = self.drive_controller.calculate(
            self.chassis.get_pose(),
            target_state,
            self.goal.rotation(),
        )
        self.chassis.drive_local(
            chassis_speed.vx,
            chassis_speed.vy,
            chassis_speed.omega,
        )

    def calculate_trajectory(self, path: list[Pose2d]) -> Trajectory:
        waypoints: list[Translation2d] = []
        waypoints = [
            waypoint.translation()
            if game.is_red()
            else game.field_flip_translation2d(waypoint.translation())
            for waypoint in path[0:-1]
        ]
        self.goal = path[-1] if game.is_red() else game.field_flip_pose2d(path[-1])

        traj_config = TrajectoryConfig(
            maxVelocity=self.MAX_VEL, maxAcceleration=self.MAX_ACCEL
        )
        traj_config.addConstraint(CentripetalAccelerationConstraint(5.0))
        topRight = Translation2d(self.goal.X() + 0.5, self.goal.Y() + 0.5)
        bottomLeft = Translation2d(self.goal.X() - 0.5, self.goal.Y() - 0.5)
        traj_config.addConstraint(
            RectangularRegionConstraint(
                bottomLeft, topRight, MaxVelocityConstraint(0.5)
            )
        )

        pose = self.chassis.get_pose()

        next_pos = waypoints[0] if waypoints else self.goal.translation()
        translation = next_pos - pose.translation()

        # Generating a trajectory when the robot is very close to the goal is unnecesary, so this
        # return an empty trajectory that starts at the end point so the robot won't move.
        distance_to_goal = (self.goal.translation() - pose.translation()).norm()
        if distance_to_goal <= 0.01:
            return Trajectory([Trajectory.State(0, 0, 0, pose)])

        spline_start_momentum_x = translation.x * self.kD
        spline_start_momentum_y = translation.y * self.kD

        goal_spline = Spline3.ControlVector(
            (self.goal.X(), self.goal.rotation().cos() * self.END_CONTROL_SCALER),
            (self.goal.Y(), self.goal.rotation().sin() * self.END_CONTROL_SCALER),
        )

        start_point_spline = Spline3.ControlVector(
            (pose.x, spline_start_momentum_x),
            (pose.y, spline_start_momentum_y),
        )

        chassis_velocity = self.chassis.get_velocity()
        chassis_speed = math.hypot(chassis_velocity.vx, chassis_velocity.vy)
        traj_config.setStartVelocity(chassis_speed)
        try:
            trajectory = TrajectoryGenerator.generateTrajectory(
                start_point_spline, waypoints, goal_spline, traj_config
            )
        except Exception:
            return Trajectory([Trajectory.State(0, 0, 0, pose)])

        self.robot_object = self.field.getObject("auto_trajectory")
        self.robot_object.setTrajectory(trajectory)
        return trajectory

    def is_at_goal(self) -> bool:
        return (
            self.goal.translation() - self.chassis.get_pose().translation()
        ).norm() < self.POSITION_TOLERANCE and abs(
            (self.goal.rotation() - self.chassis.get_rotation()).radians()
        ) < self.ANGLE_TOLERANCE


class PreloadOnly(AutoBase):
    MODE_NAME = "Preload only"


class Front2Note(AutoBase):
    MODE_NAME = "Front of speaker 2 note"

    def setup(self) -> None:
        self.note_paths = [
            NotePaths(
                pick_up_path=[
                    Pose2d(14.2, 5.5, math.radians(0.0)),
                ],
                shoot_path=[
                    Pose2d(14.2, 5.5, math.radians(0.0)),
                ],
            )
        ]
