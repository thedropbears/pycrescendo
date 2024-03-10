import math
from typing import Optional
from magicbot.state_machine import AutonomousStateMachine, state
from wpimath.trajectory import (
    TrajectoryConfig,
    Trajectory,
    TrajectoryGenerator,
)
from wpimath.trajectory.constraint import (
    CentripetalAccelerationConstraint,
)
from wpimath.controller import (
    HolonomicDriveController,
    PIDController,
)
from wpilib import Field2d, RobotBase
from wpimath.geometry import Rotation2d, Translation2d, Pose2d
from wpimath.spline import Spline3

from utilities.position import Path
import utilities.game as game
from utilities.game import get_goal_speaker_position

from components.chassis import ChassisComponent
from components.intake import IntakeComponent

from controllers.note import NoteManager


class AutoBase(AutonomousStateMachine):
    chassis: ChassisComponent
    note_manager: NoteManager
    field: Field2d

    intake_component: IntakeComponent

    POSITION_TOLERANCE = 0.05
    ANGLE_TOLERANCE = math.radians(5)
    MAX_VEL = 3
    MAX_ACCEL = 2
    ENFORCE_HEADING_SPEED = MAX_VEL / 6

    def __init__(
        self,
        note_paths: list[Path],
        shoot_paths: list[Path],
        starting_pose: Optional[Pose2d] = None,
    ):
        """Should be overloaded by subclass method with paths"""
        self.note_paths = note_paths
        self.shoot_paths = shoot_paths
        self.starting_pose = starting_pose

    def setup(self) -> None:
        x_controller = PIDController(2.5, 0, 0)
        y_controller = PIDController(2.5, 0, 0)
        heading_controller = self.chassis.heading_controller

        self.drive_controller = HolonomicDriveController(
            x_controller, y_controller, heading_controller
        )
        # Since robot is stationary from one action to another, point the control vector at the goal to avoid the robot taking unnecessary turns before moving towards the goal
        self.kD = 0.3

        for i, path in enumerate(self.shoot_paths):
            self.shoot_paths[i].final_heading = rotation_to_red_speaker(
                path.waypoints[-1]
            )

        self.goal_heading: Rotation2d
        self.trajectory_marker = self.field.getObject("auto_trajectory")

    def on_enable(self):
        # Setup starting position in the simulator
        if RobotBase.isSimulation() and self.starting_pose is not None:
            starting_pose = self.starting_pose
            if not game.is_red():
                starting_pose = game.field_flip_pose2d(self.starting_pose)
            self.chassis.set_pose(starting_pose)
        super().on_enable()

    @state(first=True)
    def initialise(self) -> None:
        # Make a working copy of the NotePaths so that we can pop
        # This isn't necessary but makes testing better because we can re-run auto routines
        self.note_paths_working_copy = list(self.note_paths)
        self.shoot_paths_working_copy = list(self.shoot_paths)

        self.intake_component.deploy()

        # We always start ready to shoot, so fire straight away
        self.next_state("shoot_note")

    @state
    def shoot_note(self) -> None:
        self.note_manager.try_shoot()

        if self.note_manager.has_just_fired() or RobotBase.isSimulation():
            if len(self.note_paths_working_copy) == 0:
                # Just shot the last note
                self.done()
            else:
                self.next_state(self.ensure_robot_config)

    @state
    def ensure_robot_config(self):
        if self.intake_component.is_fully_deployed() or RobotBase.isSimulation():
            self.next_state(self.pick_up)

    @state
    def pick_up(self, state_tm: float, initial_call: bool) -> None:
        if initial_call:
            # go to just behind the note
            self.trajectory = self.calculate_trajectory(
                self.note_paths_working_copy.pop(0)
            )

        self.note_manager.try_intake()

        # Drive with the intake always facing the tangent
        self.drive_on_trajectory(state_tm, enforce_tangent_heading=True)

        if self.note_manager.has_note():
            # Check if we have a note collected
            # Return heading control to path controller
            self.chassis.stop_snapping()
            self.next_state("drive_to_shoot")
        if self.is_at_goal():
            self.next_state(self.drive_to_shoot)

    def translation_to_goal(self, position: Translation2d) -> Translation2d:
        return get_goal_speaker_position().toTranslation2d() - position

    @state
    def drive_to_shoot(self, state_tm: float, initial_call: bool) -> None:
        if initial_call:
            last_waypoint = self.shoot_paths_working_copy[0].waypoints[-1]
            self.trajectory = self.calculate_trajectory(
                self.shoot_paths_working_copy.pop(0)
            )
            translation_to_goal = self.translation_to_goal(last_waypoint)
            self.goal_heading = Rotation2d(
                math.atan2(translation_to_goal.y, translation_to_goal.x) + math.pi
            )

        # Do some driving...
        self.drive_on_trajectory(state_tm)

        if self.is_at_goal():
            if self.note_manager.has_note():
                # If we are in position, remove this note from the list and shoot it
                self.next_state("shoot_note")
            else:
                if len(self.note_paths_working_copy) != 0:
                    self.next_state("pick_up")
                else:
                    self.done()

    def drive_on_trajectory(
        self, trajectory_tm: float, enforce_tangent_heading: bool = False
    ):
        # Grabbing the target position at the current point in time from the trajectory.
        target_state = self.trajectory.sample(trajectory_tm)

        # Calculating the speeds required to get to the target position.
        chassis_speed = self.drive_controller.calculate(
            self.chassis.get_pose(),
            target_state,
            self.goal_heading,
        )
        self.chassis.drive_local(
            chassis_speed.vx,
            chassis_speed.vy,
            chassis_speed.omega,
        )

        # if we are enforcing heading, hijack rotational control from the main controller
        if enforce_tangent_heading:
            speed = Translation2d(chassis_speed.vx, chassis_speed.vy).norm()
            if speed > self.ENFORCE_HEADING_SPEED:
                field_chassis_speeds = self.chassis.to_field_oriented(chassis_speed)
                heading_target = math.atan2(
                    field_chassis_speeds.vy, field_chassis_speeds.vx
                )
                self.goal_heading = Rotation2d(heading_target)
                self.chassis.snap_to_heading(heading_target)
        else:
            self.chassis.snap_to_heading(self.goal_heading.radians())

    def calculate_trajectory(self, path: Path) -> Trajectory:
        pose = self.chassis.get_pose()

        waypoints: list[Translation2d]
        if game.is_red():
            waypoints = path.waypoints[:-1]
            self.goal = path.waypoints[-1]
            self.goal_heading = path.final_heading
        else:
            waypoints = [
                game.field_flip_translation2d(waypoint)
                for waypoint in path.waypoints[:-1]
            ]
            self.goal = game.field_flip_translation2d(path.waypoints[-1])
            self.goal_heading = game.field_flip_rotation2d(path.final_heading)

        traj_config = TrajectoryConfig(
            maxVelocity=self.MAX_VEL, maxAcceleration=self.MAX_ACCEL
        )
        traj_config.addConstraint(CentripetalAccelerationConstraint(5.0))

        # Generating a trajectory when the robot is very close to the goal is unnecesary, so this
        # return an empty trajectory that starts at the end point so the robot won't move.
        distance_to_goal = (self.goal - pose.translation()).norm()
        if distance_to_goal <= self.POSITION_TOLERANCE:
            return Trajectory([Trajectory.State(0, 0, 0, pose)])

        next_pos = waypoints[0] if waypoints else self.goal
        translation = next_pos - pose.translation()

        spline_start_momentum_x = translation.x * self.kD
        spline_start_momentum_y = translation.y * self.kD
        start_point_spline = Spline3.ControlVector(
            (pose.x, spline_start_momentum_x),
            (pose.y, spline_start_momentum_y),
        )

        prev_pos = waypoints[-1] if waypoints else pose.translation()
        translation = self.goal - prev_pos

        spline_goal_momentum_x = translation.x * self.kD
        spline_goal_momentum_y = translation.y * self.kD
        goal_spline = Spline3.ControlVector(
            (self.goal.X(), spline_goal_momentum_x),
            (self.goal.Y(), spline_goal_momentum_y),
        )

        traj_config.setStartVelocity(0.0)
        try:
            trajectory = TrajectoryGenerator.generateTrajectory(
                start_point_spline, waypoints, goal_spline, traj_config
            )
        except Exception:
            return Trajectory([Trajectory.State(0, 0, 0, pose)])

        self.trajectory_marker.setTrajectory(trajectory)
        return trajectory

    def is_at_goal(self) -> bool:
        return (
            self.goal - self.chassis.get_pose().translation()
        ).norm() < self.POSITION_TOLERANCE

    def done(self):
        self.chassis.stop_snapping()
        self.trajectory_marker.setPoses([])
        super().done()


def rotation_to_red_speaker(position: Translation2d) -> Rotation2d:
    t = game.RED_SPEAKER_POSE.toPose2d().translation() - position
    return t.angle() + Rotation2d(math.pi)
