from dataclasses import dataclass
import math
from magicbot.state_machine import AutonomousStateMachine, state
from wpimath.trajectory import (
    TrajectoryConfig,
    Trajectory,
    TrajectoryGenerator,
    TrapezoidProfileRadians,
)
from wpimath.trajectory.constraint import (
    CentripetalAccelerationConstraint,
)
from wpimath.controller import (
    HolonomicDriveController,
    PIDController,
    ProfiledPIDControllerRadians,
)
from wpilib import Field2d
from wpimath.spline import Spline3
from wpimath.geometry import Rotation2d, Translation2d

from utilities.position import NotePoses
import utilities.game as game

from components.chassis import ChassisComponent
from components.intake import IntakeComponent

# Add controllers for intake and shooter when available
from controllers.shooter import Shooter


@dataclass
class Path:
    waypoints: list[Translation2d]
    final_heading: Rotation2d

    def copy(self):
        return Path(
            [Translation2d(i.x, i.y) for i in self.waypoints],
            Rotation2d(self.final_heading.radians()),
        )


@dataclass
class NotePaths:
    # All paths assume RED alliance
    # They will automatically be flipped if we are blue
    pick_up_path: Path
    shoot_path: Path
    pickup_offset: Translation2d


class AutoBase(AutonomousStateMachine):
    chassis: ChassisComponent
    intake: IntakeComponent
    shooter: Shooter
    field: Field2d

    POSITION_TOLERANCE = 0.025
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
        self.pathstate = 0

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
            # TODO Call the shooter state machine
            # TODO Also get intake out at this time??
            pass

        if True:
            # TODO This needs to check if the state machine has finished firing
            if len(self.note_paths_working_copy) == 0:
                # Just shot the last note
                self.done()
            else:
                self.next_state("drive_to_pick_up")

    @state
    def drive_to_pick_up(self, state_tm: float, initial_call: bool) -> None:
        if initial_call:
            # go to just behind the note
            self.pathstate = 0
            newpath = self.note_paths_working_copy[0].pick_up_path.copy()
            newpath.waypoints[-1] += self.note_paths_working_copy[0].pickup_offset
            newpath.final_heading = (
                self.note_paths_working_copy[0].pickup_offset.angle()
            ) + Rotation2d(math.pi)
            self.trajectory = self.calculate_trajectory(newpath)

        # Do some driving...
        self.drive_on_trajectory(state_tm)

        if self.intake.is_note_present():
            # Check if we have a note collected
            self.next_state("drive_to_shoot")
        if self.is_at_goal():
            if self.pathstate == 0:
                self.pathstate = 1
                # TODO Also deploy the intake
                self.trajectory = self.calculate_trajectory(
                    Path(
                        [self.note_paths_working_copy[0].pick_up_path.waypoints[-1]],
                        (self.note_paths_working_copy[0].pickup_offset.angle())
                        + Rotation2d(math.pi),
                    )
                )
            elif self.pathstate == 1:
                self.pathstate = 0
                if not self.intake.is_note_present():
                    pass  # TODO: do something if we don't have a note, e.g. go to next note position
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

    def drive_on_trajectory(self, trajectory_tm: float):
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

        self.robot_object = self.field.getObject("auto_trajectory")
        self.robot_object.setTrajectory(trajectory)
        return trajectory

    def is_at_goal(self) -> bool:
        return (
            self.goal - self.chassis.get_pose().translation()
        ).norm() < self.POSITION_TOLERANCE and abs(
            (self.goal_heading - self.chassis.get_rotation()).radians()
        ) < self.ANGLE_TOLERANCE

    def done(self):
        self.robot_object.setPoses([])
        super().done()


def rotation_to_red_speaker(position: Translation2d) -> Rotation2d:
    t: Translation2d = game.RED_SPEAKER_POSE.toPose2d().translation() - position
    return Rotation2d(math.atan2(t.y, t.x))


class AllNotes(AutoBase):
    MODE_NAME = "All the notes"

    def setup(self) -> None:
        self.note_paths = [
            NotePaths(
                pick_up_path=Path(
                    [NotePoses.Stage1.translation()],
                    Rotation2d(),
                ),
                shoot_path=Path(
                    [NotePoses.Stage1.translation()],
                    rotation_to_red_speaker(NotePoses.Stage1.translation()),
                ),
                pickup_offset=Translation2d(1, -0.6),
            ),
            NotePaths(
                pick_up_path=Path(
                    [NotePoses.Stage2.translation()],
                    Rotation2d(),
                ),
                shoot_path=Path(
                    [NotePoses.Stage2.translation()],
                    rotation_to_red_speaker(NotePoses.Stage2.translation()),
                ),
                pickup_offset=Translation2d(0, 1),
            ),
            NotePaths(
                pick_up_path=Path(
                    [
                        NotePoses.Stage3.translation() + Translation2d(0.3, 0.1)
                    ],  # otherwise it'll hit the stage leg
                    Rotation2d(),
                ),
                shoot_path=Path(
                    [
                        NotePoses.Stage3.translation() + Translation2d(1, 0)
                    ],  # drive it back out to avoid the stage leg when turning
                    rotation_to_red_speaker(NotePoses.Stage3.translation()),
                ),
                pickup_offset=Translation2d(0.9, 0.4),
            ),
        ]


class Front2Note(AutoBase):
    MODE_NAME = "Front of speaker 2 note"

    def setup(self) -> None:
        self.note_paths = [
            NotePaths(
                pick_up_path=Path(
                    [NotePoses.Stage2.translation()],
                    Rotation2d(),
                ),
                shoot_path=Path(
                    [NotePoses.Stage2.translation()],
                    rotation_to_red_speaker(NotePoses.Stage2.translation()),
                ),
                pickup_offset=Translation2d(0, -1),
            )
        ]
