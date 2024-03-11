import math

from wpimath.geometry import Translation2d

from magicbot import StateMachine, state, timed_state, feedback, tunable

from components.chassis import ChassisComponent
from components.intake import IntakeComponent
from components.shooter import ShooterComponent
from components.led import LightStrip
from utilities.game import get_goal_speaker_position, NOTE_DIAMETER, SPEAKER_HOOD_WIDTH
from utilities.functions import constrain_angle


class Shooter(StateMachine):
    shooter_component: ShooterComponent
    chassis: ChassisComponent
    intake_component: IntakeComponent
    status_lights: LightStrip

    SPEED_LIMIT = tunable(1)
    SPINNING_SPEED_LIMIT = tunable(1)

    def __init__(self):
        self.range = 0.0
        self.bearing_tolerance = 0.0
        self.bearing_to_speaker = 0.0

    def translation_to_goal(self) -> Translation2d:
        return (
            get_goal_speaker_position().toTranslation2d()
            - self.chassis.get_pose().translation()
        )

    @feedback
    def is_aiming_finished(self) -> bool:
        heading = self.chassis.get_rotation().radians()
        # Check that we are greater than the min angle, and smaller than max. We might wrap past zero, so use the constrain_angle function
        return (
            abs(constrain_angle(self.bearing_to_speaker - heading))
            < self.bearing_tolerance
        )

    def coast_down(self) -> None:
        self.shooter_component.coast_down()

    def update_range(self) -> None:
        self.range = self.translation_to_goal().norm()
        self.shooter_component.set_range(self.range)

    def try_jettison(self):
        self.engage(self.preparing_to_jettison)

    @feedback
    def in_range(self):
        return self.shooter_component.is_range_in_bounds(self.range)

    @state(first=True)
    def aiming(self, initial_call) -> None:
        if initial_call:
            self.aim()
        else:
            if (
                self.is_aiming_finished()
                and self.shooter_component.is_ready()
                and self.in_range()
                and self.is_below_speed_limit()
                and self.is_below_spinning_limit()
            ):
                self.next_state(self.firing)
            else:
                self.aim()

    def is_below_speed_limit(self) -> bool:
        vel = self.chassis.get_velocity()
        return math.hypot(vel.vx, vel.vy) < self.SPEED_LIMIT

    def is_below_spinning_limit(self) -> bool:
        return self.chassis.get_velocity().omega < self.SPINNING_SPEED_LIMIT

    def aim(self) -> None:
        # Update range
        self.update_range()

        # Determine heading required for goal
        translation_to_goal = self.translation_to_goal()

        # We need to aim at least a note's radius inside the outer bounds of the goal. Also add a safety margin
        margin = 0.10
        offset = (SPEAKER_HOOD_WIDTH - NOTE_DIAMETER) / 2.0 - margin
        offset_bearing = constrain_angle(
            math.atan2(translation_to_goal.y + offset, translation_to_goal.x) + math.pi
        )

        self.bearing_to_speaker = constrain_angle(
            math.atan2(translation_to_goal.y, translation_to_goal.x) + math.pi
        )

        self.bearing_tolerance = abs(
            constrain_angle(self.bearing_to_speaker - offset_bearing)
        )

        # Set to appropriate heading
        self.chassis.snap_to_heading(self.bearing_to_speaker)

    @state
    def preparing_to_jettison(self) -> None:
        self.shooter_component.prepare_to_jettison()
        if self.shooter_component.is_ready():
            self.next_state(self.firing)

    @state(must_finish=True)
    def firing(self) -> None:
        self.intake_component.feed_shooter()
        if not self.intake_component.has_note():
            self.next_state(self.waiting_for_shot_to_complete)

    @timed_state(duration=0.2, must_finish=True)
    def waiting_for_shot_to_complete(self):
        self.intake_component.feed_shooter()
