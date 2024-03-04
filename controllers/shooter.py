import math
import numpy as np

from wpimath.geometry import Translation2d

from magicbot import StateMachine, state, timed_state, feedback

from components.chassis import ChassisComponent
from components.intake import IntakeComponent
from components.shooter import ShooterComponent
from components.led import LightStrip
from utilities.game import get_goal_speaker_position
from utilities.functions import constrain_angle


class Shooter(StateMachine):
    shooter_component: ShooterComponent
    chassis: ChassisComponent
    intake: IntakeComponent
    status_lights: LightStrip

    # make sure this is always > chassis heading tolerance
    ANGLE_TOLERANCES = (math.radians(5), math.radians(1))
    RANGES = (0, 5)

    def __init__(self):
        self.range = 0.0
        self.bearing_to_speaker = 0.0

    def translation_to_goal(self) -> Translation2d:
        return (
            get_goal_speaker_position().toTranslation2d()
            - self.chassis.get_pose().translation()
        )

    @feedback
    def is_aiming_finished(self) -> bool:
        tolerance = float(np.interp(self.range, self.RANGES, self.ANGLE_TOLERANCES))
        heading = self.chassis.get_rotation().radians()
        return abs(constrain_angle(self.bearing_to_speaker - heading)) < tolerance

    def coast_down(self) -> None:
        self.shooter_component.coast_down()

    def update_range(self) -> None:
        self.range = self.translation_to_goal().norm()
        self.shooter_component.set_range(self.range)

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
            ):
                self.next_state(self.firing)
            else:
                self.aim()

    def aim(self) -> None:
        translation_to_goal = self.translation_to_goal()

        # Update range
        self.update_range()

        # Determine heading required for goal
        self.bearing_to_speaker = constrain_angle(
            math.atan2(translation_to_goal.y, translation_to_goal.x) + math.pi
        )

        # Set to appropriate heading
        self.chassis.snap_to_heading(self.bearing_to_speaker)

    @state(must_finish=True)
    def firing(self) -> None:
        self.update_range()
        self.intake.inject()
        if not self.intake.has_note():
            self.next_state(self.waiting_for_shot_to_complete)

    @timed_state(duration=0.2, must_finish=True)
    def waiting_for_shot_to_complete(self):
        self.update_range()
        self.intake.inject()
