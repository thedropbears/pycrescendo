import math

from wpimath.geometry import Translation2d

from magicbot import StateMachine, state, timed_state, feedback

from components.chassis import ChassisComponent
from components.intake import IntakeComponent
from components.shooter import ShooterComponent
from components.led import LightStrip
from utilities.game import get_goal_speaker_position


class Shooter(StateMachine):
    shooter_component: ShooterComponent
    chassis: ChassisComponent
    intake: IntakeComponent
    status_lights: LightStrip

    def translation_to_goal(self) -> Translation2d:
        return (
            get_goal_speaker_position().toTranslation2d()
            - self.chassis.get_pose().translation()
        )

    def coast_down(self) -> None:
        self.shooter_component.coast_down()

    def update_range(self) -> None:
        self.shooter_component.set_range(self.translation_to_goal().norm())

    @feedback
    def in_range(self):
        range = self.translation_to_goal().norm()
        return self.shooter_component.is_range_in_bounds(range)

    @state(first=True)
    def aiming(self, initial_call) -> None:
        if initial_call:
            self.aim()
        else:
            if (
                self.chassis.at_desired_heading()
                and self.shooter_component.is_ready()
                and self.in_range()
            ):
                self.status_lights.shooting()
                self.next_state(self.firing)
            else:
                self.status_lights.not_in_range()
                self.aim()

    def aim(self) -> None:
        translation_to_goal = self.translation_to_goal()

        # Update range
        self.update_range()

        # Determine heading required for goal
        bearing_to_speaker = (
            math.atan2(translation_to_goal.y, translation_to_goal.x) + math.pi
        )

        # Set to appropriate heading
        self.chassis.snap_to_heading(bearing_to_speaker)

    @timed_state(duration=1, must_finish=True)
    def firing(self) -> None:
        self.update_range()
        self.intake.inject()
