import math

from wpimath.geometry import Translation2d

from magicbot import StateMachine, state, timed_state, default_state, feedback

from components.chassis import ChassisComponent
from components.intake import IntakeComponent
from components.shooter import ShooterComponent
from utilities.game import get_goal_speaker_position


class Shooter(StateMachine):
    shooter_component: ShooterComponent
    chassis: ChassisComponent
    intake: IntakeComponent

    def translation_to_goal(self) -> Translation2d:
        return (
            get_goal_speaker_position().toTranslation2d()
            - self.chassis.get_pose().translation()
        )

    @feedback
    def in_range(self):
        range = self.translation_to_goal().norm()
        return self.shooter_component.is_range_in_bounds(range)

    @default_state
    def idling(self) -> None:
        self.shooter_component.set_range(self.translation_to_goal().norm())

    @state(first=True)
    def aiming(self, initial_call) -> None:
        if initial_call:
            self.aim()
        else:
            if (
                # self.chassis.at_desired_heading()
                self.shooter_component.is_ready()
                and self.in_range()
            ):
                self.next_state(self.firing)
            else:
                self.aim()

    def aim(self) -> None:
        translation_to_goal = self.translation_to_goal()

        # Update range
        self.shooter_component.set_range(translation_to_goal.norm())

        # Determine heading required for goal
        bearing_to_speaker = (
            math.atan2(translation_to_goal.y, translation_to_goal.x) + math.pi
        )

        # Set to appropriate heading
        self.chassis.snap_to_heading(bearing_to_speaker)

    @timed_state(duration=1, must_finish=True)
    def firing(self) -> None:
        self.shooter_component.set_range(self.translation_to_goal().norm())
        self.intake.inject()
