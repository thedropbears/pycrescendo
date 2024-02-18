import math

from magicbot import StateMachine, state, timed_state, will_reset_to
from wpimath.geometry import Translation2d

from components.chassis import ChassisComponent
from components.injector import InjectorComponent
from components.intake import IntakeComponent
from components.shooter import ShooterComponent
from utilities.game import get_goal_speaker_position


class NoteManager(StateMachine):

    shooter_component: ShooterComponent

    chassis: ChassisComponent
    injector_component: InjectorComponent
    intake: IntakeComponent

    shot_desired = will_reset_to(False)
    intake_desired = will_reset_to(False)
    intake_cancel_desired = will_reset_to(False)

    def __init__(self):
        pass

    def try_intake(self):
        self.intake_desired = True

    def cancel_intake(self):
        self.intake_cancel_desired = True

    def try_shoot(self):
        self.shot_desired = True

    @property
    def translation_to_goal(self) -> Translation2d:
        return (
            get_goal_speaker_position().toTranslation2d()
            - self.chassis.get_pose().translation()
        )

    @state(must_finish=True)
    def idling(self, initial_call):
        if initial_call:
            self.intake.retract()

        # Update range
        self.shooter_component.set_range(self.translation_to_goal.norm())

        if self.intake_desired:
            self.next_state(self.dropping_intake)

    @state(must_finish=True)
    def dropping_intake(self, inital_call):
        if inital_call:
            self.intake.deploy()

        # Update range
        self.shooter_component.set_range(self.translation_to_goal.norm())

        if self.intake.is_fully_deployed():
            self.next_state(self.intaking)

    @state(must_finish=True)
    def intaking(self):
        self.intake.intake()
        self.injector_component.intake()

        # Update range
        self.shooter_component.set_range(self.translation_to_goal.norm())

        if self.injector_component.has_note():
            self.next_state(self.holding_note)

        if self.intake_cancel_desired:
            self.next_state(self.idling)

    @state(first=True, must_finish=True)
    def holding_note(self, initial_call):
        if initial_call:
            self.intake.retract()
        # Update range
        self.shooter_component.set_range(self.translation_to_goal.norm())

        if self.shot_desired:
            self.next_state(self.aiming)

    @state(must_finish=True)
    def aiming(self):
        if not self.shot_desired:
            self.next_state(self.holding_note)

        translation_to_goal = self.translation_to_goal

        # Determine heading required for goal
        bearing_to_speaker = math.atan2(translation_to_goal.y, translation_to_goal.x)

        # Update range
        self.shooter_component.set_range(translation_to_goal.norm())

        # Set to appropriate heading
        self.chassis.snap_to_heading(bearing_to_speaker)
        if self.chassis.at_desired_heading() and self.shooter_component.is_ready():
            self.next_state(self.firing)

    @timed_state(duration=1, next_state=idling)
    def firing(self):
        self.injector_component.shoot()
