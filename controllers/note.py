from magicbot import StateMachine, state, will_reset_to

from components.injector import InjectorComponent
from components.intake import IntakeComponent
from controllers.shooter import Shooter


class NoteManager(StateMachine):

    shooter: Shooter

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

    @state(first=True, must_finish=True)
    def idling(self, initial_call):
        if initial_call:
            self.intake.retract()

        if self.intake_desired:
            self.next_state(self.dropping_intake)

    @state(must_finish=True)
    def dropping_intake(self, inital_call):
        if inital_call:
            self.intake.deploy()

        if self.intake.is_fully_deployed():
            self.next_state(self.intaking)

    @state(must_finish=True)
    def intaking(self):
        self.intake.intake()
        self.injector_component.intake()

        if self.injector_component.has_note():
            self.next_state(self.holding_note)

        if self.intake_cancel_desired:
            self.next_state(self.idling)

    @state(must_finish=True)
    def holding_note(self, initial_call):
        if initial_call:
            self.intake.retract()
        if self.shot_desired:
            self.next_state(self.trying_to_shoot)

    @state(must_finish=True)
    def trying_to_shoot(self):
        self.shooter.shoot()
        if self.shooter.has_just_fired():
            self.next_state(self.idling)

        if not self.shot_desired:
            self.next_state(self.holding_note)
