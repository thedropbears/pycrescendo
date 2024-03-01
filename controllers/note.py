from magicbot import StateMachine, state, feedback, will_reset_to

from components.intake import IntakeComponent
from controllers.shooter import Shooter


class NoteManager(StateMachine):
    shooter: Shooter
    intake: IntakeComponent

    shot_desired = will_reset_to(False)

    def __init__(self) -> None:
        self.intake_desired = False

    def try_intake(self) -> None:
        self.intake_desired = True

    def cancel_intake(self) -> None:
        self.intake_desired = False

    def try_shoot(self) -> None:
        self.shot_desired = True

    @feedback
    def has_note(self) -> bool:
        return self.intake.has_note()

    def has_just_fired(self) -> bool:
        """Intended to be polled by autonomous to tell when shooting is finished"""
        return (
            self.last_state == "holding_note"
            and self.current_state == "not_holding_note"
        )

    def execute(self) -> None:
        self.last_state = self.current_state
        super().execute()

    def on_enable(self) -> None:
        super().on_enable()
        if self.has_note():
            self.engage()
        else:
            self.engage(self.not_holding_note)

    @state(must_finish=True, first=True)
    def holding_note(self) -> None:
        self.intake_desired = False

        if self.shot_desired:
            self.shooter.engage()

        if not self.has_note():
            self.next_state(self.not_holding_note)

    @state(must_finish=True)
    def not_holding_note(self) -> None:
        if self.intake_desired:
            self.intake.deploy()
            self.intake.intake()
        else:
            self.intake.retract()

        if self.has_note():
            self.next_state(self.holding_note)
