from magicbot import StateMachine, state, feedback, will_reset_to
import wpilib

from components.intake import IntakeComponent
from components.led import LightStrip
from controllers.shooter import Shooter
from controllers.intake import Intake


class NoteManager(StateMachine):
    shooter: Shooter
    intake_component: IntakeComponent
    intake: Intake
    status_lights: LightStrip

    shot_desired = will_reset_to(False)
    intake_desired = will_reset_to(False)
    cancel_intake_desired = will_reset_to(False)
    jettison_desired = will_reset_to(False)

    def __init__(self) -> None:
        self.last_state = ""

    def try_intake(self) -> None:
        self.intake_desired = True

    def try_cancel_intake(self) -> None:
        self.cancel_intake_desired = True

    def try_shoot(self) -> None:
        self.shot_desired = True

    def jettison(self) -> None:
        self.jettison_desired = True

    @feedback
    def has_note(self) -> bool:
        return self.intake_component.has_note()

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
        self.last_state = ""
        if self.has_note() or wpilib.DriverStation.isAutonomous():
            self.engage()
        else:
            self.engage(self.not_holding_note)

    @state(must_finish=True, first=True)
    def holding_note(self) -> None:
        self.shooter.update_range()

        if self.jettison_desired:
            self.next_state(self.outtaking)
            return

        if self.shooter.in_range():
            self.status_lights.in_range()
            if self.shot_desired:
                self.shooter.engage()
        else:
            self.status_lights.not_in_range()

        if not self.has_note():
            self.next_state(self.not_holding_note)

    @state(must_finish=True)
    def not_holding_note(self, initial_call) -> None:
        if initial_call:
            self.status_lights.no_note()

        if self.jettison_desired:
            self.next_state(self.outtaking)
            return

        self.shooter.coast_down()
        if self.intake_desired:
            self.shooter.update_range()
            self.intake.engage()

        elif self.cancel_intake_desired:
            self.intake.try_cancel_intake()

        elif self.has_note():
            self.next_state(self.holding_note)

    @state(must_finish=True)
    def outtaking(self) -> None:
        self.intake.try_outtake()
        self.shooter.try_jettison()

        if not self.jettison_desired:
            if self.has_note():
                self.next_state(self.holding_note)
            else:
                self.next_state(self.not_holding_note)
