from magicbot import StateMachine, state, feedback, will_reset_to
import wpilib

from components.intake import IntakeComponent
from components.led import LightStrip
from controllers.shooter import Shooter


class NoteManager(StateMachine):
    shooter: Shooter
    intake: IntakeComponent
    status_lights: LightStrip

    shot_desired = will_reset_to(False)

    def __init__(self) -> None:
        self.intake_desired = False
        self.last_state = ""

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
        self.last_state = ""
        if self.has_note() or wpilib.DriverStation.isAutonomous():
            self.engage()
        else:
            self.engage(self.not_holding_note)

    @state(must_finish=True, first=True)
    def holding_note(self) -> None:
        self.intake_desired = False
        self.shooter.update_range()

        if self.shooter.in_range():
            self.status_lights.in_range()
        else:
            self.status_lights.not_in_range()

        if not wpilib.DriverStation.isAutonomous():
            self.intake.retract()

        if self.shot_desired:
            self.shooter.engage()

        if not self.has_note():
            self.next_state(self.not_holding_note)

    @state(must_finish=True)
    def not_holding_note(self, initial_call) -> None:
        if initial_call:
            self.status_lights.no_note()

        self.shooter.coast_down()
        if self.intake_desired:
            self.shooter.update_range()
            self.intake.deploy()
            self.intake.intake()
            # NOTE: Flash won't work cause it is called every tick
            self.status_lights.intake_deployed()
        elif not wpilib.DriverStation.isAutonomous():
            self.intake.retract()

        if self.has_note():
            self.next_state(self.holding_note)
