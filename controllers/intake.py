from magicbot import StateMachine, state, timed_state, default_state, will_reset_to
from wpilib import DriverStation

from components.intake import IntakeComponent
from components.led import LightStrip


class Intake(StateMachine):

    intake_component: IntakeComponent
    status_lights: LightStrip

    cancel_desired = will_reset_to(False)
    outtake_desired = will_reset_to(False)

    def try_cancel_intake(self) -> None:
        self.cancel_desired = True

    def try_outtake(self) -> None:
        self.outtake_desired = True

    @default_state
    def idling(self) -> None:
        if not DriverStation.isAutonomous():
            self.intake_component.retract()

    @state(first=True, must_finish=True)
    def dropping_intake(self, initial_call) -> None:
        if initial_call:
            self.status_lights.intake_deployed()
            self.intake_component.deploy()
        if self.intake_component.is_fully_deployed():
            self.next_state(self.intaking)

    @state(must_finish=True)
    def intaking(self) -> None:
        self.intake_component.intake()

        if self.intake_component.has_intake_stalled():
            self.next_state(self.unstall_intake)

        if self.outtake_desired:
            self.next_state(self.outtaking)

        if self.intake_component.has_note() or self.cancel_desired:
            self.done()

    @state(must_finish=True)
    def outtaking(self) -> None:
        self.intake_component.hover()
        self.intake_component.backdrive_intake()
        if not self.outtake_desired:
            self.done()

    @timed_state(duration=0.5, next_state="dropping_intake", must_finish=True)
    def unstall_intake(self) -> None:
        self.intake_component.hover()
        self.intake_component.backdrive_intake()
