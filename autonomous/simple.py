from magicbot import AutonomousStateMachine, timed_state

from components.chassis import ChassisComponent
from controllers.note import NoteManager

from utilities.game import is_red


class SimpleAuto(AutonomousStateMachine):
    MODE_NAME = "DEFAULT - PLEASE SELECT A REAL MODE"
    DEFAULT = True

    chassis: ChassisComponent
    note_manager: NoteManager

    @timed_state(first=True, duration=5, next_state="drive_forward")
    def shoot_note(self) -> None:
        self.note_manager.try_shoot()

        if self.note_manager.has_just_fired():
            self.next_state(self.drive_forward)

    @timed_state(duration=4)
    def drive_forward(self):
        vx = -0.5 if is_red() else 0.5
        self.chassis.drive_field(vx, 0, 0)
