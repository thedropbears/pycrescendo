from magicbot.state_machine import AutonomousStateMachine, state

from components.chassis import Chassis

# Add controllers for intake and shooter when available

from wpimath.geometry import Pose2d

from dataclasses import dataclass


@dataclass
class NotePaths:
    pick_up_path: tuple[Pose2d, ...]
    shoot_path: tuple[Pose2d, ...]


class AutoBase(AutonomousStateMachine):
    chassis: Chassis
    # Add controllers for intake and shooter when available

    def __init__(self) -> None:
        self.note_paths: list[NotePaths] = []
        self.has_initial_note: bool = True

    @state(first=True)
    def initialise(self) -> None:
        # We always start ready to shoot, so fire straight away
        self.has_initial_note = True
        self.next_state("shoot_note")

    @state
    def shoot_note(self, initial_call: bool) -> None:
        if initial_call:
            # Call the shooter state machine
            pass

        if True:
            # This needs to check if the state machine has finished firing
            if len(self.note_paths) == 0:
                # Just shot the last note
                self.done()
            else:
                self.next_state("drive_to_pick_up")

    @state
    def drive_to_pick_up(self, initial_call: bool) -> None:
        if initial_call:
            self.calculate_trajectory(self.note_paths[0].pick_up_path)
            # Also deploy the intake

        if True:
            # Check if we have a note collected
            self.next_state("drive_to_shoot")

    @state
    def drive_to_shoot(self, initial_call) -> None:
        if initial_call:
            self.calculate_trajectory(self.note_paths[0].shoot_path)
        # Do some driving...

        if True:
            # If we are in position, remove this note from the list and shoot it
            self.note_paths.pop(0)
            self.next_state("shoot_note")

    def calculate_trajectory(self, path: tuple[Pose2d, ...]) -> None:
        # Driving to notes or to shoot is the same
        # Abstract trajectory generation into a function
        pass


class PreloadOnly(AutoBase):
    MODE_NAME = "Preload only"


class Front2Note(AutoBase):
    MODE_NAME = "Front of speaker 2 note"

    def setup(self) -> None:
        self.note_paths = [
            NotePaths(
                pick_up_path=(Pose2d(1.0, 1.0, 0.0),),
                shoot_path=(Pose2d(1.0, 1.0, 0.0),),
            )
        ]
