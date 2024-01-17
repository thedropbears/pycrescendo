from magicbot.state_machine import AutonomousStateMachine, state

from components.chassis import Chassis
from components.climb import Climber
from components.intake import Intake
from components.shooter import Shooter


class Autonomous(AutonomousStateMachine):
    chassis: Chassis
    climb: Climber
    intake: Intake
    shooter: Shooter

    def __init__(self) -> None:
        pass

    @state(first=True)
    def initialize(self) -> None:
        pass

    @state
    def drive_to_note(self) -> None:
        pass

    @state
    def pick_up_note(self) -> None:
        pass

    @state
    def drive_to_target(self) -> None:
        pass

    @state
    def score_note(self) -> None:
        pass
