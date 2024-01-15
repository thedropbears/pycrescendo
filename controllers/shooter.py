from magicbot import StateMachine, state
from components.chassis import Chassis


class Shooter(StateMachine):
    chassis: Chassis

    def setup(self) -> None:
        pass

    @state(first=True)
    def acquiring(self) -> None:
        # determine heading required for goal

        # set to appropriate heading

        # progress state machine if within tolerance
        if self.chassis.at_desired_heading():
            self.next_state("shooting")

    @state(must_finish=True)
    def shooting(self) -> None:
        # commence shooting action
        self.done()
