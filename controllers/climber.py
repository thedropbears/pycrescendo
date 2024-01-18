from magicbot import StateMachine, state
from components.climb import Climber


class Climber(StateMachine):
    climber_component: Climber

    def setup(self) -> None:
        pass

    @state(first=True)
    def extend_hook(self) -> None:
        self.next_state("retract_hook")

    @state(must_finish=True)
    def retract_hook(self) -> None:
        self.done()
