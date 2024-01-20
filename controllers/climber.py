from magicbot import StateMachine, state
from components.climber import ClimberComponent


class Climber(StateMachine):
    climber_component: ClimberComponent

    def setup(self) -> None:
        pass

    @state(must_finish=True, first=True)
    def extend_hook(self, initial_call: bool) -> None:
        if initial_call:
            self.climber_component.deploy()
        if self.climber_component.has_deploy_finished():
            self.next_state("retract_hook")

    @state(must_finish=True)
    def retract_hook(self, initial_call: bool) -> None:
        if initial_call:
            self.climber_component.retract()
        if self.climber_component.has_climbed_finished:
            self.done()
