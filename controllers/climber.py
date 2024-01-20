from magicbot import StateMachine, state, will_reset_to
from components.climber import ClimberComponent


class Climber(StateMachine):
    climber_component: ClimberComponent
    button_pressed = will_reset_to(False)

    def setup(self) -> None:
        pass

    def __init__(self) -> None:
        self.button_pressed = False
        super().__init__()

    def climb(self) -> None:
        self.button_pressed = True

    @state(must_finish=True, first=True)
    def extend_hook(self, initial_call: bool) -> None:
        if initial_call:
            self.climber_component.deploy()
        if self.climber_component.has_deploy_finished() & self.button_pressed:
            self.next_state("retract_hook")

    @state(must_finish=True)
    def retract_hook(self, initial_call: bool) -> None:
        if initial_call:
            self.climber_component.retract()
        if self.climber_component.has_climb_finished:
            self.done()
