from magicbot import StateMachine, state, will_reset_to
from components.climber import ClimberComponent


class Climber(StateMachine):
    climber_component: ClimberComponent
    should_climb = will_reset_to(False)

    def climb(self) -> None:
        self.should_climb = True

    def deploy(self) -> None:
        self.engage()

    @state(must_finish=True, first=True)
    def extend_hook(self, initial_call: bool) -> None:
        if initial_call:
            self.climber_component.deploy()
        if self.climber_component.has_deploy_finished() and self.button_pressed:
            self.next_state("retract_hook")

    @state(must_finish=True)
    def retract_hook(self, initial_call: bool) -> None:
        if initial_call:
            self.climber_component.retract()
        if self.climber_component.has_climb_finished():
            self.done()
