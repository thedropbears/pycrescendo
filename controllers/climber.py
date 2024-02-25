from magicbot import StateMachine, state, will_reset_to
from components.climber import ClimberComponent


class Climber(StateMachine):
    climber_component: ClimberComponent
    should_climb = will_reset_to(False)

    def __init__(self) -> None:
        self.extended = False

    def climb(self) -> None:
        self.should_climb = True

    def deploy(self) -> None:
        self.engage()

    def stop(self) -> None:
        self.done()

    def try_toggle(self) -> None:
        if self.extended:
            self.climb()
        else:
            self.deploy()

    @state(must_finish=True, first=True)
    def extend_hook(self) -> None:
        self.extended = True
        self.climber_component.deploy()
        if self.climber_component.has_deploy_finished() and self.should_climb:
            self.next_state("retract_hook")

    @state(must_finish=True)
    def retract_hook(self) -> None:
        self.extended = False
        self.climber_component.retract()
        if self.climber_component.has_climb_finished():
            self.done()
