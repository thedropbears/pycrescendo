from magicbot import StateMachine, state

from components.injector import InjectorComponent
from components.intake import IntakeComponent
from controllers.shooter import Shooter


class NoteManager(StateMachine):

    shooter: Shooter

    injector_component: InjectorComponent
    intake: IntakeComponent

    def __init__(self):
        pass

    def try_intake(self):
        pass

    def cancel_intake(self):
        pass

    def try_shoot(self):
        pass

    @state(first=True, must_finish=True)
    def idling(self):
        pass

    @state(must_finish=True)
    def intaking(self):
        pass

    @state(must_finish=True)
    def holding_note(self):
        pass

    @state(must_finish=True)
    def trying_to_shoot(self):
        pass
