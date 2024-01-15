from magicbot import StateMachine, state
from components.chassis import Chassis

from utilities.game import get_goal_speaker_position

from math import atan2


class Shooter(StateMachine):
    chassis: Chassis

    def setup(self) -> None:
        pass

    @state(first=True)
    def acquiring(self) -> None:
        # determine heading required for goal
        translation_to_goal = (
            get_goal_speaker_position().toTranslation2d()
            - self.chassis.get_pose().translation()
        )
        bearing_to_speaker = atan2(translation_to_goal.y, translation_to_goal.x)
        # set to appropriate heading
        self.chassis.snap_to_heading(bearing_to_speaker)

        # progress state machine if within tolerance
        if self.chassis.at_desired_heading():
            self.next_state("shooting")

    @state(must_finish=True)
    def shooting(self) -> None:
        # commence shooting action
        self.done()
