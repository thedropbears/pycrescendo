from math import atan2
from magicbot import StateMachine, state, timed_state, default_state, will_reset_to
from components.chassis import ChassisComponent
from components.shooter import ShooterComponent
from utilities.game import get_goal_speaker_position


class Shooter(StateMachine):
    chassis: ChassisComponent
    shooter_component: ShooterComponent
    should_fire = will_reset_to(False)
    SHOOTING_TIME_DURATION = 3

    def shoot(self) -> None:
        self.should_fire = True
        self.engage()

    def update_ranging(self) -> None:
        dist = (
            self.chassis.get_pose()
            .translation()
            .distance(get_goal_speaker_position().toTranslation2d())
        )
        # TODO get flywheel speeds and inclination angle from lookup table
        flywheel_target = dist
        inclination_target = 0
        self.shooter_component.set_flywheel_target(flywheel_target)
        self.shooter_component.set_inclination(inclination_target)

    @default_state
    def idle(self) -> None:
        """Run ranging whenever we are not doing anything else"""
        self.update_ranging()
        if self.should_fire:
            self.next_state("acquiring")

    @state(first=True)
    def acquiring(self) -> None:
        # NOTE Turret can't rotate, instead we face the chassis towards the goal
        # Determine heading required for goal
        translation_to_goal = (
            get_goal_speaker_position().toTranslation2d()
            - self.chassis.get_pose().translation()
        )
        bearing_to_speaker = atan2(translation_to_goal.y, translation_to_goal.x)

        # Set to appropriate heading
        self.chassis.snap_to_heading(bearing_to_speaker)

        # Update ranging but don't check for tolerance yet
        self.update_ranging()

        # Progress state machine if within tolerance
        if self.chassis.at_desired_heading():
            self.next_state("ranging")

    @state
    def ranging(self) -> None:
        # Now check ranging tolerances
        self.update_ranging()
        if self.shooter_component.is_ready():
            self.next_state("shooting")

    @timed_state(
        must_finish=True, next_state="resetting", duration=SHOOTING_TIME_DURATION
    )
    def shooting(self) -> None:
        self.shooter_component.start_injection()

    @state
    def resetting(self) -> None:
        self.shooter_component.stop_injection()
        self.done()
