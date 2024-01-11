from magicbot import tunable, feedback


class Intake:
    intake_speed = tunable(1.0)

    def __init__(self) -> None:
        pass

    def deploy(self) -> None:
        pass

    def retract(self) -> None:
        pass

    def run_backwards(self) -> None:
        pass

    @feedback
    def is_note_present(self) -> bool:
        pass

    @feedback
    def is_fully_retracted(self) -> bool:
        pass

    @feedback
    def is_fully_deployed(self) -> bool:
        pass

    def execute(self) -> None:
        pass
