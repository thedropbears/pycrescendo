class ClimberComponent:
    def __init__(self) -> None:
        self.deployed = False

    def deploy(self) -> None:
        self.deployed = True

    def execute(self) -> None:
        if self.deployed:
            # Deploy the climber
            pass
        else:
            pass
