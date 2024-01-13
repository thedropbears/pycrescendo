from magicbot import tunable


class Motor:
    shoot_speed = tunable(1.0)  # speed is tunable via NetworkTables

    def __init__(self):
        self.deployed = False

    def deploy(self):
        self.deployed = True

    def is_ready(self):
        # Check if the motor is ready to shoot
        return True

    def is_note_present(self):
        # Check if the note is in the intake
        return False

    def is_fully_retracted(self):
        # Check if the intake is fully retracted
        return True

    def is_fully_deployed(self):
        # Check if the intake is fully deployed
        return False

    def execute(self):
        if self.deployed:
            # set the motor to the shoot_speed
            pass
        else:
            # set the motor speed to 0
            pass
        self.deployed = False
