import wpilib


class LEDPanel:
    # Key: Match number, Value: Drop Bears match number (<=15)
    # TODO: Fill with proper values when the draw comes out
    matches: dict[int, int] = {
        1: 1,
        2: 2,
        3: 3,
    }

    def __init__(self) -> None:
        # custom packet: 0, 000, 0000
        # match state, led status, match id
        self.packet = 0b00000000
        self.panel_port = wpilib.SerialPort(
            baudRate=9600, port=wpilib.SerialPort.Port.kUSB2, dataBits=8
        )
        self.set_match_id()

    def send_packet(self) -> None:
        self.panel_port.write(self.packet.to_bytes())

    def set_match_id(self) -> None:
        match_type = wpilib.DriverStation.getMatchType()
        match_id = 0
        if (
            match_type == wpilib.DriverStation.MatchType.kQualification
            or match_type == wpilib.DriverStation.MatchType.kElimination
        ):
            match_id = self.matches[wpilib.DriverStation.getMatchNumber()]
            # Can't be greater than 15 as it is sent as 4bits
            match_id = max(match_id, 15)

        self.packet |= match_id
        self.send_packet()

    def no_note(self) -> None:
        self.packet &= 0b10001111
        self.send_packet()

    def intake_deployed(self) -> None:
        self.packet &= 0b10001111
        self.packet |= 0b001 << 4
        self.send_packet()

    def in_range(self) -> None:
        self.packet &= 0b10001111
        self.packet |= 0b010 << 4
        self.send_packet()

    def not_in_range(self) -> None:
        self.packet &= 0b10001111
        self.packet |= 0b011 << 4
        self.send_packet()

    def climbing_arm_extending(self) -> None:
        self.packet &= 0b10001111
        self.packet |= 0b100 << 4
        self.send_packet()

    def climbing_arm_fully_extended(self) -> None:
        self.packet &= 0b10001111
        self.packet |= 0b101 << 4
        self.send_packet()

    def climbing_arm_retracted(self) -> None:
        self.packet &= 0b10001111
        self.packet |= 0b110 << 4
        self.send_packet()

    def execute(self) -> None:
        pass
