import time
import math
import random
from abc import ABC, abstractmethod
from enum import Enum, IntEnum

import wpilib
from ids import PwmChannels


MAX_BRIGHTNESS = 50  # Integer value 0-255
HSV = tuple[int, int, int]

PULSE_SPEED = 2
BREATHE_SPEED = 4
RAINBOW_SPEED = 8
MORSE_SPEED = 0.2


class HsvColour(Enum):
    RED = (0, 255, MAX_BRIGHTNESS)
    ORANGE = (20, 255, MAX_BRIGHTNESS)
    YELLOW = (30, 255, MAX_BRIGHTNESS)
    MAGENTA = (150, 255, MAX_BRIGHTNESS)
    BLUE = (120, 255, MAX_BRIGHTNESS)
    CYAN = (90, 255, MAX_BRIGHTNESS)
    GREEN = (60, 255, MAX_BRIGHTNESS)
    WHITE = (0, 0, MAX_BRIGHTNESS)
    OFF = (0, 0, 0)


class PatternState(IntEnum):
    SOLID = 1
    PULSE = 2
    BREATHE = 3
    RAINBOW = 4
    MORSE = 5


class LightStrip:
    # QUESTION? Should lightstrip take channel as arg in init?
    #   or should this be delegated to a parent class of lightstrip and its init function
    def __init__(
        self, strip_length: int, pwm_channel: PwmChannels = PwmChannels.led_strip
    ) -> None:
        self.leds = wpilib.AddressableLED(pwm_channel)
        self.leds.setLength(strip_length)
        self.strip_length = strip_length

        self.led_data = wpilib.AddressableLED.LEDData()
        self.strip_data = [self.led_data] * strip_length

        self.pattern_state = PatternState.MORSE
        self.pattern_start_time = time.monotonic()

        self.colour: HSV = HsvColour.BLUE.value
        self.led_data.setHSV(*self.colour)
        self.leds.setData(self.strip_data)
        self.leds.start()

    def set_state(self, new_pattern: PatternState) -> None:
        self.pattern_state = new_pattern
        self.pattern_start_time = time.monotonic()

    def set_colour(self, new_colour: HsvColour) -> None:
        self.colour = new_colour

    # TODO ADD FUNCTIONS TO CALL FROM ROBOT.PY
    #   Should only call set_state & set_colour
    # Maybe should make parent of lightstrip called statuslights?
    #   statuslights will only contain *public* function calls for robot.py to call

    def execute(self) -> None:
        if self.pattern_state in PATTERN_MAP:
            colour = PATTERN_MAP[self.pattern_state].update(
                self.colour, self.pattern_start_time
            )
        else:
            colour = self.colour

        self.led_data.setHSV(*colour)
        self.leds.setData(self.strip_data)


class Pattern(ABC):
    def __init__(self, speed: float = 1) -> None:
        self.speed = speed

    @abstractmethod
    def update(self, colour: HSV, start_time: float) -> HSV:
        return


class Pulse(Pattern):
    def __init__(self) -> None:
        super().__init__(PULSE_SPEED)

    def update(self, colour: HSV, start_time: float) -> HSV:
        elapsed_time = time.monotonic() - start_time
        brightness = math.cos(self.speed * elapsed_time * math.tau) >= 0
        return (colour[0], colour[1], int(MAX_BRIGHTNESS * brightness))


class Breathe(Pattern):
    def __init__(self) -> None:
        super().__init__(BREATHE_SPEED)

    def update(self, colour: HSV, start_time: float) -> HSV:
        elapsed_time = time.monotonic() - start_time
        brightness = (math.sin(self.speed * elapsed_time * math.tau) + 1) / 2
        return (colour[0], colour[1], int(MAX_BRIGHTNESS * brightness))


class Rainbow(Pattern):
    def __init__(self) -> None:
        super().__init__(RAINBOW_SPEED)

    def update(self, colour: HSV, start_time: float) -> HSV:
        elapsed_time = time.monotonic() - start_time
        hue = round(360 * (elapsed_time / self.speed % 1))
        return (hue, colour[1], MAX_BRIGHTNESS)


class Morse(Pattern):
    # NOTE Might be better to read this data from a file?
    MESSAGES = (
        "KILL ALL HUMANS",
        "MORSE CODE IS FOR NERDS",
        "HONEYBADGER DONT CARE",
        "GLHF",
        "I HATE MORSE CODE",
    )
    MORSE_TRANSLATION = {
        "A": ".-",
        "B": "-...",
        "C": "-.-.",
        "D": "-..",
        "E": ".",
        "F": "..-.",
        "G": "--.",
        "H": "....",
        "I": "..",
        "J": ".---",
        "K": "-.-",
        "L": ".-..",
        "M": "--",
        "N": "-.",
        "O": "---",
        "P": ".--.",
        "Q": "--.-",
        "R": ".-.",
        "S": "...",
        "T": "-",
        "U": "..-",
        "V": "...-",
        "W": ".--",
        "X": "-..-",
        "Y": "-.--",
        "Z": "--..",
        "1": ".----",
        "2": "..---",
        "3": "...--",
        "4": "....-",
        "5": ".....",
        "6": "-....",
        "7": "--...",
        "8": "---..",
        "9": "----.",
        "0": "-----",
    }
    DOT_LENGTH = 1
    DASH_LENGTH = 3
    SPACE_LENGTH = 4

    def __init__(self) -> None:
        super().__init__(MORSE_SPEED)
        self.pick_new_message()

    def update(self, colour: HSV, start_time: float) -> HSV:
        elapsed_time = time.monotonic() - start_time
        if elapsed_time > self.message_time:
            return colour

        # TODO Might be better to store current token index and time?
        running_total = 0
        for token in self.morse_message:
            if token == ".":
                running_total += self.DOT_LENGTH * self.speed
            elif token == "-":
                running_total += self.DASH_LENGTH * self.speed
            elif token == " ":
                running_total += self.SPACE_LENGTH * self.speed

            # This is the current character
            if running_total > elapsed_time:
                if token == " ":
                    return HsvColour.OFF.value
                else:
                    return colour

        # Default (Should never be hit!)
        return HsvColour.OFF.value

    def pick_new_message(self) -> None:
        # QUESTION? Should functions take args or assume previous step already done
        self.message = self.random_message()
        self.morse_message = self.translate_message(self.message)
        self.message_length = self.calculate_message_length(self.morse_message)
        self.message_time = self.speed * self.message_length

    def random_message(self) -> str:
        # TODO Maybe make it not pick the same message as last time?
        return random.choice(self.MESSAGES)

    def translate_message(self, message: str) -> str:
        message = message.upper()
        morse_message = ""
        for letter in message:
            if letter == " ":
                morse_message += " "
                continue
            if letter not in self.MORSE_TRANSLATION:
                continue
            morse_message += self.MORSE_TRANSLATION[letter] + " "

        # Add some space at end of message
        morse_message += "  "
        return morse_message

    def calculate_message_length(self, morse_message: str) -> int:
        return (
            self.DOT_LENGTH * morse_message.count(".")
            + self.DASH_LENGTH * morse_message.count("-")
            + self.SPACE_LENGTH * morse_message.count(" ")
        )


# TODO Prefferably not at bottom of file :/
#   Can I make more multiple scripts for the leds?
#   Maybe in a folder?
#   - led.py (main component)
#   - led_constants.py (hsv colours, states, speeds)
#   - led_patterns.py (stores pattern classes)
PATTERN_MAP = {
    PatternState.PULSE: Pulse(),
    PatternState.BREATHE: Breathe(),
    PatternState.RAINBOW: Rainbow(),
    PatternState.MORSE: Morse(),
}
