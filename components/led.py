import dataclasses
import time
import math
import random
from abc import ABC, abstractmethod
from enum import Enum
from typing import Callable, Protocol

import wpilib
from ids import PwmChannels


MAX_BRIGHTNESS = 50  # Integer value 0-255
Hsv = tuple[int, int, int]

FLASH_SPEED = 2
BREATHE_SPEED = 0.5
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

    def with_hue(self, hue: int) -> Hsv:
        """
        Change the hue of the colour.

        Args:
            hue: The desired hue in [0,180).
        """
        _, s, v = self.value
        return (hue, s, v)

    def with_relative_brightness(self, multiplier: float) -> Hsv:
        """
        Scale the brightness of the colour.

        `multiplier` MUST be non-negative, and SHOULD be <= 1.
        """
        h, s, v = self.value
        return (h, s, int(v * multiplier))


class LightStrip:
    def __init__(self, strip_length: int) -> None:
        self.leds = wpilib.AddressableLED(PwmChannels.led_strip)
        self.leds.setLength(strip_length)
        self.strip_length = strip_length

        self.led_data = wpilib.AddressableLED.LEDData()
        self.strip_data = [self.led_data] * strip_length

        self.pattern: Pattern = Rainbow(HsvColour.MAGENTA)
        self.high_priority_pattern: Pattern | None = None

        self.leds.setData(self.strip_data)
        self.leds.start()

    def no_note(self) -> None:
        self.pattern = Solid(HsvColour.OFF)

    def intake_deployed(self) -> None:
        self.pattern = Flash(HsvColour.MAGENTA)

    def in_range(self) -> None:
        self.pattern = Solid(HsvColour.GREEN)

    def not_in_range(self) -> None:
        self.pattern = Solid(HsvColour.RED)

    def climbing_arm_extending(self) -> None:
        self.high_priority_pattern = Flash(HsvColour.YELLOW)

    def climbing_arm_fully_extended(self) -> None:
        self.high_priority_pattern = Solid(HsvColour.YELLOW)

    def climbing_arm_retracted(self) -> None:
        self.high_priority_pattern = None

    def morse(self) -> None:
        if not isinstance(self.pattern, Morse):
            self.pattern = Morse(HsvColour.ORANGE)

    def rainbow(self) -> None:
        self.pattern = Rainbow(HsvColour.RED)

    def invalid_start(self) -> None:
        self.pattern = Breathe(HsvColour.RED)

    def missing_start_pose(self) -> None:
        self.pattern = Breathe(HsvColour.CYAN)

    def no_vision(self) -> None:
        self.pattern = Breathe(HsvColour.ORANGE)

    def too_close_to_stage(self) -> None:
        self.pattern = Breathe(HsvColour.MAGENTA)

    def disabled(self) -> None:
        self.pattern = Solid(HsvColour.OFF)

    def execute(self) -> None:
        if self.high_priority_pattern is None:
            colour = self.pattern.update()
        else:
            colour = self.high_priority_pattern.update()
        self.led_data.setHSV(*colour)
        self.leds.setData(self.strip_data)


class Pattern(Protocol):
    def update(self) -> Hsv: ...


@dataclasses.dataclass
class Solid(Pattern):
    colour: HsvColour

    def update(self) -> Hsv:
        return self.colour.value


@dataclasses.dataclass
class TimeBasedPattern(ABC, Pattern):
    colour: HsvColour
    clock: Callable[[], float] = time.monotonic

    @abstractmethod
    def update(self) -> Hsv: ...


@dataclasses.dataclass
class Flash(TimeBasedPattern):
    speed: float = FLASH_SPEED

    def update(self) -> Hsv:
        brightness = math.cos(self.speed * self.clock() * math.tau) >= 0
        return self.colour.with_relative_brightness(brightness)


@dataclasses.dataclass
class Breathe(TimeBasedPattern):
    speed: float = BREATHE_SPEED

    def update(self) -> Hsv:
        brightness = (math.sin(self.speed * self.clock() * math.tau) + 1) / 2
        return self.colour.with_relative_brightness(brightness)


@dataclasses.dataclass
class Rainbow(TimeBasedPattern):
    speed: float = RAINBOW_SPEED

    def update(self) -> Hsv:
        hue = round(360 * (self.clock() / self.speed % 1))
        return self.colour.with_hue(hue)


@dataclasses.dataclass(eq=False)
class Morse(TimeBasedPattern):
    speed: float = MORSE_SPEED
    start_time: float = dataclasses.field(init=False)

    # NOTE Might be better to read this data from a file?
    MESSAGES = (
        "KILL ALL HUMANS",
        "MORSE CODE IS FOR NERDS",
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

    def __post_init__(self) -> None:
        self.pick_new_message()
        self.start_clock()

    def start_clock(self) -> None:
        self.start_time = self.clock()

    def elapsed_time(self) -> float:
        return self.clock() - self.start_time

    def update(self) -> Hsv:
        elapsed_time = self.elapsed_time()

        if elapsed_time > self.message_time:
            # End of message, repeat the message
            self.start_clock()
            return HsvColour.OFF.value

        # TODO Might be better to store current token index and time?
        running_total = 0.0
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
                    return self.colour.value

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

    @classmethod
    def translate_message(cls, message: str) -> str:
        message = message.upper()
        morse_message = ""
        for letter in message:
            if letter == " ":
                morse_message += " "
                continue
            morse_message += cls.MORSE_TRANSLATION[letter] + " "

        # Add some space at end of message
        morse_message += "    "
        return morse_message

    @classmethod
    def calculate_message_length(cls, morse_message: str) -> int:
        return (
            cls.DOT_LENGTH * morse_message.count(".")
            + cls.DASH_LENGTH * morse_message.count("-")
            + cls.SPACE_LENGTH * morse_message.count(" ")
        )
