from __future__ import annotations

import random
import typing

import hal
import pytest
import wpilib.simulation
from wpilib.simulation import DriverStationSim

if typing.TYPE_CHECKING:
    from pyfrc.test_support.controller import TestController


def rand_bool() -> bool:
    return random.getrandbits(1) != 0


def rand_axis() -> float:
    """Get a random number between -1 and 1."""
    return random.random() * 2 - 1


def rand_pov() -> int:
    """Pick a random POV hat value."""
    return random.choice((-1, 0, 45, 90, 135, 180, 225, 270, 315))


class AllTheThings:
    """Fuzzer for robot hardware inputs."""

    def __init__(self) -> None:
        self.dios = [
            dio
            for dio in map(wpilib.simulation.DIOSim, range(hal.getNumDigitalChannels()))
            if dio.getInitialized()
        ]

    def fuzz(self) -> None:
        for dio in self.dios:
            if dio.getIsInput():  # pragma: no branch
                dio.setValue(rand_bool())


class DSInputs:
    """Fuzzer for HIDs attached to the driver station."""

    def __init__(self) -> None:
        self.gamepad = wpilib.simulation.XboxControllerSim(0)
        self.joystick = wpilib.simulation.JoystickSim(1)

    def fuzz(self) -> None:
        fuzz_xbox_gamepad(self.gamepad)
        fuzz_joystick(self.joystick)


def fuzz_joystick(joystick: wpilib.simulation.JoystickSim) -> None:
    """Fuzz a Logitech Extreme 3D Pro flight stick."""
    for axis in range(5):
        joystick.setRawAxis(axis, rand_axis())
    for button in range(12):
        joystick.setRawButton(button, rand_bool())
    joystick.setPOV(rand_pov())


def fuzz_xbox_gamepad(gamepad: wpilib.simulation.XboxControllerSim) -> None:
    """Fuzz an XInput gamepad."""
    gamepad.setLeftX(rand_axis())
    gamepad.setLeftY(rand_axis())
    gamepad.setRightX(rand_axis())
    gamepad.setRightY(rand_axis())
    gamepad.setLeftTriggerAxis(random.random())
    gamepad.setRightTriggerAxis(random.random())
    for button in range(10):
        gamepad.setRawButton(button, rand_bool())
    gamepad.setPOV(rand_pov())


def _test_fuzz(
    control: TestController, station: hal.AllianceStationID, fuzz_disabled_hids: bool
) -> None:
    with control.run_robot():
        things = AllTheThings()
        hids = DSInputs()
        DriverStationSim.setAllianceStationId(station)

        # Disabled mode
        control.step_timing(seconds=0.2, autonomous=False, enabled=False)
        things.fuzz()
        if fuzz_disabled_hids:
            hids.fuzz()
        control.step_timing(seconds=0.2, autonomous=False, enabled=False)

        # Autonomous mode
        things.fuzz()
        control.step_timing(seconds=0.2, autonomous=True, enabled=False)
        things.fuzz()
        control.step_timing(seconds=0.2, autonomous=True, enabled=True)

        # Transition between autonomous and teleop
        things.fuzz()
        control.step_timing(seconds=0.2, autonomous=False, enabled=False)
        things.fuzz()
        control.step_timing(seconds=0.2, autonomous=False, enabled=True)

        # Teleop
        for _ in range(20):
            things.fuzz()
            hids.fuzz()
            control.step_timing(seconds=0.1, autonomous=False, enabled=True)


alliance_stations = [
    station
    for station in hal.AllianceStationID.__members__.values()
    if station != hal.AllianceStationID.kUnknown
]
alliance_station_names = [station.name[1:] for station in alliance_stations]


@pytest.mark.parametrize("station", alliance_stations, ids=alliance_station_names)
def test_fuzz(control: TestController, station: hal.AllianceStationID) -> None:
    _test_fuzz(control, station, fuzz_disabled_hids=False)


@pytest.mark.parametrize("station", alliance_stations, ids=alliance_station_names)
def test_fuzz_disabled(control: TestController, station: hal.AllianceStationID) -> None:
    _test_fuzz(control, station, fuzz_disabled_hids=True)


def test_fuzz_test(control: TestController) -> None:
    with control.run_robot():
        hids = DSInputs()

        # Start the robot in disabled mode for a short period
        control.step_timing(seconds=0.5, autonomous=False, enabled=False)

        DriverStationSim.setTest(True)
        DriverStationSim.setEnabled(True)

        assert control.robot_is_alive

        for _ in range(20):
            hids.fuzz()
            DriverStationSim.notifyNewData()
            wpilib.simulation.stepTiming(0.2)
            assert control.robot_is_alive
