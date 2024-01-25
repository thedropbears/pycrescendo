from __future__ import annotations

import math
import typing
import phoenix6
import phoenix6.unmanaged
import wpilib

from pyfrc.physics.core import PhysicsInterface
from wpimath.kinematics import SwerveDrive4Kinematics
from wpilib.simulation import SimDeviceSim

from components.chassis import SwerveModule

if typing.TYPE_CHECKING:
    from robot import MyRobot


class SimpleTalonFXMotorSim:
    def __init__(self, motor: phoenix6.hardware.TalonFX, kV: float) -> None:
        self.sim_state = motor.sim_state
        self.sim_state.set_supply_voltage(12.0)
        self.kV = kV  # volt seconds per unit

    def update(self, dt: float) -> None:
        voltage = self.sim_state.motor_voltage
        velocity = voltage / self.kV  # units per second
        self.sim_state.set_rotor_velocity(velocity)
        self.sim_state.add_rotor_position(velocity * dt)


class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: MyRobot):
        self.physics_controller = physics_controller

        self.kinematics: SwerveDrive4Kinematics = robot.chassis.kinematics
        self.swerve_modules: list[SwerveModule] = robot.chassis.modules

        # Motors
        self.wheels = [
            SimpleTalonFXMotorSim(
                module.drive,
                kV=2.7,
            )
            for module in robot.chassis.modules
        ]
        self.steer = [
            SimpleTalonFXMotorSim(
                module.steer,
                kV=0.01,  # TODO: get from sysid logs
            )
            for module in robot.chassis.modules
        ]

        self.imu = SimDeviceSim("navX-Sensor", 4)
        self.imu_yaw = self.imu.getDouble("Yaw")

    def update_sim(self, now: float, tm_diff: float) -> None:
        # Enable the Phoenix6 simulated devices
        # TODO: delete when phoenix6 integrates with wpilib
        if wpilib.DriverStation.isEnabled():
            phoenix6.unmanaged.feed_enable(0.1)

        for wheel in self.wheels:
            wheel.update(tm_diff)
        for steer in self.steer:
            steer.update(tm_diff)

        speeds = self.kinematics.toChassisSpeeds(
            (
                self.swerve_modules[0].get(),
                self.swerve_modules[1].get(),
                self.swerve_modules[2].get(),
                self.swerve_modules[3].get(),
            )
        )

        self.imu_yaw.set(self.imu_yaw.get() - math.degrees(speeds.omega * tm_diff))

        self.physics_controller.drive(speeds, tm_diff)
