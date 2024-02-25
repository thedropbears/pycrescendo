from __future__ import annotations

import math
import typing

import phoenix6
import phoenix6.unmanaged
import wpilib
from pyfrc.physics.core import PhysicsInterface
from wpilib.simulation import DCMotorSim, SimDeviceSim
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.system.plant import DCMotor
from wpimath.units import kilogram_square_meters

from components.chassis import SwerveModule
from components.shooter import ShooterComponent

if typing.TYPE_CHECKING:
    from robot import MyRobot


class SimpleTalonFXMotorSim:
    def __init__(
        self, motor: phoenix6.hardware.TalonFX, units_per_rev: float, kV: float
    ) -> None:
        self.sim_state = motor.sim_state
        self.sim_state.set_supply_voltage(12.0)
        self.kV = kV  # volt seconds per unit
        self.units_per_rev = units_per_rev

    def update(self, dt: float) -> None:
        voltage = self.sim_state.motor_voltage
        velocity = voltage / self.kV  # units per second
        velocity_rps = velocity * self.units_per_rev
        self.sim_state.set_rotor_velocity(velocity_rps)
        self.sim_state.add_rotor_position(velocity_rps * dt)


class Falcon500MotorSim:
    def __init__(
        self,
        *motors: phoenix6.hardware.TalonFX,
        # Reduction between motor and encoder readings, as output over input.
        # If the mechanism spins slower than the motor, this number should be greater than one.
        gearing: float,
        moi: kilogram_square_meters,
    ):
        self.gearing = gearing
        self.sim_states = [motor.sim_state for motor in motors]
        for sim_state in self.sim_states:
            sim_state.set_supply_voltage(12.0)
        self.motor_sim = DCMotorSim(DCMotor.falcon500(len(motors)), gearing, moi)

    def update(self, dt: float) -> None:
        voltage = self.sim_states[0].motor_voltage
        self.motor_sim.setInputVoltage(voltage)
        self.motor_sim.update(dt)
        motor_rev_per_mechanism_rad = self.gearing / math.tau
        for sim_state in self.sim_states:
            sim_state.set_raw_rotor_position(
                self.motor_sim.getAngularPosition() * motor_rev_per_mechanism_rad
            )
            sim_state.set_rotor_velocity(
                self.motor_sim.getAngularVelocity() * motor_rev_per_mechanism_rad
            )


class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: MyRobot):
        self.physics_controller = physics_controller

        self.kinematics: SwerveDrive4Kinematics = robot.chassis.kinematics
        self.swerve_modules: list[SwerveModule] = robot.chassis.modules

        # Motors
        self.wheels = [
            SimpleTalonFXMotorSim(
                module.drive,
                units_per_rev=1 / module.DRIVE_MOTOR_REV_TO_METRES,
                kV=2.7,
            )
            for module in robot.chassis.modules
        ]
        self.steer = [
            Falcon500MotorSim(
                module.steer,
                gearing=1 / module.STEER_GEAR_RATIO,
                # measured from MKCad CAD
                moi=0.0009972,
            )
            for module in robot.chassis.modules
        ]

        g_mm2_to_kg_m2 = 1e-9
        # The following moments of inertia were measured from the CAD on 2024-02-25.
        single_wheel_moi = 106304.0223 * g_mm2_to_kg_m2
        tiny_shafts_moi = 2 * 921.70073 * g_mm2_to_kg_m2
        inverted_shafts_moi = 2 * (2516.60792 + 921.70073) * g_mm2_to_kg_m2
        motor_pulleys_moi = (1115.67156 + 2782.3926) * g_mm2_to_kg_m2
        motor_linkage_shafts_moi = 2 * 1126.02215 * g_mm2_to_kg_m2
        top_roller_shaft_moi = 3291.67688 * g_mm2_to_kg_m2
        inversing_shaft_moi = 8179.57108 * g_mm2_to_kg_m2
        inversing_gears_moi = 4 * 42065.7167 * g_mm2_to_kg_m2

        self.flywheel = Falcon500MotorSim(
            robot.shooter_component.flywheel_left,
            robot.shooter_component.flywheel_right,
            gearing=ShooterComponent.FLYWHEEL_GEAR_RATIO,
            moi=(
                8 * single_wheel_moi
                + tiny_shafts_moi
                + inverted_shafts_moi
                + 2 * motor_pulleys_moi
                + motor_linkage_shafts_moi
                + top_roller_shaft_moi
                + inversing_shaft_moi
                + inversing_gears_moi
            ),
        )

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

        self.flywheel.update(tm_diff)

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
