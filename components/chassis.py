from logging import Logger
import math
from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.controls import VoltageOut, VelocityVoltage, PositionDutyCycle
from phoenix6.signals import NeutralModeValue
from phoenix6.configs import (
    config_groups,
    MotorOutputConfigs,
    FeedbackConfigs,
    Slot0Configs,
)
import magicbot
import navx
import wpilib
from wpimath.kinematics import (
    SwerveDrive4Kinematics,
    ChassisSpeeds,
    SwerveModuleState,
    SwerveModulePosition,
)
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.controller import SimpleMotorFeedforwardMeters
from wpimath.trajectory import TrapezoidProfileRadians
from wpimath.controller import ProfiledPIDControllerRadians

from magicbot import feedback

from utilities.functions import constrain_angle, rate_limit_module
from utilities.ctre import FALCON_FREE_RPS
from ids import CancoderIds, TalonIds


class SwerveModule:
    DRIVE_GEAR_RATIO = (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0)
    STEER_GEAR_RATIO = (14 / 50) * (10 / 60)
    WHEEL_CIRCUMFERENCE = 4 * 2.54 / 100 * math.pi

    DRIVE_MOTOR_REV_TO_METRES = WHEEL_CIRCUMFERENCE * DRIVE_GEAR_RATIO
    STEER_MOTOR_REV_TO_RAD = math.tau * STEER_GEAR_RATIO

    # limit the acceleration of the commanded speeds of the robot to what is actually
    # achiveable without the wheels slipping. This is done to improve odometry
    accel_limit = 15  # m/s^2

    def __init__(
        self,
        x: float,
        y: float,
        drive_id: int,
        steer_id: int,
        encoder_id: int,
        steer_reversed=True,
        drive_reversed=True,
    ):
        """
        x, y: where the module is relative to the center of the robot
        *_id: can ids of steer and drive motors and absolute encoder
        """
        self.translation = Translation2d(x, y)
        self.state = SwerveModuleState(0, Rotation2d(0))
        self.do_smooth = True

        if drive_reversed:
            drive_reversed = config_groups.InvertedValue.CLOCKWISE_POSITIVE
        else:
            drive_reversed = config_groups.InvertedValue.COUNTER_CLOCKWISE_POSITIVE

        if steer_reversed:
            steer_reversed = config_groups.InvertedValue.CLOCKWISE_POSITIVE
        else:
            steer_reversed = config_groups.InvertedValue.COUNTER_CLOCKWISE_POSITIVE

        # Create Motor and encoder objects
        self.steer = TalonFX(steer_id)
        self.drive = TalonFX(drive_id)
        self.drive_id = drive_id
        self.encoder = CANcoder(encoder_id)

        # Reduce CAN status frame rates before configuring
        self.steer.get_fault_field().set_update_frequency(
            frequency_hz=4, timeout_seconds=0.01
        )
        self.drive.get_fault_field().set_update_frequency(
            frequency_hz=4, timeout_seconds=0.01
        )

        # Configure steer motor
        steer_config = self.steer.configurator

        steer_motor_config = MotorOutputConfigs()
        steer_motor_config.neutral_mode = NeutralModeValue.BRAKE
        steer_motor_config.inverted = steer_reversed

        steer_gear_ratio_config = FeedbackConfigs().with_sensor_to_mechanism_ratio(
            1 / self.STEER_GEAR_RATIO
        )

        # configuration for motor pid
        steer_pid = Slot0Configs().with_k_p(0.3409939393939394).with_k_i(0).with_k_d(0)

        steer_config.apply(steer_motor_config)
        steer_config.apply(steer_pid, 0.01)
        steer_config.apply(steer_gear_ratio_config)

        # Configure drive motor
        drive_config = self.drive.configurator

        drive_motor_config = MotorOutputConfigs()
        drive_motor_config.neutral_mode = NeutralModeValue.BRAKE
        drive_motor_config.inverted = drive_reversed

        drive_gear_ratio_config = FeedbackConfigs().with_sensor_to_mechanism_ratio(
            1 / self.DRIVE_GEAR_RATIO
        )

        # configuration for motor pid and feedforward
        self.drive_pid = (
            Slot0Configs().with_k_p(0.026450530596285438).with_k_i(0).with_k_d(0)
        )
        self.drive_ff = SimpleMotorFeedforwardMeters(kS=0.18877, kV=2.7713, kA=0.18824)

        drive_config.apply(drive_motor_config)
        drive_config.apply(self.drive_pid, 0.01)
        drive_config.apply(drive_gear_ratio_config)

        self.central_angle = math.atan2(x, y)
        self.module_locked = False

    def get_angle_absolute(self) -> float:
        """Gets steer angle (rot) from absolute encoder"""
        return self.encoder.get_absolute_position().value

    def get_angle_integrated(self) -> float:
        """Gets steer angle from motor's integrated relative encoder"""
        return self.steer.get_position().value * math.tau

    def get_rotation(self) -> Rotation2d:
        """Get the steer angle as a Rotation2d"""
        return Rotation2d(self.get_angle_integrated())

    def get_drive_current(self) -> float:
        return self.drive.get_stator_current().value

    def get_speed(self) -> float:
        # velocity is in rot/s, return in m/s
        return self.drive.get_velocity().value * self.WHEEL_CIRCUMFERENCE

    def get_distance_traveled(self) -> float:
        return self.drive.get_position().value * self.WHEEL_CIRCUMFERENCE

    def set(self, desired_state: SwerveModuleState):
        if self.module_locked:
            desired_state = SwerveModuleState(0, Rotation2d(self.central_angle))

        # smooth wheel velocity vector
        if self.do_smooth:
            self.state = rate_limit_module(self.state, desired_state, self.accel_limit)
        else:
            self.state = desired_state
        self.state = SwerveModuleState.optimize(self.state, self.get_rotation())

        self.drive_request = VelocityVoltage(0)
        self.steer_request = VoltageOut(0)
        if abs(self.state.speed) < 0.01 and not self.module_locked:
            self.drive.set_control(self.drive_request.with_velocity(0))
            self.steer.set_control(self.steer_request)
            return

        current_angle = self.get_angle_integrated()
        target_displacement = constrain_angle(
            self.state.angle.radians() - current_angle
        )
        target_angle = target_displacement + current_angle
        self.steer_request = PositionDutyCycle(target_angle / math.tau)
        self.steer.set_control(self.steer_request)

        # rescale the speed target based on how close we are to being correctly aligned
        target_speed = self.state.speed * math.cos(target_displacement) ** 2
        speed_volt = self.drive_ff.calculate(target_speed)

        # original position change/100ms, new m/s -> rot/s
        self.drive.set_control(
            self.drive_request.with_velocity(
                target_speed / self.WHEEL_CIRCUMFERENCE
            ).with_feed_forward(speed_volt)
        )

    #
    def sync_steer_encoders(self) -> None:
        self.steer.set_position(self.get_angle_absolute())

    def get_position(self) -> SwerveModulePosition:
        return SwerveModulePosition(self.get_distance_traveled(), self.get_rotation())

    def get(self) -> SwerveModuleState:
        return SwerveModuleState(self.get_speed(), self.get_rotation())


class Chassis:
    # metres between centre of left and right wheels
    TRACK_WIDTH = 0.61
    # metres between centre of front and back wheels
    WHEEL_BASE = 0.61

    # size including bumpers
    LENGTH = 1.0105
    WIDTH = 0.8705
    DRIVE_CURRENT_THRESHOLD = 35
    # maxiumum speed for any wheel
    max_wheel_speed = FALCON_FREE_RPS * SwerveModule.DRIVE_MOTOR_REV_TO_METRES

    control_loop_wait_time: float

    chassis_speeds = magicbot.will_reset_to(ChassisSpeeds(0, 0, 0))
    field: wpilib.Field2d
    logger: Logger

    send_modules = magicbot.tunable(False)
    do_fudge = magicbot.tunable(True)
    do_smooth = magicbot.tunable(True)
    swerve_lock = magicbot.tunable(False)

    def setup(self) -> None:
        self.imu = navx.AHRS.create_spi()
        self.heading_controller = ProfiledPIDControllerRadians(
            1, 0, 0, TrapezoidProfileRadians.Constraints(2, 2)
        )
        self.heading_controller.enableContinuousInput(-math.pi, math.pi)
        self.align_to_setpoint = False

        self.modules = [
            # Front Left
            SwerveModule(
                self.WHEEL_BASE / 2,
                self.TRACK_WIDTH / 2,
                TalonIds.drive_1,
                TalonIds.steer_1,
                CancoderIds.swerve_1,
            ),
            # Back Left
            SwerveModule(
                -self.WHEEL_BASE / 2,
                self.TRACK_WIDTH / 2,
                TalonIds.drive_2,
                TalonIds.steer_2,
                CancoderIds.swerve_2,
            ),
            # Back Right
            SwerveModule(
                -self.WHEEL_BASE / 2,
                -self.TRACK_WIDTH / 2,
                TalonIds.drive_3,
                TalonIds.steer_3,
                CancoderIds.swerve_3,
            ),
            # Front Right
            SwerveModule(
                self.WHEEL_BASE / 2,
                -self.TRACK_WIDTH / 2,
                TalonIds.drive_4,
                TalonIds.steer_4,
                CancoderIds.swerve_4,
            ),
        ]

        self.kinematics = SwerveDrive4Kinematics(
            self.modules[0].translation,
            self.modules[1].translation,
            self.modules[2].translation,
            self.modules[3].translation,
        )
        self.sync_all()
        self.imu.zeroYaw()
        self.imu.resetDisplacement()
        self.estimator = SwerveDrive4PoseEstimator(
            self.kinematics,
            self.imu.getRotation2d(),
            self.get_module_positions(),
            Pose2d(3, 0, 0),
            stateStdDevs=(0.05, 0.05, 0.01),
            visionMeasurementStdDevs=(0.4, 0.4, math.inf),
        )
        self.field_obj = self.field.getObject("fused_pose")
        self.module_objs: list[wpilib.FieldObject2d] = []
        for idx, _module in enumerate(self.modules):
            self.module_objs.append(self.field.getObject("s_module_" + str(idx)))
        self.set_pose(Pose2d(4, 3.5, Rotation2d.fromDegrees(180)))

    def drive_field(self, vx: float, vy: float, omega: float) -> None:
        """Field oriented drive commands"""
        current_heading = self.get_rotation()
        self.chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            vx, vy, omega, current_heading
        )

    def drive_local(self, vx: float, vy: float, omega: float) -> None:
        """Robot oriented drive commands"""
        self.chassis_speeds = ChassisSpeeds(vx, vy, omega)

    def setpoint_rotation_diff(self, setpoint: Pose2d) -> float:
        """return omega velocity for alignment to the given setpoint"""
        cur_pose = self.estimator.getEstimatedPosition()
        pose_diff_y = setpoint.y - cur_pose.y
        pose_diff_x = setpoint.x - cur_pose.x
        angle = math.atan2(pose_diff_y, pose_diff_x)
        if pose_diff_y < 0:
            angle = angle + math.tau

        angle = math.pi / 2 - angle
        self.heading_diff = angle - cur_pose.rotation().radians()

    def execute(self) -> None:
        # rotate desired velocity to compensate for skew caused by discretization
        # see https://www.chiefdelphi.com/t/field-relative-swervedrive-drift-even-with-simulated-perfect-modules/413892/

        if self.align_to_setpoint:
            self.chassis_speeds.omega = self.heading_controller.calculate(
                self.heading_diff
            )

        if self.heading_controller.atGoal():
            self.align_to_setpoint = False
            self.heading_diff = 0

        if self.do_fudge:
            # in the sim i found using 5 instead of 0.5 did a lot better
            desired_speed_translation = Translation2d(
                self.chassis_speeds.vx, self.chassis_speeds.vy
            ).rotateBy(
                Rotation2d(-self.chassis_speeds.omega * 5 * self.control_loop_wait_time)
            )
            desired_speeds = ChassisSpeeds(
                desired_speed_translation.x,
                desired_speed_translation.y,
                self.chassis_speeds.omega,
            )
        else:
            desired_speeds = self.chassis_speeds

        if self.swerve_lock:
            self.do_smooth = False

        desired_states = self.kinematics.toSwerveModuleStates(desired_speeds)
        desired_states = self.kinematics.desaturateWheelSpeeds(
            desired_states, attainableMaxSpeed=self.max_wheel_speed
        )

        for state, module in zip(desired_states, self.modules):
            module.module_locked = self.swerve_lock
            module.do_smooth = self.do_smooth
            module.set(state)

        self.update_odometry()

    def on_enable(self) -> None:
        # update the odometry so the pose estimator dosent have an empty buffer
        self.update_odometry()

    @magicbot.feedback
    def get_imu_speed(self) -> float:
        return math.hypot(self.imu.getVelocityX(), self.imu.getVelocityY())

    def lock_swerve(self) -> None:
        self.swerve_lock = True

    def unlock_swerve(self) -> None:
        self.swerve_lock = False

    def get_velocity(self) -> ChassisSpeeds:
        """Gets field relative measured robot ChassisSpeeds"""
        self.local_speed = self.kinematics.toChassisSpeeds(
            (
                self.modules[0].get(),
                self.modules[1].get(),
                self.modules[2].get(),
                self.modules[3].get(),
            )
        )
        return ChassisSpeeds.fromFieldRelativeSpeeds(
            self.local_speed, -self.get_rotation()
        )

    def update_odometry(self) -> None:
        self.estimator.update(self.imu.getRotation2d(), self.get_module_positions())
        self.field_obj.setPose(self.get_pose())
        if self.send_modules:
            robot_location = self.estimator.getEstimatedPosition()
            for idx, module in enumerate(self.modules):
                module_location = (
                    robot_location.translation()
                    + module.translation.rotateBy(robot_location.rotation())
                )
                module_rotation = module.get_rotation().rotateBy(
                    robot_location.rotation()
                )
                self.module_objs[idx].setPose(Pose2d(module_location, module_rotation))

    def sync_all(self) -> None:
        for m in self.modules:
            m.sync_steer_encoders()

    def set_pose(self, pose: Pose2d) -> None:
        self.estimator.resetPosition(
            self.imu.getRotation2d(), self.get_module_positions(), pose
        )
        self.field.setRobotPose(pose)
        self.field_obj.setPose(pose)

    def zero_yaw(self) -> None:
        """Sets pose to current pose but with a heading of zero"""
        cur_pose = self.estimator.getEstimatedPosition()
        self.estimator.resetPosition(
            self.imu.getRotation2d(),
            self.get_module_positions(),
            Pose2d(cur_pose.translation(), Rotation2d(0)),
        )

    def get_module_positions(
        self,
    ) -> tuple[
        SwerveModulePosition,
        SwerveModulePosition,
        SwerveModulePosition,
        SwerveModulePosition,
    ]:
        return (
            self.modules[0].get_position(),
            self.modules[1].get_position(),
            self.modules[2].get_position(),
            self.modules[3].get_position(),
        )

    def get_pose(self) -> Pose2d:
        """Get the current location of the robot relative to ???"""
        return self.estimator.getEstimatedPosition()

    def get_rotation(self) -> Rotation2d:
        """Get the current heading of the robot."""
        return self.get_pose().rotation()

    @feedback
    def get_tilt(self) -> float:
        return math.radians(self.imu.getRoll())

    @feedback
    def get_tilt_rate(self) -> float:
        return math.radians(self.imu.getRawGyroY())

    @feedback
    def get_drive_current(self) -> float:
        return sum(abs(x.get_drive_current()) for x in self.modules)

    @feedback
    def may_be_stalled(self) -> bool:
        return self.get_drive_current() > self.DRIVE_CURRENT_THRESHOLD
