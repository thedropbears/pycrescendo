import wpilib
from magicbot import feedback
from rev import CANSparkMax
from ids import SparkMaxIds, DioChannels


class ClimberComponent:
    GEAR_RATIO = 1 / 48  # using a Neo with 4:1 4:1 3:1 ratio
    MOTOR_REV_TOP_LIMIT = 7.110515
    MOTOR_REV_BOTTOM_LIMIT = 0

    def __init__(self) -> None:
        self.climbing_motor = CANSparkMax(
            SparkMaxIds.climber, CANSparkMax.MotorType.kBrushless
        )
        self.deploy_limit_switch = wpilib.DigitalInput(
            DioChannels.climber_deploy_switch
        )
        self.retract_limit_switch = wpilib.DigitalInput(
            DioChannels.climber_retract_switch
        )
        self.speed = 0.0
        self.climb_encoder = self.climbing_motor.getEncoder()
        self.climb_encoder.setPositionConversionFactor(self.GEAR_RATIO)

        self.encoder_limit_enabled = False
        self.climbing_motor.setSoftLimit(
            CANSparkMax.SoftLimitDirection.kForward, self.MOTOR_REV_TOP_LIMIT
        )
        self.climbing_motor.setSoftLimit(
            CANSparkMax.SoftLimitDirection.kReverse, self.MOTOR_REV_BOTTOM_LIMIT
        )
        self.climbing_motor.enableSoftLimit(
            CANSparkMax.SoftLimitDirection.kForward, False
        )
        self.climbing_motor.enableSoftLimit(
            CANSparkMax.SoftLimitDirection.kReverse, False
        )
        return

    @feedback
    def has_climb_finished(self) -> bool:
        return not self.retract_limit_switch.get() or (
            self.encoder_limit_enabled
            and self.climbing_motor.getFault(CANSparkMax.FaultID.kSoftLimitRev)
        )

    @feedback
    def has_deploy_finished(self) -> bool:
        return not self.deploy_limit_switch.get() or (
            self.encoder_limit_enabled
            and self.climbing_motor.getFault(CANSparkMax.FaultID.kSoftLimitFwd)
        )

    def enable_soft_limits(self) -> None:
        self.climbing_motor.enableSoftLimit(
            CANSparkMax.SoftLimitDirection.kForward, True
        )
        self.climbing_motor.enableSoftLimit(
            CANSparkMax.SoftLimitDirection.kReverse, True
        )
        return

    def deploy(self) -> None:
        if self.has_deploy_finished():
            self.speed = 0.0
        else:
            self.speed = 1.0
        return

    def retract(self) -> None:
        if self.has_climb_finished():
            self.speed = 0.0
        else:
            self.speed = -1.0
        return

    def execute(self) -> None:
        if not self.encoder_limit_enabled:
            if self.has_climb_finished():
                self.enable_soft_limits()
                self.encoder_limit_enabled = True
                self.climb_encoder.setPosition(self.MOTOR_REV_BOTTOM_LIMIT)

            if self.has_deploy_finished():
                self.enable_soft_limits()
                self.encoder_limit_enabled = True
                self.climb_encoder.setPosition(self.MOTOR_REV_TOP_LIMIT)

        self.climbing_motor.set(self.speed)
        self.speed = 0.0
        return
