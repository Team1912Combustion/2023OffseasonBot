
import wpilib
import wpilib.drive
import commands2
import math
import navx
from wpimath.kinematics import DifferentialDriveOdometry, DifferentialDriveWheelSpeeds

import constants

class DriveSubsystem(commands2.SubsystemBase):
    def __init__(self) -> None:
        """Creates a new DriveSubsystem"""
        super().__init__()

        # The motors on the left side of the drive.
        self.left = wpilib.Spark(constants.DriveConstants.kLeftMotorPort)
        self.right = wpilib.Spark(constants.DriveConstants.kRightMotorPort)

        # The robot's drive
        self.drive = wpilib.drive.DifferentialDrive(self.left, self.right)

        # The left-side drive encoder
        self.leftEncoder = wpilib.Encoder(
            constants.DriveConstants.kLeftEncoderPorts[0],
            constants.DriveConstants.kLeftEncoderPorts[1],
            constants.DriveConstants.kLeftEncoderReversed,
        )

        # The right-side drive encoder
        self.rightEncoder = wpilib.Encoder(
            constants.DriveConstants.kRightEncoderPorts[0],
            constants.DriveConstants.kRightEncoderPorts[1],
            constants.DriveConstants.kRightEncoderReversed,
        )

        # We need to invert one side of the drivetrain so that positive voltages
        # result in both sides moving forward. Depending on how your robot's
        # gearbox is constructed, you might have to invert the left side instead.
        self.right.setInverted(True)

        # Sets the distance per pulse for the encoders
        self.leftEncoder.setDistancePerPulse(
            constants.DriveConstants.kEncoderDistancePerPulse
        )
        self.rightEncoder.setDistancePerPulse(
            constants.DriveConstants.kEncoderDistancePerPulse
        )

        self.navx = navx.AHRS.create_spi()

        # Create an object for our odometry, which will utilize sensor data to
        # keep a record of our position on the field.
        self.odometry = DifferentialDriveOdometry(
            self.navx.getRotation2d(),
            self.leftEncoder.getDistance(),
            self.rightEncoder.getDistance(),
        )

    def periodic(self):
        self.odometry.update(
            self.navx.getRotation2d(),
            self.leftEncoder.getDistance(),
            self.rightEncoder.getDistance(),
        )

    def getPose(self):
        return self.odometry.getPose()

    def getWheelSpeeds(self):
        speeds = DifferentialDriveWheelSpeeds(
            self.leftEncoder.getRate(), self.rightEncoder.getRate()
        )
        return speeds

    def resetOdometry(self, pose):
        self.resetEncoders()
        self.odometry.resetPosition(
            self.navx.getRotation2d(),
            self.leftEncoder.getDistance(),
            self.rightEncoder.getDistance(),
            pose,
        )

    def arcadeDrive(self, fwd: float, rot: float):
        self.drive.arcadeDrive(fwd, rot)

    def resetEncoders(self):
        self.leftEncoder.reset()
        self.rightEncoder.reset()

    def getAverageEncoderDistance(self):
        return (self.leftEncoder.getDistance() + self.rightEncoder.getDistance()) / 2.0

    def getLeftEncoder(self) -> wpilib.Encoder:
        return self.leftEncoder

    def getRightEncoder(self) -> wpilib.Encoder:
        return self.rightEncoder

    def setMaxOutput(self, maxOutput: float):
        self.drive.setMaxOutput(maxOutput)

    def zeroHeading(self):
        self.navx.reset()

    def getHeading(self):
        return self.navx.getYaw() * (
            -1 if constants.DriveConstants.kGyroReversed else 1
        )

    def getTurnRate(self):
        return self.navx.getRate() * (
            -1 if constants.DriveConstants.kGyroReversed else 1
        )

    def tankDriveVolts(self, leftVolts, rightVolts):
        # Set the voltage of the left side.
        self.leftMotor.setVoltage(leftVolts)

        # Set the voltage of the right side. It's
        # inverted with a negative sign because it's motors need to spin in the negative direction
        # to move forward.
        self.rightMotor.setVoltage(-rightVolts)

        # Resets the timer for this motor's MotorSafety
        self.drive.feed()