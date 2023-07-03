
import wpilib
import wpilib.drive
import commands2
import math
import navx

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

    def arcadeDrive(self, fwd: float, rot: float):
        """
        Drives the robot using arcade controls.

        :param fwd: the commanded forward movement
        :param rot: the commanded rotation
        """
        self.drive.arcadeDrive(fwd, rot)

    def resetEncoders(self):
        """Resets the drive encoders to currently read a position of 0."""
        self.leftEncoder.reset()
        self.rightEncoder.reset()

    def getAverageEncoderDistance(self):
        """
        Gets the average distance of the two encoders.

        :returns: the average of the two encoder readings
        """
        return (self.leftEncoder.getDistance() + self.rightEncoder.getDistance()) / 2.0

    def getLeftEncoder(self) -> wpilib.Encoder:
        """
        Gets the left drive encoder.

        :returns: the left drive encoder
        """
        return self.leftEncoder

    def getRightEncoder(self) -> wpilib.Encoder:
        """
        Gets the right drive encoder.

        :returns: the right drive encoder
        """
        return self.rightEncoder

    def setMaxOutput(self, maxOutput: float):
        """
        Sets the max output of the drive. Useful for scaling the drive to drive more slowly.

        :param maxOutput: the maximum output to which the drive will be constrained
        """
        self.drive.setMaxOutput(maxOutput)

    def zeroHeading(self):
        """
        Zeroes the heading of the robot.
        """
        self.navx.reset()

    def getHeading(self):
        """
        Returns the heading of the robot.

        :returns: the robot's heading in degrees, from -180 to 180
        """
        return self.navx.getYaw() * (
            -1 if constants.DriveConstants.kGyroReversed else 1
        )

    def getTurnRate(self):
        """
        Returns the turn rate of the robot.

        :returns: The turn rate of the robot, in degrees per second
        """
        return self.navx.getRate() * (
            -1 if constants.DriveConstants.kGyroReversed else 1
        )
