# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import wpilib
import commands2
import rev

import constants

class ArmSubsystem(commands2.SubsystemBase):
    def __init__(self) -> None:
        """Creates a new DriveSubsystem"""
        super().__init__()

        # The motors on the left side of the drive.
        self.left = rev.CANSparkMax(constants.ArmConstants.kLeftMotorPort)
        self.right = rev.CANSparkMax(constants.ArmConstants.kRightMotorPort)

        # The left-side drive encoder
        self.leftEncoder = self.left.getEncoder()
        self.rightEncoder = self.right.getEncoder()

        # We need to invert one side of the drivetrain so that positive voltages
        # result in both sides moving forward. Depending on how your robot's
        # gearbox is constructed, you might have to invert the left side instead.
        self.right.setInverted(True)

        # Create an object for our odometry, which will utilize sensor data to
        # keep a record of our position on the field.
        self.odometry = DifferentialDriveOdometry(
            self.navx.getRotation2d(),
            self.leftEncoder.getDistance(),
            self.rightEncoder.getDistance(),
        )
        
        self.leftEncoder.setPosition(0.)
        self.rightEncoder.setPosition(0.)

        self.target = 0.
        self.kP = .002
        self.kO = .002

    def getTarget(self):
        return self.target

    def setTarget(self, target):
        self.target = target

    def error(self):
        return 0.5 * (self.leftEncoder.getPosition() + self.rightEncoder.getPosition()) - self.getTarget()

    def offset(self):
        return self.leftEncoder.getPosition() - self.rightEncoder.getPosition()

    def periodic(self):
        error = self.meanposition() - self.target
        leftMotorPower = self.kP * error + self.kO * self.offset
        rightMotorPower = self.kP * error - self.kO * self.offset


