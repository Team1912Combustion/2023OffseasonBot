# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import wpilib
import commands2
import wpimath

from subsystems.drivesubsystem import DriveSubsystem

import constants

class DriveDistance(commands2.CommandBase):

    def __init__(self, distance: float, timeout: float, error: float,
            drive: DriveSubsystem) -> None:

        super().__init__()

        self.distance = distance
        self.timeout = timeout
        self.error = error
        self.drive = drive
        self.addRequirements(drive)

        self.startTime = 0.0

    def initialize(self) -> None:
        self.startTime = wpilib.Timer.getFPGATimestamp()
        self.drive.arcadeDrive(0, 0)

        self.turnPID = wpimath.controller.PIDController(
            constants.DriveConstants.kStabilizationP,
            constants.DriveConstants.kStabilizationI,
            constants.DriveConstants.kStabilizationD
        )

        self.drivePID = wpimath.controller.ProfiledPIDController(
            constants.DriveConstants.kDriveP,
            constants.DriveConstants.kDriveI,
            constants.DriveConstants.kDriveD,
            wpimath.trajectory.TrapezoidProfile.Constraints(
                constants.AutoConstants.kMaxSpeedMetersPerSecond,
                constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared,
            ),
        )

        self.drive.resetEncoders()

    def execute(self) -> None:
        turnspeed = self.turnPID.calculate(self.drive.getTurnRate(), 0.)
        drivespeed = self.drivePID.calculate(self.drive.getAverageEncoderDistance(),
            self.distance)
        self.drive.arcadeDrive(drivespeed, turnspeed)

    def end(self, interrupted: bool) -> None:
        self.drive.arcadeDrive(0, 0)

    def isFinished(self) -> bool:
        timecheck = wpilib.Timer.getFPGATimestamp() - self.timeout >= 0.
        distcheck = ( abs(self.drive.getAverageEncoderDistance() - self.distance)
            <= self.error )
        return (timecheck or distcheck)
