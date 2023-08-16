# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import wpilib
import commands2
import commands2.cmd
import wpimath.controller
import wpimath.trajectory

from subsystems.armsystem import ArmSubsystem

import constants

class MoveArmProfiled(commands2.ProfiledPIDCommand):
    """A command that will move the arm to the specified position using a motion profile."""

    def __init__(self, targetPosition: float, arm: ArmSubsystem) -> None:
        """
        Move the arm to the specified position.

        :param: targetPosition The position to turn to in motor rotations
        :param: arm The arm subsystem
        """
        super().__init__(
            wpimath.controller.ProfiledPIDController(
                constants.ArmConstants.kMoveP,
                constants.ArmConstants.kMoveI,
                constants.ArmConstants.kMoveD,
                wpimath.trajectory.TrapezoidProfile.Constraints(
                    constants.ArmConstants.kMaxTurnRateRotPerS,
                    constants.ArmConstants.kMaxTurnAccelerationRotPerSSquared,
                ),
            ),
            # Close loop on heading
            arm.getPosition,
            # Set reference to target
            targetPosition,
            # Pipe output to turn robot
            lambda output, setpoint: arm.moveArm(output),
            # Require the drive
            [arm],
        )

        # Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
        # setpoint before it is considered as having reached the reference
        self.getController().setTolerance(
            constants.ArmConstants.kArmToleranceRev,
            constants.ArmConstants.kArmRateToleranceRevPerS,
        )

    def isFinished(self) -> bool:
        # End when the controller is at the reference.
        return self.getController().atSetpoint()
