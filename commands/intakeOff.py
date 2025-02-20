# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import wpilib
import commands2
import commands2.cmd

from subsystems.intakesubsystem import IntakeSubsystem

import constants

class IntakeOff(commands2.CommandBase):

    def __init__(self, intake: IntakeSubsystem) -> None:
        super().__init__()
        self.intake = intake

    def execute(self) -> None:
        self.intake.intakeStop()

    def end(self, interrupted: bool) -> None:
        self.intake.intakeStop()

    def isFinished(self) -> bool:
        return True

