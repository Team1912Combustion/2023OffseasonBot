# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import wpilib
import commands2
import rev
import constants

class IntakeSubsystem(commands2.SubsystemBase):
    def __init__(self) -> None:
        """Creates a new IntakeSubsystem"""
        super().__init__()

        # The intake motors 
        self.intake = rev.CANSparkMax(constants.ArmConstants.kIntakeMotorPort, rev.CANSparkMaxLowLevel.MotorType.kBrushless)

        # The arm encoders
        self.intakeEncoder = self.intake.getEncoder()

        self.powerIn = constants.ArmConstants.kIntakePowerIn
        self.powerOut = constants.ArmConstants.kIntakePowerOut

    def initDefaultCommand(self):
        self.setDefaultCommand(
            commands2.RunCommand(lambda: self.stop(),self,)
        )

    def intakeIn(self):
        self.intake.set(self.powerIn)

    def intakeOut(self):
        self.intake.set(self.powerOut)

    def stop(self):
        self.intake.set(0.)

