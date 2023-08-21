
import wpilib
from commands2 import RunCommand, SequentialCommandGroup, WaitCommand

from subsystems.drivesubsystem import DriveSubsystem
from subsystems.armsubsystem import ArmSubsystem
from subsystems.intakesubsystem import IntakeSubsystem

from commands.intakeOut import IntakeOut
from commands.intakeOff import IntakeOff
from commands.runautopath import RunAutoPath

import constants

class AutoCombined(SequentialCommandGroup):

    def __init__(self, drive: DriveSubsystem, arm: ArmSubsystem, intake: IntakeSubsystem) -> None:
        super().__init__()

        self.drive = drive
        self.arm = arm
        self.intake = intake

        self.addCommands(
           RunCommand(lambda: self.arm.setTarget(constants.ArmConstants.kPositionYeet),
               self.arm).withTimeout(2.),
           WaitCommand(2.),
           IntakeOut(self.intake),
           WaitCommand(0.5),
           IntakeOff(self.intake),
           RunCommand(lambda: self.arm.setTarget(constants.ArmConstants.kPositionInit),
               self.arm).withTimeout(1.),
           WaitCommand(1.),
           RunAutoPath(self.drive)
        )
