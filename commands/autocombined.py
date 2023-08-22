
import wpilib
from commands2 import RunCommand, SequentialCommandGroup, WaitCommand

from subsystems.drivesubsystem import DriveSubsystem
from subsystems.armsubsystem import ArmSubsystem
from subsystems.intakesubsystem import IntakeSubsystem

from commands.intakeOut import IntakeOut
from commands.intakeOff import IntakeOff
from commands.runautopath import RunAutoPath
from commands.runtest import RunTest

import constants

class AutoCombined(SequentialCommandGroup):

    def __init__(self, drive: DriveSubsystem, arm: ArmSubsystem, intake: IntakeSubsystem) -> None:
        super().__init__()

        self.drive = drive
        self.arm = arm
        self.intake = intake

        self.addCommands(
           RunCommand(lambda: self.arm.setTarget(constants.ArmConstants.kPositionYeet),
               self.arm).withTimeout(3.),
           #WaitCommand(1.),
           IntakeOut(self.intake).withTimeout(0.2),
           #WaitCommand(0.5),
           IntakeOff(self.intake).withTimeout(0.2),
           RunCommand(lambda: self.arm.setTarget(constants.ArmConstants.kPositionInit),
               self.arm).withTimeout(3.),
           WaitCommand(.2),
           RunTest(self.drive),
           #RunAutoPath(self.drive),
        )
