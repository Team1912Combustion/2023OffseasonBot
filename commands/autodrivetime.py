
import wpilib
from commands2 import RunCommand, SequentialCommandGroup, WaitCommand

from subsystems.drivesubsystem import DriveSubsystem
from subsystems.armsubsystem import ArmSubsystem
from subsystems.intakesubsystem import IntakeSubsystem

from commands.intakeOut import IntakeOut
from commands.intakeOff import IntakeOff
from commands.rundrivetime import RunDriveTime

import constants

class AutoDriveTime(SequentialCommandGroup):

    def __init__(self, drive: DriveSubsystem, arm: ArmSubsystem, intake: IntakeSubsystem) -> None:
        super().__init__()

        self.drive = drive
        self.arm = arm
        self.intake = intake

        self.addCommands(
           RunCommand(lambda: self.arm.setTarget(constants.ArmConstants.kPositionYeet),
               self.arm).withTimeout(3.),
           IntakeOut(self.intake).withTimeout(0.2),
           IntakeOff(self.intake).withTimeout(0.2),
           RunCommand(lambda: self.arm.setTarget(constants.ArmConstants.kPositionInit),
               self.arm).withTimeout(3.),
           WaitCommand(.2),
           RunDriveTime(self.drive),
        )
