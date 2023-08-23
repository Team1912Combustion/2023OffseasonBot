
from commands2 import SequentialCommandGroup
from subsystems.drivesubsystem import DriveSubsystem
from commands.drivetime import DriveTime

class RunDriveTime(SequentialCommandGroup):

    def __init__(self, drive: DriveSubsystem) -> None:
        super().__init__()

        self.addCommands(
            DriveTime(-0.7, 6.0, drive),
        )
