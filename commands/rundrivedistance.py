
from commands2 import SequentialCommandGroup
from subsystems.drivesubsystem import DriveSubsystem
from commands.drivetime import DriveTime

class RunDriveDistance(SequentialCommandGroup):

    def __init__(self, drive: DriveSubsystem) -> None:
        super().__init__()

        self.addCommands(
            DriveDistance(-4.6, 6.0, 0.1, drive),
        )
