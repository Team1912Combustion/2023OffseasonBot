
from commands2 import SequentialCommandGroup
from subsystems.drivesubsystem import DriveSubsystem
from commands.drivetime import DriveTime

class RunTest(SequentialCommandGroup):

    def __init__(self, drive: DriveSubsystem) -> None:
        super().__init__()

        self.addCommands(
            DriveTime(-0.6, 2.0, drive),
        )
