
from commands2 import RamseteCommand, RunCommand, SequentialCommandGroup
from wpimath.controller import (
    RamseteController,
    PIDController,
    SimpleMotorFeedforwardMeters,
)
from wpimath.kinematics import ChassisSpeeds
from wpimath.trajectory.constraint import DifferentialDriveVoltageConstraint
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator, Trajectory
from wpimath.geometry import Pose2d, Rotation2d, Translation2d

from subsystems.drivesubsystem import DriveSubsystem
import constants
from commands.drivetime import DriveTime

class RunTest(SequentialCommandGroup):

    def __init__(self, drive: DriveSubsystem) -> None:
        super().__init__()

        self.addCommands(
            DriveTime(-0.6, 2.0, drive),
            drive.tankDriveVolts(0, 0),
        )
