
from commands2 import CommandBase, RamseteCommand, RunCommand
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

class RunAutoPath(CommandBase):

    def __init__(self, drive: DriveSubsystem) -> None:
        super().__init__()
        self.drive = drive
        self.addRequirements(drive)

    def end(self) -> None:
        self.drive.tankDriveVolts(0, 0)

    def isFinished(self) -> bool:
        return True

    def initialize(self) -> None:
        # Create a voltage constraint to ensure we don't accelerate too fast.
        autoVoltageConstraint = DifferentialDriveVoltageConstraint(
            SimpleMotorFeedforwardMeters(
                constants.AutoConstants.ksVolts,
                constants.AutoConstants.kvVoltSecondsPerMeter,
                constants.AutoConstants.kaVoltSecondsSquaredPerMeter,
            ),
            constants.AutoConstants.kDriveKinematics,
            maxVoltage=10,  # 10 volts max.
        )

        # Create a configuration for the trajctory. This tells the trajectory its constraints
        # as well as its resources, such as the kinematics object.
        config = TrajectoryConfig(
            constants.AutoConstants.kMaxSpeedMetersPerSecond,
            constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared,
        )

        # Ensures that the max speed is actually obeyed.
        config.setKinematics(constants.AutoConstants.kDriveKinematics)

        # Apply the previously defined voltage constraint.
        config.addConstraint(autoVoltageConstraint)

        # Start at the origin facing the +x direction.
        initialPosition = Pose2d(0, 0, Rotation2d(0))

        # Here are the movements we also want to make during this command.
        # These movements should make an "S" like curve.
        movements = [Translation2d(1, 1), Translation2d(2, -1)]

        # End at this position, three meters straight ahead of us, facing forward.
        finalPosition = Pose2d(3, 0, Rotation2d(0))

        # An example trajectory to follow. All of these units are in meters.
        self.exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            initialPosition,
            movements,
            finalPosition,
            config,
        )

        # create the RAMSETE command
        self.ramseteCommand = RamseteCommand(
            # The trajectory to follow.
            self.exampleTrajectory,
            # A reference to a method that will return our position.
            self.drive.getPose,
            # Our RAMSETE controller.
            RamseteController(constants.AutoConstants.kRamseteB, constants.AutoConstants.kRamseteZeta),
            # A feedforward object for the robot.
            SimpleMotorFeedforwardMeters(
                constants.AutoConstants.ksVolts,
                constants.AutoConstants.kvVoltSecondsPerMeter,
                constants.AutoConstants.kaVoltSecondsSquaredPerMeter,
            ),
            # Our drive kinematics.
            constants.AutoConstants.kDriveKinematics,
            # A reference to a method which will return a DifferentialDriveWheelSpeeds object.
            self.drive.getWheelSpeeds,
            # The turn controller for the left side of the drivetrain.
            PIDController(constants.AutoConstants.kPDriveVel, 0, 0),
            # The turn controller for the right side of the drivetrain.
            PIDController(constants.AutoConstants.kPDriveVel, 0, 0),
            # A reference to a method which will set a specified
            # voltage to each motor. The command will pass the two parameters.
            self.drive.tankDriveVolts,
            # The subsystems the command should require.
            [self.drive],
        )

    def execute(self) -> None:
        # Reset the robot's position to the starting position of the trajectory.
        self.drive.resetOdometry(self.exampleTrajectory.initialPose())
        # Run the RAMSETE
        self.ramseteCommand.andThen(lambda: self.drive.tankDriveVolts(0, 0))
