
import wpilib
import commands2
import commands2.cmd
import wpimath.controller

from subsystems.drivesubsystem import DriveSubsystem

import constants

class RunAutoPath(commands2.Command):

    def __init__(self, drive: DriveSubsystem) -> None:
            
        super().__init__(
            # Close loop on heading
            drive.getHeading,
            # Require the drive
            [drive],
        )

    def isFinished(self) -> bool:
        # End when the controller is at the reference.
        return self.getController().atSetpoint()

    def getAutonomousCommand(self):
        """Returns the command to be ran during the autonomous period."""
        # Create a voltage constraint to ensure we don't accelerate too fast.

        autoVoltageConstraint = DifferentialDriveVoltageConstraint(
            SimpleMotorFeedforwardMeters(
                constants.ksVolts,
                constants.kvVoltSecondsPerMeter,
                constants.kaVoltSecondsSquaredPerMeter,
            ),
            constants.kDriveKinematics,
            maxVoltage=10,  # 10 volts max.
        )
        # Create a configuration for the trajctory. This tells the trajectory its constraints
        # as well as its resources, such as the kinematics object.
        config = TrajectoryConfig(
            constants.kMaxSpeedMetersPerSecond,
            constants.kMaxAccelerationMetersPerSecondSquared,
        )

        # Ensures that the max speed is actually obeyed.
        config.setKinematics(constants.kDriveKinematics)
        (0, 0, Rotation2d(0))

        # Here are the movements we also want to make during this command.
        # These movements should make an "S" like curve.
        movements = [Translation2d(1, 1), Translation2d(2, -1)]

        # End at this position, three meters straight ahead of us, facing forward.
        finalPosition = Pose2d(3, 0, Rotation2d(0))

        # An example trajectory to follow. All of these units are in meters.
        self.exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            initialPosition,
            movements, # Below will generate the trajectory using a set of programmed configurations

     
        # Create a configuration for the trajctory. This tells the trajectory its constraints
        # as well as its resources, such as the kinematics object.
        config = TrajectoryConfig(
            constants.kMaxSpeedMetersPerSecond,
            constants.kMaxAccelerationMetersPerSecondSquared,
        )TrajectoryGenerator
        
        # Ensures that the max speed is actually obeyed.
        config.setKinematics(constants.kDriveKinematics)
se2d(0, 0, Rotation2d(0))

        # Here are the movements we also want to make during this command.
        # These movements should make an "S" like curve.
        movements = [Translation2d(1, 1), Translation2d(2, -1)]

        # End at this position, three meters straight ahead of us, facing forward.
        finalPosition = Pose2d(3, 0, Rotation2d(0))

        # An example trajectory to follow. All of these units are in meters.
        self.exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            initialPosition,
            movements, # Below will generate the trajectory using a set of programmed configurations

        # Create a configuration for the trajctory. This tells the trajectory its constraints
        # as well as its resources, such as the kinematics object.
        config = TrajectoryConfig(
            constants.kMaxSpeedMetersPerSecond,
            constants.kMaxAccelerationMetersPerSecondSquared,
        )

        # Ensures that the max speed is actually obeyed.
        config.setKinematics(constants.kDriveKinematics)

            finalPosition,
            config,
        )

        # Below creates the RAMSETE command

        ramseteCommand = RamseteCommand(
            # The trajectory to follow.
            self.exampleTrajectory,
            # A reference to a method that will return our position.
            self.robotDrive.getPose,
            # Our RAMSETE controller.
            RamseteController(constants.kRamseteB, constants.kRamseteZeta),
            # A feedforward object for the robot.
            SimpleMotorFeedforwardMeters(
                constants.ksVolts,
                constants.kvVoltSecondsPerMeter,
                constants.kaVoltSecondsSquaredPerMeter,
            ),
            # Our drive kinematics.
            constants.kDriveKinematics,
            # A reference to a method which will return a DifferentialDriveWheelSpeeds object.
            self.robotDrive.getWheelSpeeds,
            # The turn controller for the left side of the drivetrain.
            PIDController(constants.kPDriveVel, 0, 0),
            # The turn controller for the right side of the drivetrain.
            PIDController(constants.kPDriveVel, 0, 0),
            # A reference to a method which will set a specified
            # voltage to each motor. The command will pass the two parameters.
            self.robotDrive.tankDriveVolts,
            # The subsystems the command should require.
            [self.robotDrive],
        )

        # Reset the robot's position to the starting position of the trajectory.
        self.robotDrive.resetOdometry(self.exampleTrajectory.initialPose())

        # Return the command to schedule. The "andThen()" will halt the robot after
        # the command finishes.
        return ramseteCommand.andThen(lambda: self.robotDrive.tankDriveVolts(0, 0))

            finalPosition,
            config,
        )

        # Below creates the RAMSETE command

        ramseteCommand = RamseteCommand(
        )
            # The trajecto # Below will generate the trajectory using a set of programmed configurations

        # Create a configuration for the trajctory. This tells the trajectory its constraints
        # as well as its resources, such as the kinematics object.
        config = TrajectoryConfig(
            constants.kMaxSpeedMetersPerSecond,
            constants.kMaxAccelerationMetersPerSecondSquared,
        )

        # Ensures that the max speed is actually obeyed.
        config.setKinematics(constants.kDriveKinematics)
se2d(0, 0, Rotation2d(0))

        # Here are the movements we also want to make during this command.
        # These movements should make an "S" like curve.
movements = [Translation2d(1, 1), Translation2d(2, -1)]

        # End at this position, three meters straight ahead of us, facing forward.
finalPosition = Pose2d(3, 0, Rotation2d(0))

        # An example trajectory to follow. All of these units are in meters.
self.exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            initialPosition,
            movements, # Below will generate the trajectory using a set of programmed configurations

        # Create a configuration for the trajctory. This tells the trajectory its constraints
        # as well as its resources, such as the kinematics object.
        config = TrajectoryConfig(
            constants.kMaxSpeedMetersPerSecond,
            constants.kMaxAccelerationMetersPerSecondSquared,
        )

        # Ensures that the max speed is actually obeyed.
        config.setKinematics(constants.kDriveKinematics)

            finalPosition,
            config,
        )

        # Below creates the RAMSETE command

ramseteCommand = RamseteCommand(
            # The trajectory to follow.
            self.exampleTrajectory,
            # A reference to a method that will return our position.
            self.robotDrive.getPose,
            # Our RAMSETE controller.
            RamseteController(constants.kRamseteB, constants.kRamseteZeta),
            # A feedforward object for the robot.
            SimpleMotorFeedforwardMeters(
                constants.ksVolts,
                constants.kvVoltSecondsPerMeter,
                constants.kaVoltSecondsSquaredPerMeter,
            ),
            # Our drive kinematics.
            constants.kDriveKinematics,
            # A reference to a method which will return a DifferentialDriveWheelSpeeds object.
            self.robotDrive.getWheelSpeeds,
            # The turn controller for the left side of the drivetrain.
            PIDController(constants.kPDriveVel, 0, 0),
            # The turn controller for the right side of the drivetrain.
            PIDControll # Below will generate the trajectory using a set of programmed configurations

        # Create a configuration for the trajctory. This tells the trajectory its constraints
        # as well as its resources, such as the kinematics object.
        config = TrajectoryConfig(
            constants.kMaxSpeedMetersPerSecond,
            constants.kMaxAccelerationMetersPerSecondSquared,
        )

        # Ensures that the max speed is actually obeyed.
        config.setKinematics(constants.kDriveKinematics)
se2d(0, 0, Rotation2d(0))

        # Here are the movements we also want to make during this command.
        # These movements should make an "S" like curve.
        movements = [Translation2d(1, 1), Translation2d(2, -1)]

        # End at this position, three meters straight ahead of us, facing forward.
        finalPosition = Pose2d(3, 0, Rotation2d(0))

        # An example trajectory to follow. All of these units are in meters.
        self.exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            initialPosition,
            movements, # Below will generate the trajectory using a set of programmed configurations

        # Create a configuration for the trajctory. This tells the trajectory its constraints
        # as well as its resources, such as the kinematics object.
        config = TrajectoryConfig(
            constants.kMaxSpeedMetersPerSecond,
            constants.kMaxAccelerationMetersPerSecondSquared,
        )

        # Ensures that the max speed is actually obeyed.
        config.setKinematics(constants.kDriveKinematics)

            finalPosition,
            config,
        )

        # Below creates the RAMSETE command

        ramseteCommand = RamseteCommand(
            # The trajectory to follow.
            self.exampleTrajectory,
            # A reference to a method that will return our position.
            self.robotDrive.getPose,
            # Our RAMSETE controller.
            RamseteController(constants.kRamseteB, constants.kRamseteZeta),
            # A feedforward object for the robot.
            SimpleMotorFeedforwardMeters(
                constants.ksVolts,
                constants.kvVoltSecondsPerMeter,
                constants.kaVoltSecondsSquaredPerMeter,
            ),
            # Our drive kinematics.
            constants.kDriveKinematics,
            # A reference to a method which will return a DifferentialDriveWheelSpeeds object.
            self.robotDrive.getWheelSpeeds,
            # The turn controller for the left side of the drivetrain.
            PIDController(constants.kPDriveVel, 0, 0),
            # The turn controller for the right side of the drivetrain.
            PIDController(constants.kPDriveVel, 0, 0),
            # A reference to a method which will set a specified
            # voltage to each motor. The command will pass the two parameters.
            self.robotDrive.tankDriveVolts,
            # The subsystems the command should require.
            [self.robotDrive],
        )

        # Reset the robot's position to the starting position of the trajectory.
        self.robotDrive.resetOdometry(self.exampleTrajectory.initialPose())

        # Return the command to schedule. The "andThen()" will halt the robot after
        # the command finishes.
        return ramseteCommand.andThen(lambda: self.robotDrive.tankDriveVolts(0, 0))
er(constants.kPDriveVel, 0, 0),
            # A reference to a method which will set a specified
            # voltage to each motor. The command will pass the two parameters.
            self.robotDrive.tankDriveVolts,
            # The subsystems the command should require.
            [self.robotDr # Below will generate the trajectory using a set of programmed configurations

        # Create a configuration for the trajctory. This tells the trajectory its constraints
        # as well as its resources, such as the kinematics object.
        config = TrajectoryConfig(
            constants.kMaxSpeedMetersPerSecond,
            constants.kMaxAccelerationMetersPerSecondSquared,
        )

        # Ensures that the max speed is actually obeyed.
        config.setKinematics(constants.kDriveKinematics)
se2d(0, 0, Rotation2d(0))

        # Here are the movements we also want to make during this command.
        # These movements should make an "S" like curve.
        movements = [Translation2d(1, 1), Translation2d(2, -1)]

        # End at this position, three meters straight ahead of us, facing forward.
        finalPosition = Pose2d(3, 0, Rotation2d(0))

        # An example trajectory to follow. All of these units are in meters.
        self.exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            initialPosition,
            movements, # Below will generate the trajectory using a set of programmed configurations

        # Create a configuration for the trajctory. This tells the trajectory its constraints
        # as well as its resources, such as the kinematics object.
        config = TrajectoryConfig(
            constants.kMaxSpeedMetersPerSecond,
            constants.kMaxAccelerationMetersPerSecondSquared,
        )

        # Ensures that the max speed is actually obeyed.
        config.setKinematics(constants.kDriveKinematics)

            finalPosition,
            config,
        )

        # Below creates the RAMSETE command

        ramseteCommand = RamseteCommand(
            # The trajectory to follow.
            self.exampleTrajectory,
            # A reference to a method that will return our position.
            self.robotDrive.getPose,
            # Our RAMSETE controller.
            RamseteController(constants.kRamseteB, constants.kRamseteZeta),
            # A feedforward object for the robot.
            SimpleMotorFeedforwardMeters(
                constants.ksVolts,
                constants.kvVoltSecondsPerMeter,
                constants.kaVoltSecondsSquaredPerMeter,
            ),
            # Our drive kinematics.
            constants.kDriveKinematics,
            # A reference to a method which will return a DifferentialDriveWheelSpeeds object.
            self.robotDrive.getWheelSpeeds,
            # The turn controller for the left side of the drivetrain.
            PIDController(constants.kPDriveVel, 0, 0),
            # The turn controller for the right side of the drivetrain.
            PIDController(constants.kPDriveVel, 0, 0),
            # A reference to a method which will set a specified
            # voltage to each motor. The command will pass the two parameters.
            self.robotDrive.tankDriveVolts,
            # The subsystems the command should require.
            [self.robotDrive],
        )

        # Reset the robot's position to the starting position of the trajectory.
        self.robotDrive.resetOdometry(self.exampleTrajectory.initialPose())

        # Return the command to schedule. The "andThen()" will halt the robot after
        # the command finishes.
        return ramseteCommand.andThen(lambda: self.robotDrive.tankDriveVolts(0, 0))
ive],
        )

        # Reset the robot's position to the starting position of the trajectory.
        self.robotDrive.resetOdometry(self.exampleTrajectory.initialPose())

        # Return the command to schedule. The "andThen()" will halt the robot after
        # the command finishes.
        ramseteCommand.andThen(lambda: self.robotDrive.tankDriveVolts(0, 0))
 
        #self.robotDrive.getPose,
            # Our RAMSETE controller.
        RamseteController(constants.kRamseteB, constants.kRamseteZeta),
            # A feedforward object for the robot.
        SimpleMotorFeedforwardMeters(
                constants.ksVolts,
                constants.kvVoltSecondsPerMeter,
                constants.kaVoltSecondsSquaredPerMeter,
            ),
            # Our drive kinematics.
            constants.kDriveKinematics,
            # A reference to a method which will return a DifferentialDriveWheelSpeeds object.
            self.robotDrive.getWheelSpeeds,
            # The turn controller for the left side of the drivetrain.
            PIDController(constants.kPDriveVel, 0, 0),
            # The turn controller for the right side of the drivetrain.
            PIDController(constants.kPDriveVel, 0, 0),
            # A reference to a method which will set a specified
            # voltage to each motor. The command will pass the two parameters.
            self.robotDrive.tankDriveVolts,
            # The subsystems the command should require.
            [self.robotDrive],
        )

        # Reset the robot's position to the starting position of the trajectory.
        self.robotDrive.resetOdometry(self.exampleTrajectory.initialPose())

        # Return the command to schedule. The "andThen()" will halt the robot after
        # the command finishes.
        return ramseteCommand.andThen(lambda: self.robotDrive.tankDriveVolts(0, 0))

 # Below will generate the trajectory using a set of programmed configurations

        # Create a configuration for the trajctory. This tells the trajectory its constraints
        # as well as its resources, such as the kinematics object.
        config = TrajectoryConfig(
            constants.kMaxSpeedMetersPerSecond,
            co
            
            
            nstants.kMaxAccelerationMetersPerSecondSquared,
        )

        # Ensures that the max speed is actually obeyed.
        config.setKinematics(constants.kDriveKinematics)
se2d(0, 0, Rotation2d(0))

        # Here are the movements we also want to make during this command.
        # These movements should make an "S" like curve.
        movements = [Translation2d(1, 1), Translation2d(2, -1)]

        # End at this position, three meters straight ahead of us, facing forward.
        finalPosition = Pose2d(3, 0, Rotation2d(0))

        # An example trajectory to follow. All of these units are in meters.
        self.exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            initialPosition,
            movements, # Below will generate the trajectory using a set of programmed configurations

        # Create a configuration for the trajctory. This tells the trajectory its constraints
        # as well as its resources, such as the kinematics object.
        config = TrajectoryConfig(
            constants.kMaxSpeedMetersPerSecond,
            constants.kMaxAccelerationMetersPerSecondSquared,
        )

        # Ensures that the max speed is actually obeyed.
        config.setKinematics(constants.kDriveKinematics)

            finalPosition,
            config,
        )
        

        # Below creates the RAMSETE command

        ramseteCommand = RamseteCommand(
            # The trajectory to follow.
            self.exampleTrajectory,
            # A reference to a method that will return our position.
            self.robotDrive.getPose,
            # Our RAMSETE controller.
            RamseteController(constants.kRamseteB, constants.kRamseteZeta),
            # A feedforward object for the robot.
            SimpleMotorFeedforwardMeters(
                constants.ksVolts,
                constants.kvVoltSecondsPerMeter,
                constants.kaVoltSecondsSquaredPerMeter,
            ),
            # Our drive kinematics.
            constants.kDriveKinematics,
            # A reference to a method which will return a DifferentialDriveWheelSpeeds object.
            self.robotDrive.getWheelSpeeds,
            # The turn controller for the left side of the drivetrain.
            PIDController(constants.kPDriveVel, 0, 0),
            # The turn controller for the right side of the drivetrain.
            PIDController(constants.kPDriveVel, 0, 0),
            # A reference to a method which will set a specified
            # voltage to each motor. The command will pass the two parameters.
            self.robotDrive.tankDriveVolts,
            # The subsystems the command should require.
            [self.robotDrive],
        )

        # Reset the robot's position to the starting position of the trajectory.
        self.robotDrive.resetOdometry(self.exampleTrajectory.initialPose())

        # Return the command to schedule. The "andThen()" will halt the robot after
        # the command finishes.
        return ramseteCommand.andThen(lambda: self.robotDrive.tankDriveVolts(0, 0))
        

