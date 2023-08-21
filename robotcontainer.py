
import wpilib
import wpimath.controller

import commands2
import commands2.cmd
import commands2.button

import constants

import subsystems.drivesubsystem
import subsystems.armsubsystem
import subsystems.intakesubsystem

import commands.turntoangleprofiled
import commands.runautopath
from commands.autocombined import AutoCombined
import commands.intakeIn
import commands.intakeOut

class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.

    """

    def __init__(self):
        """The container for the robot. Contains subsystems, OI devices, and commands."""
        # The robot's subsystems
        self.robotDrive = subsystems.drivesubsystem.DriveSubsystem()
        self.arm = subsystems.armsubsystem.ArmSubsystem()
        self.intake = subsystems.intakesubsystem.IntakeSubsystem()

        # commands

        # The driver controller
        self.driverController = commands2.button.CommandJoystick(
            constants.OIConstants.kDriverControllerPort)

        # The operator controller
        self.opsController = commands2.button.CommandJoystick(
            constants.OIConstants.kOpsControllerPort)

        # Configure the button bindings
        self.configureButtonBindings()

        # Configure default commands
        # Set the default drive command to split-stick arcade drive
        self.robotDrive.setDefaultCommand(
            # A split-stick arcade command, with forward/backward controlled by the left
            # hand, and turning controlled by the right.
            commands2.RunCommand(
                lambda: self.robotDrive.arcadeDrive(
                    -self.driverController.getRawAxis(1),
                    -self.driverController.getRawAxis(4)
                ),
                [self.robotDrive],
            )
        )

    def configureButtonBindings(self):

        # Drive at half speed when the right bumper is held
        commands2.button.JoystickButton(
            self.driverController, constants.OIConstants.kDriverRightBumper
        ).onTrue(
            commands2.InstantCommand(
                (lambda: self.robotDrive.setMaxOutput(0.5)), [self.robotDrive]
            )
        ).onFalse(
            commands2.InstantCommand(
                (lambda: self.robotDrive.setMaxOutput(1)), [self.robotDrive]
            )
        )

        # Stabilize robot to drive straight with gyro when left bumper is held
        commands2.button.JoystickButton(
            self.driverController, constants.OIConstants.kDriverLeftBumper
        ).whileTrue(
            commands2.PIDCommand(
                wpimath.controller.PIDController(
                    constants.DriveConstants.kStabilizationP,
                    constants.DriveConstants.kStabilizationI,
                    constants.DriveConstants.kStabilizationD,
                ),
                # Close the loop on the turn rate
                self.robotDrive.getTurnRate,
                # Setpoint is 0
                0,
                # Pipe the output to the turning controls
                lambda output: self.robotDrive.arcadeDrive(
                    -self.driverController.getRawAxis(1), output
                ),
                # Require the robot drive
                [self.robotDrive],
            )
        )

      # # Turn to 90 degrees when the 'X' button is pressed, with a 5 second timeout
      # commands2.button.JoystickButton(
      #     self.driverController, constants.OIConstants.kDriverXbutton
      # ).onTrue(commands.turntoangle.TurnToAngle(90, self.robotDrive).withTimeout(5))

        # Turn to +90 degrees with a profile when the X button is pressed, with a 5 second timeout
        commands2.button.JoystickButton(
            self.driverController, constants.OIConstants.kDriverXbutton
        ).onTrue(
            commands.turntoangleprofiled.TurnToAngleProfiled(
                90, self.robotDrive
            ).withTimeout(5)
        )

        # Turn to -90 degrees with a profile when the B button is pressed, with a 5 second timeout
        commands2.button.JoystickButton(
            self.driverController, constants.OIConstants.kDriverBbutton
        ).onTrue(
            commands.turntoangleprofiled.TurnToAngleProfiled(
                -90, self.robotDrive
            ).withTimeout(5)
        )

        # move the arm to the initial position
        commands2.button.JoystickButton(self.opsController, constants.OIConstants.kOpsAbutton).onTrue(
            commands2.RunCommand(
                lambda: self.arm.setTarget(constants.ArmConstants.kPositionInit),
                [self.arm],
            )
        )

        # move the arm to the intake position
        commands2.button.JoystickButton(self.opsController, constants.OIConstants.kOpsYbutton).onTrue(
            commands2.RunCommand(
                lambda: self.arm.setTarget(constants.ArmConstants.kPositionIntake),
                [self.arm],
            )
        )

        # move the arm to the yeet position
        commands2.button.JoystickButton(self.opsController, constants.OIConstants.kOpsXbutton).onTrue(
            commands2.RunCommand(
                lambda: self.arm.setTarget(constants.ArmConstants.kPositionYeet),
                [self.arm],
            )
        )

    #   # run intake in
    #   commands2.button.JoystickButton(
    #       self.opsController, constants.OIConstants.kOpsLeftBumper
    #   ).whileTrue(
    #       commands2.RunCommand(lambda: self.intake.cmdIntakeIn(),[self.intake],)
    #   )

    #   # run intake out
    #   commands2.button.JoystickButton(
    #       self.opsController, constants.OIConstants.kOpsRightBumper
    #   ).whileTrue(
    #       commands2.RunCommand(lambda: self.intake.cmdIntakeOut(),[self.intake],)
    #   )

        # run intake in
        commands2.button.JoystickButton(
            self.opsController, constants.OIConstants.kOpsLeftBumper
        ).whileTrue(
            commands.intakeIn.IntakeIn(self.intake)
        )

        # run intake out
        commands2.button.JoystickButton(
            self.opsController, constants.OIConstants.kOpsRightBumper
        ).whileTrue(
            commands.intakeOut.IntakeOut(self.intake)
        )

    def getAutonomousCommand(self) -> commands2.Command:
        return AutoCombined(self.robotDrive,self.arm,self.intake)
