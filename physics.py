#
# See the documentation for more details on how this works
#
# The idea here is you provide a simulation object that overrides specific
# pieces of WPILib, and modifies motors/sensors accordingly depending on the
# state of the simulation. An example of this would be measuring a motor
# moving for a set period of time, and then changing a limit switch to turn
# on after that period of time. This can help you do more complex simulations
# of your robot code without too much extra effort.
#

import wpilib
import wpilib.simulation
from wpimath.system import LinearSystemId
from wpimath.system.plant import DCMotor
from wpimath.geometry import Pose2d, Rotation2d

import constants

from pyfrc.physics.core import PhysicsInterface

import typing

if typing.TYPE_CHECKING:
    from robot import MyRobot


class PhysicsEngine:
    """
    Simulates a Differential Drivetrain using system identification data for the space-state model.
    """

    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):
        self.physics_controller = physics_controller

        # Motors
        self.l_motor = wpilib.simulation.PWMSim(
            robot.container.robotDrive.left.getChannel()
        )
        self.r_motor = wpilib.simulation.PWMSim(
            robot.container.robotDrive.right.getChannel()
        )

        self.system = LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5, 0.3)
        self.drivesim = wpilib.simulation.DifferentialDrivetrainSim(
            self.system,
            constants.AutoConstants.kTrackWidthMeters,
            DCMotor.CIM(constants.AutoConstants.kDriveTrainMotorCount),
            constants.AutoConstants.kGearingRatio,
            constants.AutoConstants.kWheelRadius,
        )

        self.leftEncoderSim = wpilib.simulation.EncoderSim(
            robot.container.robotDrive.leftEncoder
        )
        self.rightEncoderSim = wpilib.simulation.EncoderSim(
            robot.container.robotDrive.rightEncoder
        )
        self.rightEncoderSim.setReverseDirection(True)

        self.physics_controller.field.setRobotPose(robot.container.robotDrive.getPose())

        self.drivesim.setPose(robot.container.robotDrive.getPose())

    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.

        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """

        # Simulate the drivetrain
        l_motor = self.l_motor.getSpeed()
        r_motor = self.r_motor.getSpeed()

        voltage = wpilib.RobotController.getInputVoltage()
        self.drivesim.setInputs(l_motor * voltage, -1. * r_motor * voltage)
        self.drivesim.update(tm_diff)

        self.leftEncoderSim.setDistance(self.drivesim.getLeftPosition())
        self.leftEncoderSim.setRate(self.drivesim.getLeftVelocity())
        self.rightEncoderSim.setDistance(self.drivesim.getRightPosition())
        self.rightEncoderSim.setRate(self.drivesim.getRightVelocity())

        self.physics_controller.field.setRobotPose(self.drivesim.getPose())
