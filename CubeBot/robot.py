#!/usr/bin/env python3
"""
    This is a demo program showing the use of the RobotDrive class,
    specifically it contains the code necessary to operate a robot with
    tank drive.
"""

import wpilib
from wpilib.drive import DifferentialDrive

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        """Robot initialization function"""

        # object that handles basic drive operations
        self.frontLeftMotor = wpilib.Spark(2)
        self.rearLeftMotor = wpilib.Spark(3)
        self.frontRightMotor = wpilib.Spark(0)
        self.rearRightMotor = wpilib.Spark(1)

        self.left = wpilib.MotorControllerGroup(self.frontLeftMotor, self.rearLeftMotor)
        self.right = wpilib.MotorControllerGroup(
            self.frontRightMotor, self.rearRightMotor
        )   

        # self.left.setInverted
        self.right.setInverted(True)

        self.myRobot = DifferentialDrive(self.left, self.right)
        self.myRobot.setExpiration(0.1)

        # joysticks 1 & 2 on the driver station
        self.driverController = wpilib.Joystick(0)

    def teleopInit(self):
        """Executed at the start of teleop mode"""
        self.myRobot.setSafetyEnabled(True)

    def teleopPeriodic(self):
        """Runs the motors with tank steering"""
        leftStick = -1. * self.driverController.getRawAxis(1)
        rightStick = -1. * self.driverController.getRawAxis(4)
        self.myRobot.arcadeDrive(leftStick, rightStick, True)


if __name__ == "__main__":
    wpilib.run(MyRobot)

