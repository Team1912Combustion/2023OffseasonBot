# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import wpilib
import wpimath
import commands2
import rev
import constants

class PidInstance:
  def __init__(self):
    self.iState = 0.
    self.prev_err = 0.
    self.output = 0.
    self.kP = 0.
    self.kI = 0.
    self.kD = 0.
    self.kF = 0.
    self.rest = 0.
    self.iZone = 0.
    self.kMinOutput = 0.
    self.kMaxOutput = 0.

  def run(self, present_val, setpoint):
    error = setpoint - present_val
    p = error * self.kP         # kP is 1 / real input units

    if (abs(error) <= self.iZone or self.iZone == 0.):
      self.iState = self.iState + (error * self.kI)
    else:
      self.iState = 0.

    d = error - self.prev_err
    self.prev_err = error
    d = d * self.kD             # kD is 1 / real input units / loop time units

    f = (self.rest - setpoint) * self.kF      # kF is 1 / real input units

    output = p + self.iState + d + f
    self.output = min(max(output, self.kMinOutput), self.kMaxOutput)

    return self.output

class ArmSubsystem(commands2.SubsystemBase):
    def __init__(self) -> None:
        """Creates a new ArmSubsystem"""
        super().__init__()

        # The motors on the left side of the arm.
        self.left = rev.CANSparkMax(constants.ArmConstants.kLeftMotorPort, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.right = rev.CANSparkMax(constants.ArmConstants.kRightMotorPort, rev.CANSparkMaxLowLevel.MotorType.kBrushless)

        # The arm encoders
        self.leftEncoder = self.left.getEncoder()
        self.rightEncoder = self.right.getEncoder()
        self.leftEncoder.setPosition(0.)
        self.rightEncoder.setPosition(0.)

        self.left.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        self.right.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)

        # We need to invert one side of the arm
        self.left.setInverted(constants.ArmConstants.kLeftMotorReversed)
        self.right.setInverted(constants.ArmConstants.kRightMotorReversed)

        # set limits
        self.minpos = constants.ArmConstants.kPositionMin
        self.maxpos = constants.ArmConstants.kPositionMax
        self.target = 0.

      # self.pid = PidInstance()
      # self.pid.kP = 0.002
      # self.pid.kF = 0.0001
      # self.pid.rest = 5.
      # self.pid.target = 0.
      # self.pid.kMinOutput = -0.05
      # self.pid.kMaxOutput = 0.05

        self.alignpid = PidInstance()
        self.alignpid.kP = 0.05
        self.alignpid.target = 0.
        self.alignpid.kMinOutput = -0.1
        self.alignpid.kMaxOutput = 0.1

        self.pid = wpimath.controller.ProfiledPIDController(
            constants.ArmConstants.kMoveP,
            constants.ArmConstants.kMoveI,
            constants.ArmConstants.kMoveD,
            wpimath.trajectory.TrapezoidProfile.Constraints(
                constants.ArmConstants.kMaxMoveRateRevPerS,
                constants.ArmConstants.kMaxMoveAccelerationRevPerSSquared,
            ),
        )

    def getTarget(self):
        return self.target

    def setTarget(self, target):
        self.target = min(max(target,self.minpos),self.maxpos)

    def getPosition(self):
        return 0.5 * (self.leftEncoder.getPosition() + self.rightEncoder.getPosition())

    def getOffset(self):
        return self.rightEncoder.getPosition() - self.leftEncoder.getPosition()

    def moveArm(self, leftpower, rightpower):
        self.left.set(leftpower)
        self.right.set(rightpower)

    def periodic(self):
        movepower = self.pid.calculate(self.getPosition(), self.getTarget())
        alignpower = self.alignpid.run(self.getOffset(), 0.)
        leftMotorPower = movepower - alignpower
        rightMotorPower = movepower + alignpower
        self.moveArm(leftMotorPower, rightMotorPower)

