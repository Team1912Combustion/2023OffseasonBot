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

        self.left.restoreFactoryDefaults()
        self.right.restoreFactoryDefaults()

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

        self.kMinOutput = -0.15
        self.kMaxOutput = 0.15

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

        # make absolutely sure the encoder is set to 0
        while abs(self.leftEncoder.getPosition()) > .1:
          self.leftEncoder.setPosition(0.)
        while abs(self.rightEncoder.getPosition()) > .1:
          self.rightEncoder.setPosition(0.)

        # make absolutely sure the power is set to 0
        while abs(self.left.get()) > .05:
          self.left.set(0.)
        while abs(self.right.get()) > .05:
          self.right.set(0.)

    def getTarget(self):
        return self.target

    def setTarget(self, target):
        self.target = min(max(target,self.minpos),self.maxpos)

    def getPosition(self):
        m_pos = 0.5 * (self.leftEncoder.getPosition() + self.rightEncoder.getPosition())
        return min(max(m_pos,self.minpos-.2),self.maxpos+.2)

    def getOffset(self):
        return self.rightEncoder.getPosition() - self.leftEncoder.getPosition()

    def moveArm(self, leftpower, rightpower):
        self.left.set(leftpower)
        self.right.set(rightpower)

    def limitPower(self, power):
        return min(max(power, self.kMinOutput), self.kMaxOutput)

    def periodic(self):
        # PID result to send to motors
        movepower = self.pid.calculate(self.getPosition(), self.getTarget())
        # adjust for alignment
        alignpower = self.alignpid.run(self.getOffset(), 0.)
        leftMotorPower = movepower - alignpower
        rightMotorPower = movepower + alignpower
        # limit for max power
        leftMotorPower= self.limitPower(leftMotorPower)
        rightMotorPower= self.limitPower(rightMotorPower)
        # send to motors
        self.moveArm(leftMotorPower, rightMotorPower)

