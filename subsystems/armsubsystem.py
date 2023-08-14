# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import wpilib
import commands2
import wpimath.controller
import wpimath.trajectory

import rev

import constants

class ArmSubsystemPID(commands2.ProfiledPIDSubsystem):
  """A robot arm subsystem that moves with a motion profile."""

  # Create a new ArmSubsystem
  def __init__(self) -> None:
    super().__init__(
      wpimath.controller.ProfiledPIDController(
        constants.ArmConstants.kP,
        0,
        0,
        wpimath.trajectory.TrapezoidProfile.Constraints(
          constants.ArmConstants.kMaxVelocityRadPerSecond,
          constants.ArmConstants.kMaxAccelerationRadPerSecSquared,
        ),
      ),
      0,
    )

    self.leftmotor = rev.CANSparkMax(constants.ArmConstants.kMotorPort)
    self.rightmotor = rev.CANSparkMax(constants.ArmConstants.kMotorPort)
    self.leftencoder = self.motor.getEncoder()
    self.rightencoder = self.motor.getEncoder()
    self.leftencoder.set(0.)
    self.rightencoder.set(0.)

    self.feedforward = wpimath.controller.ArmFeedforward(
      constants.ArmConstants.kSVolts,
      constants.ArmConstants.kGVolts,
      constants.ArmConstants.kVVoltSecondPerRad,
      constants.ArmConstants.kAVoltSecondSquaredPerRad,
    )

#   self.encoder.setDistancePerPulse(
#     constants.ArmConstants.kEncoderDistancePerPulse
#   )

    # Start arm at rest in neutral position
    self.setGoal(constants.ArmConstants.kArmOffsetRads)

    def _useOutput(self,
      output: float, setpoint: wpimath.trajectory.TrapezoidProfile.State) -> None:

      # Calculate the feedforward from the setpoint
      feedforward = self.feedforward.calculate(setpoint.position, setpoint.velocity)

      # Add the feedforward to the PID output to get the motor output
      self.motor.setVoltage(output + feedforward)

    def _getMeasurement(self) -> float:
      return self.encoder.getDistance() + constants.ArmConstants.kArmOffsetRads

