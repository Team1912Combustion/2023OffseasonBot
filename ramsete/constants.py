"""
A place for the constant values in the code that may be used in more than one place. 
This offers a convenient resources to teams who need to make both quick and universal
changes.
"""

from wpimath.kinematics import DifferentialDriveKinematics

import math

# ID for the driver's joystick.
kDriverControllerPort = 0

# The PWM IDs for the drivetrain motor controllers.
kLeftMotor1Port = 1
kRightMotor1Port = 0

# Encoders and their respective motor controllers.
kLeftEncoderPorts = (2, 3)
kRightEncoderPorts = (0, 1)
kLeftEncoderReversed = False
kRightEncoderReversed = True

# In meters, distance between wheels on each side of robot.
kTrackWidthMeters = 0.60
kDriveKinematics = DifferentialDriveKinematics(kTrackWidthMeters)

# Encoder counts per revolution/rotation.
kEncoderCPR = 360
kWheelDiameterMeters = 0.152

# The following works assuming the encoders are directly mounted to the wheel shafts.
kEncoderDistancePerPulse = (kWheelDiameterMeters * math.pi) / kEncoderCPR

# NOTE: Please do NOT use these values on your robot. Rather, characterize your
# drivetrain using the FRC Characterization tool. These are for demo purposes
# only!
ksVolts = 2.155
kvVoltSecondsPerMeter = 2.5377
kaVoltSecondsSquaredPerMeter = 1.1553

# The P gain for our turn controllers.
kPDriveVel = 1

# The max velocity and acceleration for our autonomous.
kMaxSpeedMetersPerSecond = .5
kMaxAccelerationMetersPerSecondSquared = .5

# Baseline values for a RAMSETE follower in units of meters
# and seconds. These are recommended, but may be changes if wished.
kRamseteB = 2
kRamseteZeta = 0.7

# The number of motors on the robot.
kDrivetrainMotorCount = 4
