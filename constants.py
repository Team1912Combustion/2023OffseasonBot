
import math

class DriveConstants:
    kLeftMotor1Port = 2
    kLeftMotor2Port = 3
    kRightMotor1Port = 0
    kRightMotor2Port = 1

    kLeftEncoderPorts = (2, 3)
    kRightEncoderPorts = (0, 1)
    kLeftEncoderReversed = False
    kRightEncoderReversed = True

    kEncoderCPR = 1024
    kWheelDiameterInches = 6

    # Assumes the encoders are directly mounted on the wheel shafts
    kEncoderDistancePerPulse = (kWheelDiameterInches * math.pi) / kEncoderCPR

    kGyroReversed = False

    kStabilizationP = 1
    kStabilizationI = 0.5
    kStabilizationD = 0

    kTurnP = 1
    kTurnI = 0
    kTurnD = 0

    kMaxTurnRateDegPerS = 100
    kMaxTurnAccelerationDegPerSSquared = 300

    kTurnToleranceDeg = 5
    kTurnRateToleranceDegPerS = 10  # degrees per second

class OIConstants:
    kDriverControllerPort = 0
    kDriverRightBumper = 7
    kDriverLeftBumper = 6
    kDriverXbutton = 0
    kDriverYbutton =  3
