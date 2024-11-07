#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math

import commands2
import wpilib
import xrp

from wpimath.kinematics import DifferentialDriveOdometry
from wpimath.geometry import Rotation2d, Pose2d, Translation2d
from wpilib import SmartDashboard, Timer


class Drivetrain(commands2.Subsystem):
    kCountsPerRevolution = 585.0
    kWheelDiameterInch = 2.3622
    kWheelBaseWidthInch = 10.0  # is it 10 inches?
    kMinProductiveEffort = 0.4  # control signal smaller than this might not result in XRP motor spinning
    kMaxSpeed = 18  # inches per second

    def __init__(self, maxAcceleration: float = 999) -> None:
        super().__init__()
        self.leftSpeed = 0
        self.rightSpeed = 0
        self.maxAcc = maxAcceleration

        # The XRP has the left and right motors set to
        # PWM channels 0 and 1 respectively
        self.leftMotor = xrp.XRPMotor(0)
        self.rightMotor = xrp.XRPMotor(1)
        self.rightMotor.setInverted(True)

        # The XRP has onboard encoders that are hardcoded
        # to use DIO pins 4/5 and 6/7 for the left and right
        self.leftEncoder = wpilib.Encoder(4, 5)
        self.rightEncoder = wpilib.Encoder(6, 7)

        # And an onboard gyro (and you have to power on your XRP when it is on flat surface)
        self.gyro = xrp.XRPGyro()
        self.accelerometer = wpilib.BuiltInAccelerometer()
        self.reflectanceSensor = xrp.XRPReflectanceSensor()
        self.distanceSensor = xrp.XRPRangefinder()

        # Use inches as unit for encoder distances
        self.leftEncoder.setDistancePerPulse(
            (math.pi * self.kWheelDiameterInch) / self.kCountsPerRevolution
        )
        self.rightEncoder.setDistancePerPulse(
            (math.pi * self.kWheelDiameterInch) / self.kCountsPerRevolution
        )
        self.resetEncoders()
        self.resetGyro()

        # Set up the differential drive controller and differential drive odometry
        self.odometry = DifferentialDriveOdometry(
            Rotation2d.fromDegrees(self.getGyroAngleZ()), self.getLeftDistanceInch(), self.getRightDistanceInch())

    def periodic(self) -> None:
        heading = Rotation2d.fromDegrees(self.getGyroAngleZ())
        pose = self.odometry.update(heading, self.getLeftDistanceInch(), self.getRightDistanceInch())
        SmartDashboard.putNumber("distance-to-obst", self.getDistanceToObstacle())
        SmartDashboard.putNumber("left-reflect", self.reflectanceSensor.getLeftReflectanceValue())
        SmartDashboard.putNumber("right-reflect", self.reflectanceSensor.getRightReflectanceValue())
        SmartDashboard.putNumber("x", pose.x)
        SmartDashboard.putNumber("y", pose.y)
        SmartDashboard.putNumber("z-heading", pose.rotation().degrees())


    def arcadeDrive(self, fwd: float, rot: float, square: bool = False) -> None:
        """
        Drives the robot using arcade controls.

        :param fwd: the commanded forward movement
        :param rot: the commanded rotation
        :param square: make the inputs a little smoother around zero (helps human operators)
        """
        # 1. compute the desired wheel speeds, without accounting for max acceleration
        if square:
            rot = rot * abs(rot)
            fwd = fwd * abs(fwd)
        desiredLeftSpeed, desiredRightSpeed = _to_left_right_speeds(fwd, rot)

        # 2. adjust the desired wheel speeds for allowed max acceleration (to avoid skidding on the floor)
        self.leftSpeed = _clip(desiredLeftSpeed, self.leftSpeed - self.maxAcc, self.leftSpeed + self.maxAcc)
        self.rightSpeed = _clip(desiredRightSpeed, self.rightSpeed - self.maxAcc, self.rightSpeed + self.maxAcc)

        # 3. set the motors to proceed with those speeds
        t = Timer.getFPGATimestamp()
        self.leftMotor.set(_protect_from_min_motor_speed(self.leftSpeed, t))
        self.rightMotor.set(_protect_from_min_motor_speed(self.rightSpeed, t))

    def stop(self) -> None:
        """
        Stop the drivetrain motors immediately, without respecting maxAcceleration
        """
        self.leftSpeed = 0
        self.rightSpeed = 0
        self.arcadeDrive(0, 0)

    def resetEncoders(self) -> None:
        """Resets the drive encoders to currently read a position of 0."""
        self.leftEncoder.reset()
        self.rightEncoder.reset()

    def getLeftEncoderCount(self) -> int:
        return self.leftEncoder.get()

    def getRightEncoderCount(self) -> int:
        return self.rightEncoder.get()

    def getLeftDistanceInch(self) -> float:
        return self.leftEncoder.getDistance()

    def getRightDistanceInch(self) -> float:
        return self.rightEncoder.getDistance()

    def getAverageDistanceInch(self) -> float:
        """Gets the average distance of the TWO encoders."""
        return (self.getLeftDistanceInch() + self.getRightDistanceInch()) / 2.0

    def getAccelX(self) -> float:
        """The acceleration in the X-axis.

        :returns: The acceleration of the XRP along the X-axis in Gs
        """
        return self.accelerometer.getX()

    def getAccelY(self) -> float:
        """The acceleration in the Y-axis.

        :returns: The acceleration of the XRP along the Y-axis in Gs
        """
        return self.accelerometer.getY()

    def getAccelZ(self) -> float:
        """The acceleration in the Z-axis.

        :returns: The acceleration of the XRP along the Z-axis in Gs
        """
        return self.accelerometer.getZ()

    def getGyroVelocityZ(self) -> float:
        """The angular velocity in the Z-axis.

        :returns: The acceleration of the XRP along the Z-axis in Gs
        """
        return self.gyro.getRateZ()

    def getGyroAngleX(self) -> float:
        """Current angle of the XRP around the X-axis.

        :returns: The current angle of the XRP in degrees
        """
        return self.gyro.getAngleX()

    def getGyroAngleY(self) -> float:
        """Current angle of the XRP around the Y-axis.

        :returns: The current angle of the XRP in degrees
        """
        return self.gyro.getAngleY()

    def getGyroAngleZ(self) -> float:
        """Current angle of the XRP around the Z-axis.

        :returns: The current angle of the XRP in degrees
        """
        return self.gyro.getAngleZ()

    def getDistanceToObstacle(self) -> float:
        """Distance to obstacle in the front, as given by the distance sensor

        :returns: Distance in meters, values >0.5 are not very reliable and are replaced with nan.
        """
        distance = self.distanceSensor.getDistance()
        return distance if distance < 0.5 else math.nan

    def getPose(self) -> Pose2d:
        return self.odometry.getPose()

    def getLocation(self) -> Translation2d:
        return self.getPose().translation()

    def getHeading(self) -> Rotation2d:
        return self.getPose().rotation()

    def resetGyro(self) -> None:
        """Reset the gyro"""
        self.gyro.reset()

    def resetOdometry(self, pose: Pose2d = Pose2d()) -> None:
        self.resetGyro()
        self.resetEncoders()
        heading = Rotation2d.fromDegrees(self.getGyroAngleZ())
        self.odometry.resetPosition(heading, self.getLeftDistanceInch(), self.getRightDistanceInch(), pose)

    def resetPose(self, pose: Pose2d = Pose2d()) -> None:
        self.resetOdometry(pose)

def _clip(x, minimum, maximum):
    if x > maximum:
        x = maximum
    if x < minimum:
        x = minimum
    return x

def _protect_from_min_motor_speed(speed, t, min_motor_speed=Drivetrain.kMinProductiveEffort, period_seconds=0.25):
    # 1. if speed is zero or above min_motor_speed, just use that
    if speed == 0 or abs(speed) > min_motor_speed:
        return speed
    # 2. if speed is smaller than min_motor_speed, do something else:
    #  - use +-min_motor_speed but only a certain % of the time
    #  - and use zero speed otherwise
    probability = t / period_seconds
    probability = probability - int(probability)  # floating point number between 0.0 and 0.999999
    if min_motor_speed * probability < abs(speed):
        return math.copysign(min_motor_speed, speed)
    else:
        return 0

def _to_left_right_speeds(fwd, rot):
    rot = _clip(rot, -1.0, +1.0)
    max_fwd = 1.0 - abs(rot)  # maximum achievable forward effort without spinning one of two motors at >100%
    fwd = _clip(fwd, -max_fwd, +max_fwd)
    left = fwd - rot
    right = fwd + rot
    # we also need to protect them from going under min motor speed
    return left, right