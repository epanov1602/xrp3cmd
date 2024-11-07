#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from __future__ import annotations

import typing

import commands2

from subsystems.drivetrain import Drivetrain
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.trajectory import Trajectory, TrajectoryConfig, TrajectoryGenerator
from wpimath.controller import RamseteController
from wpilib import Timer

class DriveTrajectory(commands2.Command):
    def __init__(
        self,
        drivetrain: Drivetrain,
        endpoint: Pose2d,
        waypoints: typing.List[Translation2d]=(),
        speed=Drivetrain.kMaxSpeed,
        accel=999.9,
        b=2.1,
        zeta=0.7,
    ):
        """

        :param drivetrain: tank drive train
        :param endpoint: end point of the trajectory
        :param waypoints: a list of waypoints, if any
        :param speed: maximum allowed driving speed (can be smaller than max speed of the drivetrain)
        :param accel: maximum allowed acceleration
        :param b: RAMSETE unicycle parameter, how quickly to correct the trajectory (default 2.1)
        :param zeta: RAMSETE unicycle parameter, how much to avoid oversteering (default 0.7)
        """
        super().__init__()
        assert endpoint is not None

        from wpimath.kinematics import DifferentialDriveKinematics, DifferentialDriveWheelSpeeds

        self.kinematics = DifferentialDriveKinematics(Drivetrain.kWheelBaseWidthInch)
        self.wheelSpeeds = DifferentialDriveWheelSpeeds(0, 0)
        self.config = TrajectoryConfig(speed, accel)
        self.config.setKinematics(self.kinematics)
        self.drivetrain = drivetrain
        self.endpoint = endpoint
        self.waypoints = waypoints

        self.controller = RamseteController(b, zeta)
        self.trajectory = None
        self.t0 = None

        self.addRequirements(self.drivetrain)

    def initialize(self):
        location = self.drivetrain.getPose()

        # find the waypoint nearest to the current location: we want to skip all the waypoints before it
        nearest, distance = None, None
        for w in self.waypoints:
            d = location.translation().distance(w)
            if nearest is None or d < distance:
                nearest, distance = w, d

        # only use the waypoints that we must not skip (only those located after the waypoint nearest to us now)
        waypoints = []
        if nearest is None:
            waypoints = self.waypoints
        else:
            skip = True
            for w in self.waypoints:
                if not skip:
                    waypoints.append(w)
                elif w == nearest:
                    skip = False

        # this will be the trajectory
        self.trajectory = TrajectoryGenerator.generateTrajectory(
            start=location,
            interiorWaypoints=waypoints,
            end=self.endpoint,
            config=self.config)

        # the time starts now
        self.t0 = Timer.getFPGATimestamp()


    def execute(self):
        t = Timer.getFPGATimestamp() - self.t0
        goal = self.trajectory.sample(t)  # take the point on the trajectory at t seconds from the beginning
        location = self.drivetrain.getPose()  # compare it with the actual location of the robot now, to compute speeds
        speeds = self.controller.calculate(location, goal.pose, goal.velocity, goal.curvature * goal.velocity)
        self.wheelSpeeds = self.kinematics.toWheelSpeeds(speeds)
        # on XRP there is no good way to add a PID controller for wheel speeds, so using this approximation
        fwd = (self.wheelSpeeds.left + self.wheelSpeeds.right) / (2 * Drivetrain.kMaxSpeed)
        rot = (self.wheelSpeeds.right - self.wheelSpeeds.left) / 2 * 0.0002
        # give these input signals to the drivetrain
        self.drivetrain.arcadeDrive(fwd, rot, square=False)

    def isFinished(self) -> bool:
        t = Timer.getFPGATimestamp() - self.t0
        if t > 1.0 and self.wheelSpeeds.right == 0 and self.wheelSpeeds.left == 0:
            return True

    def end(self, interrupted: bool):
        self.drivetrain.arcadeDrive(0, 0, square=False)
