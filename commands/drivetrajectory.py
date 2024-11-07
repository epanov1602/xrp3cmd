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
    def __init__(self, drivetrain, endpoint: Pose2d, waypoints: typing.List[Translation2d]=(), speed=18., accel=999.):
        super().__init__()
        from wpimath.kinematics import DifferentialDriveKinematics

        self.drivetrain = drivetrain
        self.kinematics = DifferentialDriveKinematics(Drivetrain.kWheelBaseWidthInch)
        self.config = TrajectoryConfig(speed, accel)
        self.config.setKinematics(self.kinematics)
        self.endpoint = endpoint
        self.waypoints = waypoints
        assert self.endpoint is not None

        self.controller = RamseteController(2.1, 0.7)
        self.trajectory: Trajectory = None
        self.start = None

        self.addRequirements(self.drivetrain)

    def initialize(self):
        location = self.drivetrain.getPose()

        # find the waypoint nearest to the current location: we want to skip all the waypoints before it
        nearest, distance = None, None
        for w in self.waypoints:
            d = location.translation().distance(w)
            if nearest is None or d < distance:
                nearest, distance = w, d

        # only add the waypoints that we must not skip
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

        # An example trajectory to follow
        self.trajectory = TrajectoryGenerator.generateTrajectory(
            start=location,
            interiorWaypoints=waypoints,
            end=self.endpoint,
            config=self.config)

        self.start = Timer.getFPGATimestamp()

    def execute(self):
        t = Timer.getFPGATimestamp() - self.start
        goal = self.trajectory.sample(t)  # sample the trajectory at t seconds from the beginning
        location = self.drivetrain.getPose()
        speeds = self.controller.calculate(location, goal.pose, goal.velocity, goal.curvature * goal.velocity)
        wheel_speeds = self.kinematics.toWheelSpeeds(speeds)
        fwd = 0.05 * (wheel_speeds.left + wheel_speeds.right) / 2
        rot = 0.0002 * (wheel_speeds.right - wheel_speeds.left) / 2
        self.drivetrain.arcadeDrive(fwd, rot, square=False)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool):
        self.drivetrain.arcadeDrive(0, 0, square=False)
