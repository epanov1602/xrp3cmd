import commands2
import typing

from commands.aimtodirection import AimToDirection
from commands.drivedistance import DriveDistance

from subsystems.drivetrain import Drivetrain
from subsystems.cvcamera import CVCamera
from wpimath.geometry import Rotation2d


class StopWhen:
    """
    How close is "close enough", for a given object-following command?
    """
    def __init__(self, maxY=999, minY=-999, maxSize=9999, aimingToleranceDegrees=4):
        """
        When to stop object following
        :param maxY: if the "Y" (pitch) of the object is above this, finish
        :param minY: if the "Y" (pitch) of the object is below this, finish
        :param maxSize: if the angular size of the object is greater than this, finish
        :param aimingToleranceDegrees: if we aren't approaching but simply aiming (fwd_step=0), how close is enough?
        """
        self.maxY = maxY
        self.minY = minY
        self.maxSize = maxSize
        self.aimingToleranceDegrees = aimingToleranceDegrees


class FollowObject(commands2.Command):
    ANGLE_TOLERANCE = 30  # if pointing further away than this, do not move forward (but rotate to the object first)

    def __init__(self, camera: CVCamera, drivetrain: Drivetrain, fwd_step_seconds=0.25, stop_when: StopWhen=None):
        super().__init__()

        self.targetCamera = camera
        self.stopWhen = stop_when
        self.fwdStepSeconds = fwd_step_seconds
        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)

        self.finished = False
        self.targetDirection = None
        self.minDetectionIndex = None
        self.subcommand: commands2.Command = None

    def initialize(self):
        self.finished = False
        self.targetDirection = None
        self.minDetectionIndex = None

    def execute(self):
        # 1. if there is subcommand to go in some direction, just work on executing it
        if self.subcommand is not None:
            if self.subcommand.isFinished():
                self.subcommand.end(False)  # if subcommand is finished, we must end() it
                self.subcommand = None  # and we don't have it anymore
            else:
                self.subcommand.execute()  # otherwise, the subcommand must run

        # 2. otherwise, if target direction is at least known, make that subcommand to go in that direction
        elif self.targetDirection is not None:
            self.subcommand = self.makeSubcommandToGoInTargetDirection()

        # 3. but if target direction is not known at all, look at the camera and find in which target direction to go
        else:
            self.tryGetTargetDirection()

    def end(self, interrupted: bool):
        if self.subcommand is not None:
            self.subcommand.end(interrupted)
            self.subcommand = None
        self.drivetrain.arcadeDrive(0, 0)

    def isFinished(self) -> bool:
        if self.subcommand:
            return False  # if subcommand is here, it is not finished yet
        if self.finished:
            return True  # otherwise, if we are thinking we are finished, then we are

    def makeSubcommandToGoInTargetDirection(self):
        degreesFromTarget = (self.drivetrain.getHeading() - self.targetDirection).degrees()
        self.targetDirection = None  # target direction will need to be recalculated after subcommand stops
        self.minDetectionIndex = None

        if abs(degreesFromTarget) < FollowObject.ANGLE_TOLERANCE and self.fwdStepSeconds > 0:
            # 1. if robot is mostly aiming in correct direction already, just make a step in that direction
            drive = AimToDirection(self.targetDirection.degrees(), self.drivetrain, fwd_speed=1.0)
            newSubcommand = drive.withTimeout(self.fwdStepSeconds)  # add a correct timeout for the step
        else:
            # 2. rotate if robot is pointing too far from the object or if we aren't supposed to make steps forward
            newSubcommand = AimToDirection(self.targetDirection.degrees(), self.drivetrain)

        newSubcommand.initialize()
        return newSubcommand

    def tryGetTargetDirection(self):
        # 1. do we have a freshly detected object from the camera
        t, index, (x, y), size = self.targetCamera.get_detected_object()
        if self.minDetectionIndex is None:
            self.minDetectionIndex = index + 1
            return  # we don't know if we are looking at an old video frame or fresh one => try again at the next frame
        elif index < self.minDetectionIndex:
            return  # not yet, we are still looking at an old frame
        elif x is None:
            if index > self.minDetectionIndex + 50 and self.stopWhen is not None:
                self.finished = True  # no hope: object not detected after looking at >50 frames, and we have a stopWhen
            return

        # 2. if that object was freshly detected, is that close enough for the command to finish?
        if self.stopWhen is not None:
            if max(size) > self.stopWhen.maxSize or y > self.stopWhen.maxY or y < self.stopWhen.minY:
                self.finished = True  # looks like the object is very close now, time to finish
                return
            if self.fwdStepSeconds == 0 and abs(x) < self.stopWhen.aimingToleranceDegrees:
                self.finished = True  # aiming at it pretty well and not allowed to move to it
                return

        # 3. otherwise we are not done: pick a target direction for the robot to go
        currentDirection = self.drivetrain.getHeading()
        directionToObjectCenter = Rotation2d.fromDegrees(-x)
        self.targetDirection = currentDirection.rotateBy(directionToObjectCenter)

