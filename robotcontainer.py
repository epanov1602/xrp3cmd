#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import typing
import cv2
import pupil_apriltags as apriltags

import wpilib
from wpimath.geometry import Translation2d, Pose2d, Rotation2d

import commands2
from commands2.button import CommandXboxController
from commands2 import InstantCommand, WaitCommand

from commands.arcadedrive import ArcadeDrive
from commands.drivedistance import DriveDistance
from commands.rotateangle import RotateAngle

from subsystems.drivetrain import Drivetrain
from subsystems.cvcamera import CVCamera
from subsystems.arm import Arm
from subsystems.stopwatch import Stopwatch

from commands.drivetrajectory import DriveTrajectory
from commands.followobject import FollowObject, StopWhen
from commands.findobject import FindObject
from commands.aimtodirection import AimToDirection
from commands.gotopoint import GoToPoint
from commands.followline import FollowLine

from helpers import detection

class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self, nocamera=False):
        # The robot's subsystems are defined here
        self.drivetrain = Drivetrain()
        self.arm = Arm()
        self.stopwatch = Stopwatch("race-time")

        ## 1. detector for faces?
        face_detector_model = cv2.CascadeClassifier('resources/haarcascade_frontalface_default.xml')
        def face_detector(frame, tracker, previous_bbox):
           return detection.detect_biggest_face(face_detector_model, frame, previous_bbox, tracker)

        ## 2. detector for apriltags?
        apriltag_detector_model = apriltags.Detector(families="tag36h11", quad_sigma=0.0, quad_decimate=1.0, decode_sharpening=0.7)
        def apriltag_detector(frame, tracker, previous_bbox, only_these_ids=None):
            return detection.detect_biggest_apriltag(apriltag_detector_model, frame, only_these_ids, tracker)

        ## 3. detector for orange tennis balls?
        def tennis_ball_detector(frame, tracker, previous_bbox, color_hue_range=(5, 15), smallest_size_px=7):
            return detection.detect_biggest_ball(
                frame, color_hue_range, smallest_size_px, detectors="both", previous_xywh=previous_bbox, tracker=tracker,
            )

        ## 4. YOLO detector for objects from Microsoft COCO dataset
        ## to make this work, you need to run "pip install ultralytics" in your terminal
        #from ultralytics import YOLO
        #gamepiece_detector_model = YOLO("yolov10m.pt")
        #def gamepiece_detector(frame, tracker, previous_bbox, classes=["sports ball", "cell phone",]):
        #    return detection.detect_yolo_object(gamepiece_detector_model, frame, valid_classes=classes, tracker=tracker)

        self.camera = CVCamera(150, 120, 10, detector=tennis_ball_detector)
        if nocamera:
            #self.camera.start(0)  # camera of your laptop
            print("not starting camera at all")
        else:
            self.camera.start("http://192.168.42.21:81/stream")  # default XRP camera URL

        # Assume that joystick "j0" is plugged into channel 0
        self.j0 = CommandXboxController(0)
        # (you can also use CommandPS4Controller or CommandJoystick, if you prefer those)

        self.configureButtonBindings()

    def configureButtonBindings(self):
        """Use this method to define your button->command mappings"""

        follow_object = FollowObject(self.camera, self.drivetrain, fwd_step_seconds=0)
        self.j0.b().whileTrue(follow_object)

        # 1. Here is a command to drive forward 10 inches with speed 0.9
        forward30inches = GoToPoint(x=30, y=0, drivetrain=self.drivetrain)
        # let's bind this command to button "y" on the joystick
        self.j0.y().onTrue(forward30inches)

        # and here is a command to drive back 10 inches

        back30inches = GoToPoint(x=-30, y=0, drivetrain=self.drivetrain)
        self.j0.a().onTrue(back30inches)

        #  - exercise 1: can you hook this command to button "a" on the joystick?


        # 3. Instant commands (commands that just do one thing instantly)
        # normally, simple one-shot commands that don't need to be written as separate modules in commands/ directory

        # "lambda" really means "do this later when that command needs to run"
        arm_up = commands2.InstantCommand(lambda: self.arm.setAngle(90))
        self.j0.x().onTrue(arm_up)  # - bind it to button "x" pressed

        # a command for "arm down" can bind to button x "unpressed" ???
        arm_down = commands2.InstantCommand(lambda: self.arm.setAngle(0))
        # yes! we can bind it to "button unpressed" event, if we use "onFalse()"
        self.j0.x().onFalse(arm_down)

        #  - exercise 3: can you make an "arm half up" (45 degrees) command and bind it to "B button pressed"?

        # (and is anything missing?)


        # 4. A command to turn right 45 degrees *but* we can add a 5 second timeout to it
        right45degrees = AimToDirection(degrees=-45, drivetrain=self.drivetrain)
        self.j0.rightBumper().onTrue(right45degrees)

        # exercise 4: can you make a command to turn the robot left by 45 degrees and with 3 second timeout?
        left45degrees = AimToDirection(degrees= 45, drivetrain=self.drivetrain)
        self.j0.leftBumper().onTrue(left45degrees)

        # exercise 4b: can you bind this command to the left bumper button of the joystick?


        # 5. Connecting commands together: making a half square
        forward8inches1 = DriveDistance(speed=0.7, inches=8, drivetrain=self.drivetrain)
        right90degrees1 = RotateAngle(speed=0.5, degrees=90, drivetrain=self.drivetrain)
        forward8inches2 = DriveDistance(speed=0.7, inches=8, drivetrain=self.drivetrain)
        right90degrees2 = RotateAngle(speed=0.5, degrees=90, drivetrain=self.drivetrain)
        half_square = forward8inches1.andThen(right90degrees1).andThen(forward8inches2).andThen(right90degrees2)
        self.j0.povDown().onTrue(half_square)

        # exercise 5: can you actually change the code above to make it a full square?


        # 6. A little helper instant command to reset the robot coordinates in SmartDashboard
        reset_coordinates = commands2.InstantCommand(lambda: self.drivetrain.resetOdometry())
        self.j0.povUp().onTrue(reset_coordinates)

        # 7. Finally, a command to take input from joystick *later* ("lambda" = later)
        # and drive using that input as control speed signal
        drive = ArcadeDrive(
            self.drivetrain,
            lambda: -self.j0.getRawAxis(1),  # minus sign, because Xbox stick pushed forward is negative axis value
            lambda: -self.j0.getRawAxis(0),
        )
        # This command will be running *by default* on drivetrain
        # ("by default" means it will stop running when some other command is asked
        # to use drivetrain, and will restart running after that other command is done)
        self.drivetrain.setDefaultCommand(drive)

    def getAutonomousCommand(self):
        # make the "get to line" command
        driveForward = DriveDistance(speed=0.3, inches=50, drivetrain=self.drivetrain)
        getToLine = driveForward.until(self.drivetrain.isAboveTape)

        # make the "follow line" command
        followLine = FollowLine(drivetrain=self.drivetrain)

        # connect these two commands together
        auto0 = getToLine.andThen(followLine)
        return auto0

        #startStopwatch = InstantCommand(self.stopwatch.start)
        #stopStopwatch = InstantCommand(self.stopwatch.stop)
        #resetOdometry0 = InstantCommand(self.drivetrain.resetOdometry)

        # option 0: use points
        pointA = GoToPoint(97, 0, self.drivetrain)
        pointB = GoToPoint(0, 0, self.drivetrain)
        auto0 = pointA.andThen(pointB)


        # option 1: use trajectories (here it goes to 120 inches ahead, but follows the waypoints for an S curve)
        trajectoryA = DriveTrajectory(
            self.drivetrain,
            endpoint=Pose2d(80, 0, Rotation2d.fromDegrees(0)),
            waypoints=[Translation2d(10, 0), Translation2d(30, 30), Translation2d(30, -30)],
        )
        trajectoryB = DriveTrajectory(
            self.drivetrain,
            endpoint=Pose2d(0, 0, Rotation2d.fromDegrees(180)),
            waypoints=[Translation2d(70, 0), Translation2d(30, -30), Translation2d(30, 30)],
        )
        auto1 = trajectoryA.andThen(trajectoryB)


        # option 2: use visual navigation -- find object and follow object, then find next one etc.
        resetOdometry2 = InstantCommand(self.drivetrain.resetOdometry)

        findA = FindObject(self.camera, self.drivetrain, step_degrees=-15)
        followA = FollowObject(self.camera, self.drivetrain, stop_when=StopWhen(maxSize=26))
        chargeA = DriveDistance(speed=1.0, inches=10, drivetrain=self.drivetrain)

        findB = FindObject(self.camera, self.drivetrain, step_degrees=15)
        followB = FollowObject(self.camera, self.drivetrain, stop_when=StopWhen(maxSize=26))
        chargeB = DriveDistance(speed=1.0, inches=10, drivetrain=self.drivetrain)

        auto2 = (resetOdometry2
                 .andThen(findA).andThen(followA).andThen(chargeA)
                 .andThen(findB).andThen(followB).andThen(chargeB)
                 )
        return auto2

        turn = FindObject(self.camera, self.drivetrain, step_degrees=-15)
        findobject = FollowObject(self. camera, self.drivetrain, stop_when=StopWhen(maxSize=15))
        auto3 = (resetOdometry2
                 .andThen(findobject).andThen(FollowObject) .andThen(findobject)
                 .andThen(findobject) .andThen(FollowObject) .andThen(findobject)
                 )

        # which auto will you use?
        return auto0

    def teleopInit(self):
        self.drivetrain.resetOdometry()

    def autonomousInit(self):
        self.drivetrain.resetOdometry()
