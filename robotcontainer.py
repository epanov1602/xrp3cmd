#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import typing
import cv2
import pupil_apriltags as apriltags

import wpilib
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

from commands.followobject import FollowObject, StopWhen
from commands.aimtodirection import AimToDirection
from commands.gotopoint import GoToPoint
from helpers import detection

class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self):
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

        self.camera = CVCamera(80, 60, 10)  #, detector=apriltag_detector)
        self.camera.start()

        # Assume that joystick "j0" is plugged into channel 0
        self.j0 = CommandXboxController(0)
        # (you can also use CommandPS4Controller or CommandJoystick, if you prefer those)

        self.configureButtonBindings()

    def configureButtonBindings(self):
        """Use this method to define your button->command mappings"""

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

        stopWhen = StopWhen(aimingToleranceDegrees=0.0001)
        self.j0.button(1).onTrue(FollowObject(self.camera, self.drivetrain))

    def getAutonomousCommand(self):
        resetOdometry = InstantCommand(self.drivetrain.resetOdometry)
        startStopwatch = InstantCommand(self.stopwatch.start)
        stopStopwatch = InstantCommand(self.stopwatch.stop)

        # a little race with stopwatch
        autoCommand = (resetOdometry
                       .andThen(startStopwatch)
                       .andThen(GoToPoint(35, 0, self.drivetrain, 1.0, slowDownAtFinish=False))
                       .andThen(GoToPoint(40.5, 5, self.drivetrain, 1.0, slowDownAtFinish=False))
                       .andThen(GoToPoint(37, 15, self.drivetrain, 1.0, slowDownAtFinish=True))

                       .andThen(GoToPoint(-2, 39, self.drivetrain, 1.0, slowDownAtFinish=True))
                       .andThen(GoToPoint(0, 0, self.drivetrain, 1.0, slowDownAtFinish=True))
                       .andThen(stopStopwatch))

        return autoCommand

    def teleopInit(self):
        self.drivetrain.resetOdometry()
