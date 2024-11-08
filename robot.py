#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

# Run the program
# ---------------
#
# To run the program you will need to explicitly use the ws-client option:
#
#    # Windows
#    py -3 -m robotpy sim --xrp
#
#    # Linux/macOS
#    python -m robotpy sim --xrp
#
# By default the WPILib simulation GUI will be displayed. To disable the display
# you can add the --nogui option
#

import os
import typing

import wpilib
import commands2

from robotcontainer import RobotContainer

NO_ROBOT = os.getenv("XRP3_NO_ROBOT", "0") != "0"
NO_CAMERA = os.getenv("XRP3_NO_CAMERA", "0") != "0"

def find_xrp_ip_address(attempts=3):
    if NO_ROBOT:
        return ""

    import requests

    for attempt in range(attempts):
        for ip in [
            "192.168.42.1",
            "192.168.42.22",
            "192.168.42.23",
            "192.168.42.24",
            "192.168.42.25",
            "192.168.42.25",
        ]:
            try:
                r = requests.get(f'http://{ip}:5000', timeout=3)
                r.raise_for_status()  # Raises a HTTPError if the status is 4xx, 5xxx
                return ip
            except (requests.exceptions.ConnectionError, requests.exceptions.Timeout) as e:
                print(f"Robot not available to connect at {ip}: {e}")

    raise Exception("Cannot find an XRP robot at any IP addresses tried. Are we connected to the right WiFi network?")


# If your XRP isn't at the default port, set that here
os.environ["HALSIMXRP_PORT"] = "3540"
os.environ["HALSIMXRP_HOST"] = find_xrp_ip_address()


class MyRobot(commands2.TimedCommandRobot):
    """
    Command v2 robots are encouraged to inherit from TimedCommandRobot, which
    has an implementation of robotPeriodic which runs the scheduler for you
    """

    autonomousCommand: typing.Optional[commands2.Command] = None

    def robotInit(self) -> None:
        """
        This function is run when the robot is first started up and should be used for any
        initialization code.
        """

        # Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        # autonomous chooser on the dashboard.
        self.container = RobotContainer(NO_ROBOT or NO_CAMERA)

    def disabledInit(self) -> None:
        """This function is called once each time the robot enters Disabled mode."""

    def disabledPeriodic(self) -> None:
        """This function is called periodically when disabled"""

    def autonomousInit(self) -> None:
        """This autonomous runs the autonomous command selected by your RobotContainer class."""
        self.autonomousCommand = self.container.getAutonomousCommand()

        if self.autonomousCommand:
            self.autonomousCommand.schedule()

    def autonomousPeriodic(self) -> None:
        """This function is called periodically during autonomous"""

    def teleopInit(self) -> None:
        # This makes sure that the autonomous stops running when
        # teleop starts running. If you want the autonomous to
        # continue until interrupted by another command, remove
        # this line or comment it out.
        if self.autonomousCommand:
            self.autonomousCommand.cancel()
        self.container.teleopInit()

    def teleopPeriodic(self) -> None:
        """This function is called periodically during operator control"""

    def testInit(self) -> None:
        # Cancels all running commands at the start of test mode
        commands2.CommandScheduler.getInstance().cancelAll()
