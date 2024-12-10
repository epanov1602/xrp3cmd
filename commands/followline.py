import commands2
from subsystems.drivetrain import Drivetrain
from wpilib import SmartDashboard


class FollowLine(commands2.Command):

    def __init__(self, drivetrain: Drivetrain):
        super().__init__()
        self.addRequirements(drivetrain)

        self.drivetrain = drivetrain
        self.finished = False

    def initialize(self):
        self.finished = False

    def execute(self):
        # get the measurements from the reflectance sensor
        sensor = self.drivetrain.reflectanceSensor
        left = sensor.getLeftReflectanceValue()
        right = sensor.getRightReflectanceValue()

        # 1. if one of the sensors gives us invalid measurement, we must stop
        if left <= 0.0 or left >= 1.0 or right <= 0.0 or right >= 1.0:
            SmartDashboard.putString("follow-status", f"stop: invalid refl {left}, {right}")
            self.finished = True
            return

        # 2. if both sensors show bright reflectance, we conclude that we don't see any dark line anymore
        if left >= 0.76 and right >= 0.76:
            SmartDashboard.putString("follow-status", f"stop: no more line {left}, {right}")
            self.finished = True
            return

        # 3. is one side darker than the other? (turn that way)
        diff = left - right
        # now, if diff is massively >0, we should be turning back to the right quickly (and vice versa)
        # for example, if left reflectance = 0.8 and right = 0.4 this means left sensor is off the line and right is on

        # TODO: actually write this code here

        # 3. otherwise, use 'right' and 'left' reflectance to decide which way to drive
        if diff > +0.15:
            # it looks brighter on the left side: need to turn back to the right
            self.drivetrain.arcadeDrive(fwd=0.13, rot=-0.2)
            SmartDashboard.putString("follow-status", f"turning right")
        elif diff < -0.15:
            # it looks brighter on the right side: need to turn back to the left
            self.drivetrain.arcadeDrive(fwd=0.13, rot=+0.2)
            SmartDashboard.putString("follow-status", f"turning left")
        else:
            # it is neither bright on the left nor on the right: we can go forward full steam
            self.drivetrain.arcadeDrive(fwd=0.4, rot=0.0)
            SmartDashboard.putString("follow-status", f"going forward")

    def end(self, interrupted: bool):
        self.drivetrain.stop()

    def isFinished(self) -> bool:
        if self.finished == True:
            return True
