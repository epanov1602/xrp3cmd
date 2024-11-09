import commands2
from commands2 import WaitCommand

from commands.rotateangle import RotateAngle

from subsystems.drivetrain import Drivetrain
from subsystems.cvcamera import CVCamera


class FindObject(commands2.Command):
    """
    Keep robot turning by a certain number of degrees at a time (positive or negative),
    until an object is found by the camera
    """

    def __init__(self, camera: CVCamera, drivetrain: Drivetrain, step_degrees=+15, pause_seconds=1.0):
        super().__init__()
        assert pause_seconds > 0
        assert step_degrees != 0

        self.camera0 = camera
        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)

        speed = +1.0 if step_degrees > 0 else -1.0
        self.subcommand = WaitCommand(pause_seconds).andThen(RotateAngle(speed, abs(step_degrees), self.drivetrain))
        self.minDetectionIndex = None

    def initialize(self):
        self.minDetectionIndex = None
        self.subcommand.initialize()

    def execute(self):
        # keep running the turn-and-pause subcommand and restarting if it finished
        if not self.subcommand.isFinished():
            self.subcommand.execute()
        else:
            self.subcommand.end(False)
            self.subcommand.initialize()  # and start it again

    def end(self, interrupted: bool):
        self.subcommand.end(interrupted)
        self.drivetrain.arcadeDrive(0, 0)

    def isFinished(self) -> bool:
        if self.isObjectDetectedOnCamera():
            return True

    def isObjectDetectedOnCamera(self):
        # do we have a freshly detected object from the camera?
        t, index, (x, y), size = self.camera0.get_detected_object()
        # ^^ this only works if camera0 is CVCamera, but if you have Limelight/PhotonVision/something else, see below:
        #
        # - if you are using a Limelight camera, you can use code from
        #     https://github.com/epanov1602/CommandRevSwerve/blob/main/Adding_Camera.md
        #   and this line above should be changed to:
        #     index, x, y, size = self.camera0.getHB(), self.camera0.getX(), self.camera0.getY(), self.camera0.getA()
        #
        # - if you are using PhotonVision, you probably have enough experience to modify the LimelightCamera code from
        #   the example above to use the following fields arriving via NetworkTables:
        #     "targetPitch", "targetYaw", "targetArea", but have to synthesize your own 'index' out of "rawBytes" field
        #   (more information is here : https://docs.photonvision.org/en/latest/docs/additional-resources/nt-api.html)

        if self.minDetectionIndex is None:
            self.minDetectionIndex = index + 1  # we were possibly looking at an old detection
        elif index > self.minDetectionIndex and (x is not None and x != 0):
            return True  # we see a detected object! finished

