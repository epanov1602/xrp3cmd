import cv2
import commands2

from helpers.detection import detect_biggest_apriltag, detect_biggest_face
from helpers import detection

from wpilib import SmartDashboard, Timer
from threading import Lock, Thread
import pupil_apriltags as apriltags


class CVCamera(commands2.Subsystem):
    instance_count = 0

    def __init__(self, horizontal_fov_degrees=70, vertical_fov_degrees=50, max_fps=10, detector=None):
        """

        :param horizontal_fov_degrees: horizontal field of view for the camera (for example, 70 degrees)
        :param vertical_fov_degrees: vertical field of view for the camera (for example, 50 degrees)
        :param max_fps: maximum frames per second on which detection will be performed (for example, 10)
        :param detector: callable that returns an (x,y,w,h) bounding box or None (args: frame, tracker, previous_box)
        """
        super().__init__()

        self.tracker = detection.TrackerState(detection.create_vit_tracker(), display_confidence=True)
        self.detector = detector

        self.last_detected_time = 0
        self.last_detected_index = 0
        self.last_detected_center_degrees = (None, None)
        self.last_detected_size_degrees = (None, None)
        self.last_detected_bbox = None
        self.last_detected_frame = None

        self.last_valid_detected_center_degrees = (None, None)

        self.frame = None
        self.frame_index = 0

        self.max_fps = max_fps
        self.horizontal_fov_degrees = horizontal_fov_degrees
        self.vertical_fov_degrees = vertical_fov_degrees

        self.lock = Lock()

        CVCamera.instance_count += 1
        self.window_name = f"robot camera {CVCamera.instance_count}"
        cv2.namedWindow(self.window_name)

        if self.detector is None:
            def mouse_callback(event, x, y, flags, param):
                if event == cv2.EVENT_LBUTTONDOWN or event == cv2.EVENT_RBUTTONDOWN:
                    roi = self._get_roi()
                    if roi is not None:
                        self.tracker.init(self.last_detected_frame, roi)

            self.mouse_callback = mouse_callback
            cv2.setMouseCallback(self.window_name, self.mouse_callback)

        self.camera = None
        self._background_worker = None

    def start(self, *args, **kwargs):
        assert self.camera is None, "start() was already called once"

        print(f"Trying to start a web camera with args={args}, kwargs={kwargs}")
        self.camera = cv2.VideoCapture(*args, **kwargs)
        if not self.camera.isOpened():
            print(f"ERROR: failed to open a video capture with args={args}, kwargs={kwargs}")
            self.camera = None
            return

        self._background_worker = Thread(target=self._background_work, daemon=True)
        self._background_worker.start()

    def get_detected_object(self):
        return (
            self.last_detected_time,
            self.last_detected_index,
            self.last_detected_center_degrees,
            self.last_detected_size_degrees,
        )

    def periodic(self):
        t = Timer.getFPGATimestamp()
        if t - self.last_detected_time < 0.5 / self.max_fps:
            return  # too early to detect again
        with self.lock:
            frame, index = self.frame, self.frame_index
        if index == self.last_detected_index or frame is None:
            return  # nothing new to detect

        self.last_detected_bbox = self.detect(frame)
        self.last_detected_frame, self.last_detected_index, self.last_detected_time = frame, index, t
        self.set_last_detected_coordinates_degrees()

        cv2.imshow(self.window_name, self.last_detected_frame)
        cv2.waitKey(1)

    def detect(self, frame):
        if self.detector is not None:
            return self.detector(frame, self.tracker, self.last_detected_bbox)
        if self.tracker.tracking:
            _, bbox = self.tracker.update(frame)
            return bbox

    def set_last_detected_coordinates_degrees(self, draw=True):
        self.last_detected_center_degrees = (None, None)
        self.last_detected_size_degrees = (None, None)
        if self.last_detected_bbox is not None and self.last_detected_bbox[0] is not None and self.last_detected_frame is not None:
            x, y, w, h = self.last_detected_bbox
            center_x, center_y = x + 0.5 * w, y + 0.5 * h
            frame_height, frame_width = self.last_detected_frame.shape[0], self.last_detected_frame.shape[1]
            self.last_detected_center_degrees = (
                self.horizontal_fov_degrees * (center_x - 0.5 * frame_width) / frame_width,
                self.vertical_fov_degrees * (0.5 * frame_height - center_y) / frame_height,
            )
            self.last_detected_size_degrees = (
                self.horizontal_fov_degrees * (w / frame_width),
                self.vertical_fov_degrees * (h / frame_height),
            )
            self.last_valid_detected_center_degrees = self.last_detected_center_degrees
            if draw:
                index = self.last_detected_index
                size = max(self.last_detected_size_degrees)
                text = "x:{:.0f}, y:{:.0f}, size: {:.0f}, ix: {}".format(
                    self.last_detected_center_degrees[0], self.last_detected_center_degrees[1], size, index
                )
                cv2.putText(
                    self.last_detected_frame, text, (x, y - 10),
                    cv2.FONT_HERSHEY_PLAIN,1.0, (255, 255, 255), 2
                )
                cv2.rectangle(
                    self.last_detected_frame, (int(x), int(y)), (int(x + w), int(y + h)), (0, 255, 0), thickness=2
                )

    def _get_roi(self):
        if self.last_detected_frame is None:
            return None
        inverted = cv2.cvtColor(self.last_detected_frame, cv2.COLOR_BGR2RGB)
        cv2.putText(
                inverted,
                "select region, then press Enter",
                (5, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 255, 0),
                thickness=2,
            )
        roi = cv2.selectROI(self.window_name, inverted, showCrosshair=False)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)
        return roi


    def _background_work(self):
        while self.camera is not None:
            self._next_frame()

    def _next_frame(self):
        # this might need to be done on another thread, but...
        # (catch up by discarding the old frames sitting in the queue)
        t = Timer.getFPGATimestamp()
        while True:
            self.camera.grab()
            after = Timer.getFPGATimestamp()
            if after - t > 0.010:
                break  # if this took us more than 10ms to get next frame, it was fresh
            t = after
        success, frame = self.camera.retrieve()  # and decode just that last frame
        if success:
            with self.lock:
                self.frame = frame
                self.frame_index += 1
