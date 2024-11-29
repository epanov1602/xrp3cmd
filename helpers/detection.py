import typing
import numpy as np
import cv2

GREEN = (127, 255, 0)
WHITE = (255, 255, 255)
RED = (0, 0, 255)
PURPLE = (255, 0, 255)
BLUE = (255, 0, 127)


COCO_CLASSNAMES = [
    'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light', 'fire hydrant',
    'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe',
    'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat',
    'baseball glove', 'skateboard', 'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl',
    'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch', 'potted plant',
    'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink',
    'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush'
]



def detect_biggest_apriltag(detector, frame, only_these_ids=None, tracker=None):
    return detect_or_track(frame, tracker, lambda: _detect_biggest_apriltag(detector, frame, only_these_ids))

def detect_biggest_ball(frame, color_hue_range=(5, 15), smallest_size_px=8, detectors="both", previous_xywh=None, tracker=None):
    def detector():
        return _detect_biggest_ball(frame, color_hue_range, smallest_size_px, previous_xywh=previous_xywh, detectors=detectors)
    return detect_or_track(frame, tracker, detector=detector)

def detect_biggest_face(face_detector, frame, previous_xywh=None, tracker=None):
    return detect_or_track(frame, tracker, lambda: _detect_biggest_face(face_detector, frame, previous_xywh=previous_xywh))

def detect_yolo_object(yolo_model, frame, valid_classes=("person", "car"), lowest_conf=0.4, tracker=None):
    return detect_or_track(frame, tracker, lambda: _detect_yolo_object(yolo_model, frame, valid_classes, lowest_conf))


class TrackerState(object):
    def __init__(self,
                 tracker: cv2.TrackerVit,
                 display_confidence: bool = True,
                 tracker_reinit_interval: int = 40,
                 tracker_max_frames_without_object: int = 40,
                 tracker_lowest_allowed_score: float = 0.6):
        assert tracker is not None
        self.tracker = tracker
        self.tracking = False
        self.time_last_seen = 0
        self.time_last_reinit = 0
        self.frame_count = 0
        self.tracker_comments = None
        self.full_detection_reason = None
        self.display_confidence = display_confidence
        self.tracker_reinit_interval = tracker_reinit_interval
        self.tracker_max_frames_without_object = tracker_max_frames_without_object
        self.tracker_lowest_allowed_score = tracker_lowest_allowed_score

    def init(self, frame, bbox):
        self.tracker.init(frame, bbox)
        self.time_last_seen = self.frame_count
        self.time_last_reinit = self.frame_count
        self.tracking = True

    def update(self, frame):
        self.frame_count += 1

        if not self.tracking:
            return False, (None, None, None, None)

        x, y, w, h, cmt = update_tracker(self.tracker, frame, lowest_allowed_score=self.tracker_lowest_allowed_score)
        reason = "missing"
        if x is not None:
            self.time_last_seen = self.frame_count
            reason = None
        if self.frame_count >= self.time_last_reinit + self.tracker_reinit_interval:
            self.time_last_reinit = self.frame_count
            reason = "tracker_reinit_interval"
        if self.frame_count > self.time_last_seen + self.tracker_max_frames_without_object:
            self.tracking = False
            reason = "tracker_max_frames_without_object"
        self.tracker_comments = cmt
        self.full_detection_reason = reason
        can_skip_full_detection = reason is None
        return can_skip_full_detection, (x, y, w, h)


def create_vit_tracker(model_file="resources/object_tracking_vittrack_2023sep.onnx") -> cv2.TrackerVit:
    """
    Create a tracker that uses VIT
    """
    params = cv2.TrackerVit.Params()
    params.net = model_file
    return cv2.TrackerVit.create(params)


def update_tracker(tracker, frame, lowest_allowed_score=0.6):
    """
    Update a tracker (with a new videoframe), assuming that the tracker has been initialized to track something
    :param tracker: the tracker to update
    :param frame: new video frame
    :param lowest_allowed_score: if the tracker confidence score is lower than this, do not use its output
    :return: bounding box in format (x, y, w, h), or if track is lost then (None, None, None, None)
    """
    # will the tracker give us the updated bounding box?
    located, (tx, ty, tw, th) = tracker.update(frame)
    if located and tracker.getTrackingScore() >= lowest_allowed_score:

        # if tracking almost full screen now, assume that the tracker diverged
        frame_width, frame_height = frame.shape[1], frame.shape[0]

        # do we see any problems with the detection?
        comments = None
        if th > 0.75 * frame_height or tw > 0.75 * frame_width:
            comments = "trk_error: diverged"  # looks too big, our tracker has diverged
        else:
            comments = "trk_conf: {:.2}".format(tracker.getTrackingScore())
            return tx, ty, tw, th, comments

    # otherwise return (None, None, None, None)
    return None, None, None, None, None


def _detect_biggest_apriltag(detector, frame, only_these_ids=None):
    # make a grayscale image, so apriltags can be found on it
    grayscale = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    april_tags = detector.detect(grayscale)

    if len(april_tags) > 0:
        biggest_tag_size = 0
        boxes = []

        # put all boxes into the list
        for tag in april_tags:
            contour = tag.corners.astype(int)
            x, y, w, h = cv2.boundingRect(contour)
            id_location = int(x + w - 10), int(y - 10)
            cv2.polylines(frame, [contour], isClosed=True, color=BLUE, thickness=10)
            cv2.putText(frame, str(tag.tag_id), id_location, cv2.FONT_HERSHEY_SIMPLEX, 1, BLUE, thickness=2)
            if only_these_ids is not None and tag.tag_id not in only_these_ids:
                continue  # skip, because we are only supposed to look at tags with `only_these_ids`
            size = max([w, h])
            biggest_tag_size = max([biggest_tag_size, size])
            boxes.append((x, y, w, h))

        # return the biggest box
        for (x, y, w, h) in boxes:
            size = max([w, h])
            if size == biggest_tag_size:
                return x, y, w, h

    # if nothing was found, return None, None, None, None
    return None, None, None, None


def detect_or_track(frame, tracker: typing.Union[TrackerState, None], detector):
    # 1. try using tracker
    skip_detection = False
    tx, ty, tw, th = None, None, None, None
    if frame is None:
        return tx, ty, tw, th
    if tracker is not None:
        skip_detection, (tx, ty, tw, th) = tracker.update(frame)

    # 2. if this didn't work perfectly, re-detect
    if not skip_detection:
        dx, dy, dw, dh = detector()
        if dx is not None:
            tx, ty, tw, th = dx, dy, dw, dh
            if tracker is not None:
                tracker.init(frame, (dx, dy, dw, dh))
                _, (tx, ty, tw, th) = tracker.update(frame)

    # 3. if we must explain ourselves, do it now
    if tracker is not None and tracker.display_confidence and tracker.tracker_comments:
        text = tracker.tracker_comments
        location = (int(tx) + 10, int(ty + th) + 15)
        cv2.putText(frame, text, location, cv2.FONT_HERSHEY_SIMPLEX, 0.5, PURPLE, 2)

    return tx, ty, tw, th


def _detect_biggest_face(face_detector, frame, draw_boxes=False, previous_xywh=None):
    """
    Use HAAR cascade detector to detect faces (picks the widest one, if many found)
    :param frame: a video frame, color or grayscale
    :param face_detector: cascade face detector
    :return: (x, y, w, h) bounding box or (None, None, None, None)
    """
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = face_detector.detectMultiScale(gray_frame, scaleFactor=1.1, minNeighbors=9)
    if previous_xywh is not None and previous_xywh[0] is None: previous_xywh = None

    biggest_w = 0  # this will be the width of the biggest face
    nearest_distance = None
    biggest_face = None
    nearest_face = None
    for index, (x, y, w, h) in enumerate(faces):
        if w > biggest_w:
            biggest_w = w
            biggest_face = x, y, w, h
        if previous_xywh is not None:
            px, py, pw, ph = previous_xywh
            distance = abs((x + w * 0.5) - (px + pw * 0.5)) + abs((y + 0.5 * h) - (py + ph * 0.5))
            if nearest_distance is None or distance < nearest_distance:
                nearest_distance = distance
                nearest_face = x, y, w, h

    if draw_boxes:
        for (x, y, w, h) in faces:
            # is this the biggest face seen? green box for it, otherwise red for smaller
            if w == biggest_w:
                cv2.rectangle(frame, (x, y), (x + w, y + h), GREEN, thickness=2)
            else:
                cv2.rectangle(frame, (x, y), (x + w, y + h), RED, thickness=2)

    if nearest_face is not None:
        return nearest_face
    if biggest_face is not None:
        return biggest_face
    # if there were no faces, return (None, None, None, None)
    return None, None, None, None


def _detect_biggest_ball(frame, color_hue_range=(5, 15), smallest_size_px=6, previous_xywh=None, detectors="both"):
    """
    :param frame: a video frame, color or grayscale
    :return: (x, y, w, h) bounding box or (None, None, None, None)
    """
    assert (
        detectors in ("circles", "balls", "both")
    ), f"detectors can be 'circles', 'balls' or 'both', not '{detectors}'"

    balls = []
    if detectors in ("circles", "both"):
        balls.extend(detect_circles(frame, color_hue_range=color_hue_range, smallest_size_pixels=smallest_size_px))
    if detectors in ("balls", "both"):
        balls.extend(detect_balls(frame, color_hue_range=color_hue_range, smallest_size_pixels=smallest_size_px))
    if previous_xywh is not None and previous_xywh[0] is None: previous_xywh = None

    biggest_w = 0  # this will be the width of the biggest ball
    nearest_distance = None
    biggest_ball = None
    nearest_ball = None
    for index, ((center_x, center_y), radius) in enumerate(balls):
        x, y, w, h = center_x - radius, center_y - radius, radius * 2, radius * 2
        if w > biggest_w:
            biggest_w = w
            biggest_ball = x, y, w, h
        if previous_xywh is not None:
            px, py, pw, ph = previous_xywh
            distance = abs((x + w * 0.5) - (px + pw * 0.5)) + abs((y + 0.5 * h) - (py + ph * 0.5))
            if nearest_distance is None or distance < nearest_distance:
                nearest_distance = distance
                nearest_ball = x, y, w, h

    if nearest_ball is not None:
        return nearest_ball
    if biggest_ball is not None:
        return biggest_ball

    # if there were no balls, return (None, None, None, None)
    return None, None, None, None


def detect_circles(frame,
                   color_hue_range=(5, 15),
                   color_sat_range=(137, 255),
                   color_val_range=(56, 255),
                   smallest_size_pixels=10,
                   smallest_circularity=0.5,
                   erode=True):
    """
    :param frame:
    :param color_hue_range: lower and upper bounds for color between 0 and 255 (0 = red)
    :param color_sat_range: lower and upper bounds for saturation (0 = gray, 255 = saturated)
    :param color_val_range: lower and upper bounds for value (0 = black, 255 = bright)
    :param smallest_size_pixels: circles smaller than this size will be ignored
    :param smallest_circularity: circles that aren't very circular (e.g. less than 50%) will be thrown out
    :return: list of ((center_x, center_y), radius) for detected circles
    """
    import numpy as np

    # Convert frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define the lower and upper bounds for orange color
    lower_orange = np.array([color_hue_range[0], color_sat_range[0], color_val_range[0]])
    upper_orange = np.array([color_hue_range[1], color_sat_range[1], color_val_range[1]])

    # Create a mask for orange color
    mask = cv2.inRange(hsv, lower_orange, upper_orange)

    # Apply morphological operations to reduce noise
    if erode:
        mask = cv2.erode(mask, None, iterations=1)
    mask = cv2.dilate(mask, None, iterations=2)
    cv2.imshow("color hue mask", mask)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Detect balls based on contour properties
    balls = []
    for contour in contours:
        # Calculate area and circularity
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        circularity = 4 * np.pi * area / (perimeter ** 2) if perimeter > 0 else 0

        # Filter contours based on area and circularity
        if circularity > smallest_circularity and area > smallest_size_pixels * smallest_size_pixels:
            # Calculate center and radius of the ball
            (x, y), radius = cv2.minEnclosingCircle(contour)
            center = (int(x), int(y))
            radius = int(radius)

            # Add ball information to the list
            balls.append((center, radius))

    return balls


def detect_balls(image,
                 color_hue_range=(5, 15),
                 smallest_size_pixels=10,
                 smallest_circularity=0.4):

    # Create a copy for results
    result = image.copy()

    # Convert to HSV color space
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define color ranges for tennis balls (yellow-green)
    color_ranges = [
        # Bright, well-lit tennis balls
        (np.array([color_hue_range[0], 43, 126]), np.array([color_hue_range[1], 255, 255])),
        # Slightly darker tennis balls
        (np.array([color_hue_range[0], 137, 56]), np.array([color_hue_range[1], 255, 255]))
    ]

    # Combine masks from different color ranges
    final_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lower, upper in color_ranges:
        mask = cv2.inRange(hsv, lower, upper)
        final_mask = cv2.bitwise_or(final_mask, mask)

    # Noise reduction and smoothing
    kernel = np.ones((5, 5), np.uint8)
    final_mask = cv2.morphologyEx(final_mask, cv2.MORPH_OPEN, kernel, iterations=1)
    final_mask = cv2.morphologyEx(final_mask, cv2.MORPH_CLOSE, kernel, iterations=1)
    final_mask = cv2.GaussianBlur(final_mask, (5, 5), 0)

    # Find contours of objects in yellow color
    contours, _ = cv2.findContours(final_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Filter and draw contours
    detected_balls = []
    for contour in contours:
        area = cv2.contourArea(contour)

        # Filter by area (adjust these values based on your image size)
        # You guys can play with this area to detect larger or smaller objects
        if area > smallest_size_pixels * smallest_size_pixels:
            # Calculate circularity
            perimeter = cv2.arcLength(contour, True)
            circularity = 4 * np.pi * area / (perimeter * perimeter) if perimeter > 0 else 0

            # Filter by circularity
            if circularity > smallest_circularity:  # Tennis balls should be fairly circular
                # Get bounding circle
                (x, y), radius = cv2.minEnclosingCircle(contour)
                center = (int(x), int(y))
                radius = int(radius)

                # Calculate aspect ratio of bounding rect for additional verification
                x, y, w, h = cv2.boundingRect(contour)
                aspect_ratio = float(w) / h

                # Only proceed if aspect ratio is close to 1 (circle)
                if 0.75 < aspect_ratio < 1.25:
                    # Draw the detection
                    cv2.circle(result, center, radius, (0, 255, 0), 2)
                    cv2.circle(result, center, 2, (0, 0, 255), -1)

                    # Add measurements
                    #cv2.putText(result, f'Ball ({int(area)}px)',
                    #            (center[0] - 40, center[1] - 20),
                    #            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    detected_balls.append((center, radius))

    # Draw total count
    #cv2.putText(result, f'Detected: {len(detected_balls)} balls',
    #            (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    return detected_balls


def _detect_yolo_object(yolo_model, frame, valid_classes=("person", "car"), lowest_conf=0.3):
    """
    Detect an object using a YOLO model (if multiple objects detected, picks the widest)
    :return: either (x, y, w, h) for the bounding box, or (None, None, None, None)
    """
    boxes = []
    biggest_width = 0
    results = yolo_model.predict(frame)

    for result in results:
        for bbox in result.boxes:
            class_name = result.names[int(bbox.cls[0])]
            conf = float(bbox.conf)
            x, y, x2, y2 = bbox.xyxy[0]
            if class_name in valid_classes and conf > lowest_conf:
                width, height = int(x2 - x), int(y2 - y)
                boxes.append([int(x), int(y), width, height])
                if width > biggest_width:
                    biggest_width = width
            text = "{}@{:.2}".format(class_name, conf)
            location = (int(x2) + 10, int(y) + 15)
            cv2.rectangle(frame, (int(x), int(y)), (int(x2), int(y2)), RED, 2)
            cv2.putText(frame, text, location, cv2.FONT_HERSHEY_SIMPLEX, 0.5, WHITE, 1)

    for (x, y, w, h) in boxes:
        if w == biggest_width:
            cv2.rectangle(frame, (x, y), (x + w, y + h), GREEN, 4)
            return x, y, w, h

    # otherwise, nothing found
    return None, None, None, None


def to_normalized_x_y_size(frame, x, y, w, h, draw_box=False):
    """
    Convert the x, y, width and height of a detected object into [-0.5; +0.5] space,
    so robot can use them in navigation
    :param frame: frame from the camera (colored or grayscale), numpy ndarray
    :param x: X of the upper left corner of the detected object bounding box (in pixels)
    :param y: Y of the upper left corner of the detected object bounding box (in pixels)
    :param w: width of the detected object box (in pixels)
    :param h: height of the detected object box (in pixels)
    :param draw_box: annotate the frame with the bounding box
    :return: (X, Y, Size), with X and Y remapped to [-50; +50] space (X, Y = center of the box), Size between 0 and 100
    """
    if x is None:
        return None, None, None
    frame_width, frame_height = frame.shape[1], frame.shape[0]
    norm_x = 100 * ((x + w // 2) / frame_width - 0.5)  # can be between -0.5 and +0.5
    norm_y = 100 * ((y + h // 2) / frame_height - 0.5)  # can be between -0.5 and +0.5
    norm_size = 100 * max((w / frame_width, h / frame_height))
    norm_y = -norm_y  # flip the sign, so that when object is above Y is positive

    if draw_box:
        text = "nx:{:.0f}, ny:{:.0f}, size: {:.0f}".format(norm_x, norm_y, norm_size)
        cv2.putText(frame, text, (x, y - 10), cv2.FONT_HERSHEY_PLAIN, 1.0, WHITE, 2)
        cv2.rectangle(frame, (int(x), int(y)), (int(x + w), int(y + h)), GREEN, thickness=2)

    return norm_x, norm_y, norm_size
