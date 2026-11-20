"""tool_tag.py: the tool tag in one color image (D-22, plan A-9). No ROS.

The tag is the AprilTag (36h11) of the hand-eye calibration, moved to the flange. Its centre in the
camera frame comes from the four corners and the side of its black square (solvePnP, IPPE_SQUARE),
as in goal-to-torque_MPSoC/calib/calib_cpo.py, whose tag poses made the calibration: the same
corner order, the same model of the marker. pick_target_3d_node looks for it in the color image of
each capture the detector found the object in; the control task adds the object minus the tag to
the tag position that the joint angles of that capture give.

  ToolTagFinder.find(gray, t_s)   the tag in a grayscale image: first in a window around where it
                                  was last seen (it moves little between two captures), then in the
                                  whole image, with the plain detector; AprilTag corner refinement
                                  only in a window (on a whole image it takes about 20 times as
                                  long). After a whole image without the tag the next one waits
                                  whole_image_period_s: a result then has no tag ('not_searched')
  gray_of_image(...)              a grayscale view of a sensor_msgs/Image's fields (rgb8, bgr8,
                                  mono8)

Tests: python3 -m pytest src/target_3d_pkg/test/test_tool_tag.py (OpenCV with aruco, numpy)."""

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class TagResult:
    """The tag in one image. found false: reason says why, and the rest is not set."""

    found: bool
    reason: str = ''
    p_cam: np.ndarray = None  # [m] the centre of the tag in the camera frame
    reproj_px: float = 0.0  # mean reprojection error of the four corners
    ambiguity: float = 0.0  # error of the other IPPE solution / of this one (> 2: clear)
    window: bool = False  # found in the window around the last place


def aruco_detector(refine='apriltag'):
    """detect(gray) -> (corners, ids) for 36h11, with OpenCV < 4.7 (detectMarkers) and >= 4.7
    (ArucoDetector)."""
    adict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    new_api = hasattr(cv2.aruco, 'ArucoDetector')
    params = cv2.aruco.DetectorParameters() if new_api else cv2.aruco.DetectorParameters_create()
    methods = {
        'subpix': 'CORNER_REFINE_SUBPIX',
        'apriltag': 'CORNER_REFINE_APRILTAG',
        'none': None,
    }
    if refine not in methods:
        raise ValueError(f'tag corner refinement {refine!r} is none of {sorted(methods)}')
    if methods[refine] is not None:
        params.cornerRefinementMethod = getattr(cv2.aruco, methods[refine])
    if new_api:
        det = cv2.aruco.ArucoDetector(adict, params)
        return lambda gray: det.detectMarkers(gray)[:2]
    return lambda gray: cv2.aruco.detectMarkers(gray, adict, parameters=params)[:2]


def marker_points(size_m):
    """The corners of the black square in the marker frame, in the order of detectMarkers: TL, TR,
    BR, BL
    (x right, y up, z out of the face; calib_cpo.py)."""
    s = 0.5 * size_m
    return np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], dtype=np.float64)


class ToolTagFinder:
    """Finds one tag (tag_id, black square size_m) in grayscale images of one camera."""

    def __init__(self, tag_id, size_m, refine='apriltag', window_margin_px=48, window_keep_s=0.5,
                 whole_image_period_s=0.2):
        if not size_m > 0.0:
            raise ValueError(f'tag size {size_m!r} m: the side of the black square, > 0')
        if refine not in ('apriltag', 'subpix', 'none'):
            raise ValueError(f"tag corner refinement {refine!r} is none of apriltag, subpix, none")
        self.tag_id = int(tag_id)
        self.size_m = float(size_m)
        self.obj = marker_points(self.size_m)
        # Two detectors. The plain one (SUBPIX corners) searches the whole image: with AprilTag
        # refinement a whole 848x480 image with sensor noise took 120 ms on a PC against 7 ms, tag
        # or no tag, and in 270 rendered images it found no tag that the plain one missed (I-110).
        # The corners of the calibration (calib_cpo.py, AprilTag refinement) are then taken in a
        # window around the tag, where that costs little. It can drop a tag there: the SUBPIX
        # corners stay. In rendered images (test_tool_tag.py) the AprilTag corners put the centre
        # within about 1 mm across and 7 mm in depth at 0.55-1.1 m, the SUBPIX ones 28 mm in depth.
        self.coarse = aruco_detector('none' if refine == 'none' else 'subpix')
        self.fine = aruco_detector('apriltag') if refine == 'apriltag' else None
        self.in_window = ([self.fine] if self.fine is not None else []) + [self.coarse]
        self.margin = int(window_margin_px)
        self.keep_s = float(window_keep_s)
        self.whole_period_s = float(whole_image_period_s)
        self.K = None
        self.D = None
        self._last_box = None  # (u0, v0, u1, v1) of the corners the last time
        self._last_t = None
        self._miss_t = None  # when the whole image last had no tag of ours
        self.seen_ids = ()  # the ids of the last image with tags but not ours (to find tag_id)

    def set_camera(self, k, d):
        """k: the 9 values of camera_info.k, d: camera_info.d (any length; none: no distortion)."""
        self.K = np.asarray(k, dtype=np.float64).reshape(3, 3)
        self.D = np.asarray(d if d is not None and len(d) else np.zeros(5), dtype=np.float64)

    def ready(self):
        return self.K is not None

    def _corners_in(self, gray, box, detects):
        """the corners (4, 2) of the one tag with tag_id inside box of gray, by the first of
        detects that finds it, or a reason"""
        u0, v0, u1, v1 = box
        reason = 'not_found'
        for detect in detects:
            corners, ids = detect(gray[v0:v1, u0:u1])
            if ids is None or len(ids) == 0:
                continue
            ids = np.asarray(ids).flatten()
            mine = [c for c, i in zip(corners, ids) if int(i) == self.tag_id]
            if len(mine) > 1:
                return None, 'several'  # two tags with our id: which one is the tool's is unknown
            if not mine:
                reason = 'other_id'
                self.seen_ids = tuple(sorted({int(i) for i in ids}))
                continue
            return (
                np.asarray(mine[0], np.float64).reshape(4, 2) + np.array([u0, v0], np.float64),
                '',
            )
        return None, reason

    @staticmethod
    def _corner_box(pts):
        return (
            int(pts[:, 0].min()),
            int(pts[:, 1].min()),
            int(np.ceil(pts[:, 0].max())),
            int(np.ceil(pts[:, 1].max())),
        )

    def _window(self, corner_box, w, h):
        """corner_box grown by the margin, inside the image; None if no tag fits in it"""
        u0, v0, u1, v1 = corner_box
        box = (
            max(0, u0 - self.margin),
            max(0, v0 - self.margin),
            min(w, u1 + self.margin),
            min(h, v1 + self.margin),
        )
        return box if box[2] - box[0] >= 16 and box[3] - box[1] >= 16 else None

    def find(self, gray, t_s=None):
        """TagResult for one grayscale image taken at t_s [s] (for the window and the wait after
        a whole image without the tag; None: neither)."""
        if not self.ready():
            return TagResult(False, 'no_camera_info')
        h, w = gray.shape[:2]
        pts, reason, window = None, 'not_found', False
        if (
            self._last_box is not None
            and t_s is not None
            and self._last_t is not None
            and 0.0 <= t_s - self._last_t <= self.keep_s
        ):
            box = self._window(self._last_box, w, h)
            if box is not None:
                pts, reason = self._corners_in(gray, box, self.in_window)
                window = pts is not None
        if pts is None:
            # the whole image, the slow part: not again right after one that had no tag
            if (
                t_s is not None
                and self._miss_t is not None
                and 0.0 <= t_s - self._miss_t < self.whole_period_s
            ):
                return TagResult(False, 'not_searched')
            pts, reason = self._corners_in(gray, (0, 0, w, h), [self.coarse])
            if pts is None:
                self._miss_t = t_s
                return TagResult(False, reason)
            box = self._window(self._corner_box(pts), w, h)
            if self.fine is not None and box is not None:
                fine_pts, _ = self._corners_in(gray, box, [self.fine])
                if fine_pts is not None:
                    pts = fine_pts
        self._miss_t = None
        ok, rvecs, tvecs, errs = cv2.solvePnPGeneric(
            self.obj, pts, self.K, self.D, flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
        if not ok or len(tvecs) == 0:
            return TagResult(False, 'pnp_failed')
        errs = np.asarray(errs, dtype=np.float64).flatten()
        best = int(np.argmin(errs)) if errs.size else 0
        p = np.asarray(tvecs[best], dtype=np.float64).flatten()
        if not np.all(np.isfinite(p)) or p[2] <= 0.0:
            return TagResult(False, 'pnp_failed')
        ratio = float(errs[1 - best] / max(errs[best], 1e-9)) if errs.size > 1 else float('inf')
        self._last_box = self._corner_box(pts)
        self._last_t = t_s
        return TagResult(True, '', p, float(errs[best]) if errs.size else 0.0, ratio, window)


def gray_of_image(data, height, width, step, encoding):
    """A grayscale uint8 image (height, width) from the fields of a sensor_msgs/Image, or None
    for an encoding this does not read. data: bytes or a buffer; step: bytes per row (rows may
    be padded)."""
    rows = np.frombuffer(data, dtype=np.uint8)
    if rows.size < height * step:
        return None
    rows = rows[: height * step].reshape(height, step)
    if encoding == 'mono8':
        return rows[:, :width]
    if encoding in ('rgb8', 'bgr8'):
        img = rows[:, : width * 3].reshape(height, width, 3)
        return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY if encoding == 'rgb8' else cv2.COLOR_BGR2GRAY)
    return None
