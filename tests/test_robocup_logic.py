"""Tests for robocup.py's pure decision logic (no EV3, no camera).

We import the module, stub out cv2.VideoCapture, and override
Bot._frame_hsv() so we can feed exact HSV frames into tick() and
assert on the state-machine flags.
"""
import sys, os
import numpy as np

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
import robocup
import cv2


class FakeCap:
    """Drop-in for cv2.VideoCapture (we only need isOpened/get/read)."""
    def __init__(self, hsv):
        self._hsv = hsv
    def isOpened(self):
        return True
    def read(self):
        return True, np.zeros((3, self._hsv.shape[0], self._hsv.shape[1]), np.uint8)
    def get(self, prop):
        return self._hsv.shape[1]      # frame width
    def release(self):
        pass


def _solid_hsv(h, s, v, size=(240, 320), box_frac=0.5):
    """Solid-color square in the centre of an otherwise-black HSV frame."""
    arr = np.zeros((size[0], size[1], 3), dtype=np.uint8)
    y0, y1 = int(size[0]*(1-box_frac)/2), int(size[0]*(1+box_frac)/2)
    x0, x1 = int(size[1]*(1-box_frac)/2), int(size[1]*(1+box_frac)/2)
    arr[y0:y1, x0:x1] = (h, s, v)
    return arr


def make_bot(hsv_frame):
    """Build a Bot with a stubbed camera and _frame_hsv() pinned to hsv_frame."""
    robocup.cv2.VideoCapture = lambda *a, **k: FakeCap(hsv_frame)
    b = robocup.Bot(1, no_ev3=True)
    b._frame_hsv = lambda: hsv_frame   # feed HSV directly, no BGR conversion
    return b


def test_ball_far_away_still_lost_waits():
    """Empty frame: no ball, last_ball_was_close stays False."""
    b = make_bot(_solid_hsv(0, 0, 0))
    b.tick()
    assert b.last_ball_was_close is False


def test_ball_close_then_vanishes_flags_close():
    """Big orange blob on frame -> last_ball_was_close becomes True."""
    big_blob = _solid_hsv(18, 180, 200, box_frac=0.9)
    b = make_bot(big_blob)
    b.tick()
    assert b.last_ball_was_close is True, "big orange blob should count as close"


def test_no_ev3_shoot_slept():
    """Robot.shoot() in no_ev3 mode should not raise."""
    r = robocup.Robot(no_ev3=True)
    r.shoot()
