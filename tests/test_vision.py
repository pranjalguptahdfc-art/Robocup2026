import numpy as np, cv2, sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
import vision, config

def _solid_hsv(h, s, v, size=(200, 300), box_frac=0.5):
    """Solid-color HSV square in the center of an otherwise black frame."""
    arr = np.zeros((size[0], size[1], 3), dtype=np.uint8)
    y0 = size[0] // 4; y1 = size[0] * 3 // 4
    x0 = size[1] // 4; x1 = size[1] * 3 // 4
    arr[y0:y1, x0:x1] = (h, s, v)
    return arr

def test_orange_ball_detected():
    hsv = _solid_hsv(18, 180, 200)        # mid-orange
    r = vision.ball_centroid(hsv, vision.ball_spec())
    assert r is not None
    cx, cy, area = r
    assert 50 < cx < 250 and 25 < cy < 175 and area > 400

def test_orange_ball_not_white_or_red():
    hsv_w = _solid_hsv(0, 30, 230)        # white-ish
    hsv_r = _solid_hsv(0, 250, 200)       # red
    assert vision.ball_centroid(hsv_w, vision.ball_spec()) is None
    assert vision.ball_centroid(hsv_r, vision.ball_spec()) is None

def test_blue_goal_detected():
    hsv = _solid_hsv(115, 200, 200)
    r = vision.goal_centroid(hsv, vision.goal_spec("blue"))
    assert r is not None and r[2] > 300

def test_yellow_goal_detected():
    hsv = _solid_hsv(30, 200, 220)
    r = vision.goal_centroid(hsv, vision.goal_spec("yellow"))
    assert r is not None and r[2] > 300

def test_no_detection_on_black_frame():
    hsv = np.zeros((200, 300, 3), dtype=np.uint8)
    assert vision.ball_centroid(hsv, vision.ball_spec()) is None
    assert vision.goal_centroid(hsv, vision.goal_spec("blue")) is None
