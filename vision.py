import cv2
import numpy as np
import config

def ball_spec():
    return config.BALL_HSV

def goal_spec(color):
    return config.GOAL_BY_NAME[color]

def _mask(frame_hsv, spec):
    lo = np.array(spec["lower"], dtype=np.uint8)
    hi = np.array(spec["upper"], dtype=np.uint8)
    return cv2.inRange(frame_hsv, lo, hi)

def _centroid(mask, min_area):
    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    if not contours:
        return None
    c = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(c)
    if area < min_area:
        return None
    M = cv2.moments(c)
    if M["m00"] <= 0:
        return None
    return int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]), area

def ball_centroid(hsv, spec=None):
    spec = spec or config.BALL_HSV
    return _centroid(_mask(hsv, spec), spec["min_area"])

def goal_centroid(hsv, spec=None):
    spec = spec or config.GOAL_BLUE_HSV
    return _centroid(_mask(hsv, spec), spec["min_area"])
