# RoboCup 2026 — single-file autonomous soccer bot
# ======================================================
#  Run on the laptop, drives the EV3 over Bluetooth.
#
#  Hardware map (confirmed with team):
#    Motor A + B  : differential drive (left / right wheel)
#    Motor C      : dribbler  (forward = trap,  reverse = shoot)
#    Port 3 + 4   : two white colour sensors (left + right) -> boundary-line guard
#
#  Usage:
#    python robocup.py --half 1     # shoot BLUE goal,  defend YELLOW
#    python robocup.py --half 2     # shoot YELLOW goal, defend BLUE
#    python robocup.py --no-ev3     # camera-only, print commands
#
#  Either white sensor runs in a background thread: the instant one sees
#  the white boundary line, all motors stop and the bot re-searches for
#  the ball.  This is what keeps the bot inside the field.

import argparse
import math
import threading
import time
from dataclasses import dataclass, field

import cv2
import numpy as np

try:
    import ev3_dc as ev3
    EV3_AVAILABLE = True
except ImportError:
    ev3 = None
    EV3_AVAILABLE = False


# ============================================================
# CONFIG — tune these on the field.  Every magic number lives here.
# ============================================================
EV3_ADDRESS = "00:16:53:61:AB:61"   # your EV3's Bluetooth MAC

# --- Ball: 42 mm ORANGE golf ball (the 2026 spec ball is orange, NOT white)
BALL_HSV = dict(lower=(10, 120, 120), upper=(25, 255, 255), min_area=400)

# --- Goals: interior color (one yellow, one blue)
GOAL_BLUE_HSV   = dict(lower=(100, 80, 80),  upper=(130, 255, 255), min_area=300)
GOAL_YELLOW_HSV = dict(lower=(25, 80, 180),  upper=(35, 255, 255),  min_area=300)
GOAL_BY_NAME = {"blue": GOAL_BLUE_HSV, "yellow": GOAL_YELLOW_HSV}

# --- The white boundary line (field edges).  The WHITE SENSORS (ports 3 + 4)
#     watch this; the camera never has to detect it, which is why a white
#     border never confuses the orange-ball detector.
WHITE_SENSOR_PORTS = [3, 4]

# --- Ball-lost logic.  If the ball VANISHES while it was CLOSE (large
#     area), it has dropped into the dribbler -> shoot immediately.
#     If it vanished while SMALL (far away), it just left the field of
#     view -> wait LOST_TIMEOUT_S before assuming it's in the dribbler.
BALL_AREA_CLOSE = 4000     # px^2.  Above this = "close" when it disappears.
LOST_TIMEOUT_S    = 1.5    # fallback wait when ball vanished while far.

# --- Motion (differential drive on A + B)
CHASE_SPEED   = 50         # forward speed while chasing the ball
OMEGA_KP      = 0.2        # turn gain:  omega = int(err_px * OMEGA_KP)
ALIGN_TOL_PX  = 40         # |ball_x - goal_x| under this -> aligned, fire
BALL_MAX_AREA = 3000       # if ball blob this big, it's in the dribbler zone

# --- Dribbler (port C)
DIBBLE_SPEED  = 60         # forward: trap / roll the ball
SHOOT_SPEED   = 100        # reverse: full-power shot (matches your blocks)
SHOOT_MS      = 250        # how long the reverse burst lasts


# ============================================================
# VISION — pure functions, no hardware.  Testable on synthetic frames.
# ============================================================
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
    """Largest orange blob -> (cx, cy, area) or None."""
    spec = spec or BALL_HSV
    return _centroid(_mask(hsv, spec), spec["min_area"])


def goal_centroid(hsv, spec=None):
    """Largest goal-color blob -> (cx, cy, area) or None."""
    spec = spec or GOAL_BLUE_HSV
    return _centroid(_mask(hsv, spec), spec["min_area"])


# ============================================================
# ROBOT — differential drive (A + B), dribbler (C), white sensors (3 + 4)
# ============================================================
class Robot:
    def __init__(self, no_ev3=False):
        self.no_ev3 = no_ev3
        if no_ev3:
            self.wA = self.wB = self.dribbler = None
            self.white_sensors = []
            return
        r = ev3.EV3(protocol=ev3.BLUETOOTH, host=EV3_ADDRESS)
        self.wA = ev3.Motor(ev3.PORT_A, ev3_obj=r)
        self.wB = ev3.Motor(ev3.PORT_B, ev3_obj=r)
        self.dribbler = ev3.Motor(ev3.PORT_C, ev3_obj=r)
        # Two white colour sensors: ports 3 (left) and 4 (right).
        self.white_sensors = [
            ev3.Color(ev3.PORT_3, ev3_obj=r),
            ev3.Color(ev3.PORT_4, ev3_obj=r),
        ]

    # ---- differential drive: A = left, B = right -----------------
    def move(self, forward=0, turn=0):
        """forward & turn in -100..100.  A = forward+turn, B = forward-turn."""
        A = int(max(-100, min(100, forward + turn)))
        B = int(max(-100, min(100, forward - turn)))
        if self.no_ev3:
            print(f"move forward={forward:>3} turn={turn:>3}  -> A={A:>3} B={B:>3}")
            return
        self.wA.start(speed=A)
        self.wB.start(speed=B)

    def stop(self):
        if self.no_ev3:
            return
        self.wA.stop(brake=True)
        self.wB.stop(brake=True)

    def dribbler_forward(self):
        if self.no_ev3:
            return
        self.dribbler.start(speed=DIBBLE_SPEED)

    def dribbler_off(self):
        if self.no_ev3:
            return
        # Coast to a stop so the ball doesn't eject (no hard brake).
        self.dribbler.stop(brake=False)

    def shoot(self):
        if self.no_ev3:
            print(">>> SHOOT <<<")
            time.sleep(SHOOT_MS / 1000.0)
            return
        self.dribbler.start(speed=-SHOOT_SPEED)   # -100 full-power reverse
        time.sleep(SHOOT_MS / 1000.0)
        self.dribbler.stop(brake=False)

    def idle(self):
        self.stop()
        if not self.no_ev3:
            self.dribbler.stop(brake=False)

    # ---- white boundary sensors (ports 3 + 4) -------------------
    def sees_white(self):
        """True when EITHER colour sensor reads the white boundary line."""
        if self.no_ev3:
            return False
        for sensor in self.white_sensors:
            try:
                if sensor.color == ev3.WHITECOLOR:
                    return True
            except Exception:
                pass
        return False


# ============================================================
# BOUNDARY GUARD — forever-loop in a background thread.
#  Either white sensor (port 3 or 4) sees the line -> stop everything.
#  The main loop then re-searches for the ball and drives back.
# ============================================================
@dataclass
class BoundaryGuard:
    robot: Robot
    stop_flag: bool = field(default=False)

    def run(self):
        while not self.stop_flag:
            if self.robot.sees_white():
                # Debounce a couple of frames so a single glitch
                # doesn't yank the motors.
                confirmed = all(
                    self.robot.sees_white() for _ in range(3)
                )
                if confirmed:
                    self.robot.stop()
                    self.robot.dribbler_off()
                    print("[guard] white line -> stopped")
            time.sleep(0.05)   # 20 Hz polling is plenty for a boundary stop

    def stop(self):
        self.stop_flag = True


# ============================================================
# STATE MACHINE
# ============================================================
class Bot:
    def __init__(self, half, no_ev3=False):
        self.robot = Robot(no_ev3=no_ev3)
        self.goal_color = "blue" if half == 1 else "yellow"
        self.last_ball_seen = 0.0
        self.last_ball_was_close = False

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            for _ in range(3):                 # retry a few times
                time.sleep(2)
                self.cap = cv2.VideoCapture(0)
                if self.cap.isOpened():
                    break
            if not self.cap.isOpened():
                raise SystemExit("Could not open webcam")
        self._cx = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH) // 2)

        self.guard = BoundaryGuard(self.robot)

    # ---- per-frame vision -------------------------------------
    def _frame_hsv(self):
        ok, frame = self.cap.read()
        if not ok:
            raise SystemExit("Frame read failed")
        return cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    def tick(self):
        hsv = self._frame_hsv()
        now = time.time()
        ball = ball_centroid(hsv)
        goal = goal_centroid(hsv, GOAL_BY_NAME[self.goal_color])

        if ball:
            self.last_ball_seen = now
            self.last_ball_was_close = ball[2] >= BALL_AREA_CLOSE

        ball_lost = (ball is None) and (
            now - self.last_ball_seen > LOST_TIMEOUT_S
        )
        goal_on = goal is not None

        # Fast path: ball vanished while CLOSE -> it's in the dribbler.
        # No 1.5 s wait; go straight to the goal and shoot.
        if ball is None:
            if self.last_ball_was_close and goal_on:
                self._shoot_mode(goal)
            elif ball_lost and goal_on:
                self._shoot_mode(goal)
            else:
                self.robot.idle()
        else:
            if ball_lost and self.last_ball_was_close:
                self._fire()
            elif goal_on and self._aligned(ball, goal):
                self._fire()
            else:
                self._chase(ball)

    def _chase(self, ball):
        cx, _cy, _area = ball
        err = cx - self._cx
        omega = int(err * OMEGA_KP)
        # Slow forward speed as we turn, so we don't overshoot.
        fwd = int(CHASE_SPEED * max(0.3, 1.0 - abs(err) / self._cx))
        self.robot.move(forward=fwd, turn=omega)
        self.robot.dribbler_forward()

    def _aligned(self, ball, goal):
        bx, _by, ba = ball
        gx, _gy, _ga = goal
        return abs(bx - gx) < ALIGN_TOL_PX and ba < BALL_MAX_AREA

    def _fire(self):
        self.robot.stop()
        self.robot.shoot()

    def _shoot_mode(self, goal):
        gx, _gy, _ga = goal
        err = gx - self._cx
        omega = int(err * OMEGA_KP)
        self.robot.move(forward=CHASE_SPEED, turn=omega)
        self.robot.dribbler_forward()   # keep ball pinned until fire

    def run(self):
        # White-sensor forever-loop in its own thread.
        self.guard_thread = threading.Thread(target=self.guard.run, daemon=True)
        self.guard_thread.start()

        while True:
            self.tick()
            time.sleep(0.033)               # ~30 fps cap (don't flood BLE)
            k = cv2.waitKey(1) & 0xFF
            if k in (ord("q"), 27):
                break

        self.guard.stop()
        self.guard_thread.join(timeout=1.0)


def main():
    ap = argparse.ArgumentParser(description="RoboCup 2026 EV3 soccer bot")
    ap.add_argument("--half", type=int, choices=[1, 2], required=True,
                    help="1 = shoot blue, defend yellow;  2 = shoot yellow, defend blue")
    ap.add_argument("--no-ev3", action="store_true",
                    help="camera-only: print motor commands, don't touch the EV3")
    a = ap.parse_args()

    b = Bot(a.half, no_ev3=a.no_ev3)
    try:
        b.run()
    finally:
        b.robot.idle()
        b.cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
