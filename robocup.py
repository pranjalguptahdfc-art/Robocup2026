import argparse, time
import cv2
import config, vision, robot as robot_mod

class Bot:
    def __init__(self, half: int, no_ev3: bool = False):
        self.robot = robot_mod.Robot(no_ev3=no_ev3)
        self.goal_color = "blue" if half == 1 else "yellow"
        self.last_ball_seen = 0.0
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            raise SystemExit("Could not open webcam")
        self._cx = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH) // 2)

    # --- per-frame vision ------------------------------------------------
    def _frame_hsv(self):
        ok, frame = self.cap.read()
        if not ok:
            raise SystemExit("Frame read failed")
        return cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # --- state machine ----------------------------------------------------
    def tick(self):
        hsv = self._frame_hsv()
        now = time.time()
        ball = vision.ball_centroid(hsv, vision.ball_spec())
        goal = vision.goal_centroid(hsv, vision.goal_spec(self.goal_color))

        if ball:
            self.last_ball_seen = now

        ball_lost = (ball is None) and (now - self.last_ball_seen) > config.LOST_TIMEOUT_S
        goal_on  = goal is not None

        if ball is None:
            if ball_lost and goal_on:
                self._shoot_mode(goal)
            else:
                # still searching — idle, dribbler off
                self.robot.idle()
        else:
            if ball_lost:
                self.robot.idle()
            elif goal_on and self._aligned(ball, goal):
                self._fire()
            else:
                self._chase(ball)

    def _chase(self, ball):
        cx, cy, area = ball
        err = cx - self._cx
        omega = int(err * config.OMEGA_KP)
        self.robot.omni(config.CHASE_SPEED, 0, omega)
        self.robot.dribbler_forward()

    def _aligned(self, ball, goal):
        bx, by, ba = ball
        gx, gy, ga = goal
        return abs(bx - gx) < config.ALIGN_TOL_PX and ba < 3000

    def _fire(self):
        self.robot.stop()
        self.robot.shoot()

    def _shoot_mode(self, goal):
        gx, gy, ga = goal
        err = gx - self._cx
        omega = int(err * config.OMEGA_KP)
        self.robot.omni(config.CHASE_SPEED, 0, omega)
        self.robot.dribbler_forward()      # keep ball pinned; fire on next aligned frame

    def run(self):
        while True:
            self.tick()
            k = cv2.waitKey(1) & 0xFF
            if k in (ord("q"), 27):
                break

def main():
    ap = argparse.ArgumentParser(description="RoboCup 2026 EV3 soccer bot")
    ap.add_argument("--half", type=int, choices=[1, 2], required=True,
                    help="1 = shoot blue, defend yellow;  2 = shoot yellow, defend blue")
    ap.add_argument("--no-ev3", action="store_true",
                    help="Run camera-only, print commands instead of driving motors")
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
