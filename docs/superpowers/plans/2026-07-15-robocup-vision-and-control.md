# RoboCup EV3 Orange-Ball Vision + Autonomous Control — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Autonomous EV3 soccer bot with a top-mounted webcam + mirror cone that finds the 42 mm orange golf ball, drives to it, traps it with a forward-spinning dribbler, aims at the referee-assigned goal color (blue = half 1, yellow = half 2), and shoots via reverse-dribbler drive.

**Architecture:** 4-state machine (SEEK / CHASE / SHOOT_MODE / AIM_SHOOT) driven by per-frame HSV vision. `--half` flag picks goal color; one code path for both orientations. All magic numbers in `config.py`. `--no-ev3` flag lets the loop run camera-only for threshold iteration.

**Tech Stack:** Python 3, OpenCV, `ev3_dc` (Bluetooth), 3-omni base (B/C/D) + dribbler motor (A).

**Spec:** `CLAUDE.md` at repo root; official docs: `RoboCupJunior_Field_Ball_Specs_2026.docx`, `RoboCupJunior_Soccer_Rules_2026_Summary.docx`.

## Global Constraints (from official 2026 rules)
- **Ball:** 42 ± 1 mm orange golf ball, 46 ± 3 g.
- **Goals:** 60 cm × 10 cm × 74 mm box; interior yellow or blue (one each).
- **Field:** 158 × 219 cm; 20 mm white lines; center circle 60 cm.
- **Ball-capturing zone: 1.5 cm** — shot must clear it.
- **Two 10-minute halves.** Goal = ball touches back wall.
- **No orange/yellow/blue on the robot body** (interference rule).
- **Red sensor lights on other bots must be covered** — they false-trigger.
- EV3 address: `00:16:53:61:AB:61`.
- Motors: B/C/D = omni wheels; A = dribbler (forward = trap, reverse = shoot).

## HSV Starting Ranges (calibrate on-field; orange is the ball)
- **Orange ball:** H 10–25, S 120–255, V 120–255, min_area 400
- **Blue goal:**  H 100–130, S 80–255, V 80–255, min_area 300
- **Yellow goal:** H 25–35, S 80–255, V 180–255, min_area 300
- `LOST_TIMEOUT_S = 1.5`

---

## Task 1: `config.py` — central tuning constants

**Files:**
- Create: `config.py`

**Interfaces:**
- Produces: all constants used by later tasks. Single import point.

- [ ] **Step 1: Create `config.py`**

```python
# RoboCup 2026 EV3 bot — tunable knobs. Calibrate on-field before every session.
EV3_ADDRESS = "00:16:53:61:AB:61"

# Ball: 42 mm orange golf ball
BALL_HSV = dict(lower=(10, 120, 120), upper=(25, 255, 255), min_area=400)

# Goals (interior color, not posts)
GOAL_BLUE_HSV   = dict(lower=(100, 80, 80),  upper=(130, 255, 255), min_area=300)
GOAL_YELLOW_HSV  = dict(lower=(25, 80, 180),  upper=(35, 255, 255),  min_area=300)
GOAL_BY_NAME = {"blue": GOAL_BLUE_HSV, "yellow": GOAL_YELLOW_HSV}

# Timings
LOST_TIMEOUT_S = 1.5          # ball not visible this long → assume in dribbler

# Motion
CHASE_SPEED    = 50          # forward speed toward ball (0–100)
OMEGA_KP       = 0.2         # turn-error gain (px error → omega)
DIBBLE_SPEED   = 60          # dribbler forward (trap/roll)
SHOOT_SPEED    = 70          # dribbler reverse speed magnitude
SHOOT_MS       = 250         # dribbler reverse duration (ms)
ALIGN_TOL_PX   = 40          # |ball_x − goal_x| below this → fire
```

- [ ] **Step 2: Commit**

```bash
git add config.py
git commit -m "feat: config.py with orange-ball HSV, goal colors, and motion knobs"
```

---

## Task 2: `robot.py` — EV3 motion wrapper

**Files:**
- Create: `robot.py`

**Interfaces:**
- Produces:
  - `class Robot`: `__init__(no_ev3: bool = False)`. When `no_ev3=True`,
    all commands print to console and do not touch hardware.
  - `Robot.omni(vx: int, vy: int, omega: int)`
  - `Robot.stop()`
  - `Robot.dribbler_forward(speed: int | None = None)`
  - `Robot.dribbler_reverse(speed: int | None = None)`
  - `Robot.shoot()` — reverse-dribbler for `SHOOT_MS`, then release.
  - `Robot.idle()` — stop omni + dribbler.

- [ ] **Step 1: Write `robot.py`**

```python
import math, time
import ev3_dc as ev3
import config

class Robot:
    def __init__(self, no_ev3=False):
        self.no_ev3 = no_ev3
        if no_ev3:
            self.w1 = self.w2 = self.w3 = self.dribbler = None
            return
        r = ev3.EV3(protocol=ev3.BLUETOOTH, host=config.EV3_ADDRESS)
        self.w1 = ev3.Motor(ev3.PORT_B, ev3_obj=r)
        self.w2 = ev3.Motor(ev3.PORT_C, ev3_obj=r)
        self.w3 = ev3.Motor(ev3.PORT_D, ev3_obj=r)
        self.dribbler = ev3.Motor(ev3.PORT_A, ev3_obj=r)

    def omni(self, vx, vy, omega):
        if self.no_ev3:
            print(f"omni vx={vx:>3} vy={vy:>3} om={omega:>3}")
            return
        s1 = vx + omega
        s2 = -0.5*vx + (math.sqrt(3)/2)*vy + omega
        s3 = -0.5*vx - (math.sqrt(3)/2)*vy + omega
        m = max(abs(s1), abs(s2), abs(s3), 100)
        f = 100.0 / m
        self.w1.start(int(s1*f)); self.w2.start(int(s2*f)); self.w3.start(int(s3*f))

    def stop(self):
        if self.no_ev3:
            return
        for w in (self.w1, self.w2, self.w3):
            w.stop(brake=True)

    def dribbler_forward(self, speed=None):
        if self.no_ev3:
            return
        s = config.DIBBLE_SPEED if speed is None else speed
        self.dribbler.start(speed=s)

    def dribbler_reverse(self, speed=None):
        if self.no_ev3:
            return
        s = config.SHOOT_SPEED if speed is None else speed
        self.dribbler.start(speed=-s)

    def shoot(self):
        if self.no_ev3:
            print(">>> SHOOT <<<")
            time.sleep(config.SHOOT_MS / 1000.0)
            return
        self.dribbler_reverse()
        time.sleep(config.SHOOT_MS / 1000.0)
        self.dribbler.stop(brake=False)

    def idle(self):
        self.stop()
        if not self.no_ev3:
            self.dribbler.stop(brake=True)
```

- [ ] **Step 2: Commit**

```bash
git add robot.py
git commit -m "feat: robot.py wraps EV3 omni drive + dribbler with no_ev3 flag"
```

---

## Task 3: `vision.py` — orange-ball and goal detection

**Files:**
- Create: `vision.py`
- Test: `tests/test_vision.py`

**Interfaces:**
- Produces:
  - `vision.ball_spec() -> dict` — returns `config.BALL_HSV`.
  - `vision.goal_spec(color: str) -> dict` — returns blue or yellow spec.
  - `vision.ball_centroid(hsv, spec=None) -> (cx, cy, area) | None`
  - `vision.goal_centroid(hsv, spec=None) -> (cx, cy, area) | None`

- [ ] **Step 1: Write failing tests**

```python
# tests/test_vision.py
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
```

- [ ] **Step 2: Run tests — they fail (module missing)**

```bash
python -m pytest tests/test_vision.py -v
```

Expected: ImportError / ModuleNotFoundError for `vision`.

- [ ] **Step 3: Write `vision.py`**

```python
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
```

- [ ] **Step 4: Run tests — they pass**

```bash
python -m pytest tests/test_vision.py -v
```

- [ ] **Step 5: Commit**

```bash
git add vision.py tests/test_vision.py
git commit -m "feat: vision.py orange-ball + goal HSV detection with unit tests"
```

---

## Task 4: `robocup.py` — main loop, state machine, `--half` flag

**Files:**
- Create: `robocup.py`

**Interfaces:**
- Consumes: `Robot` (robot.py), `ball_centroid`, `goal_centroid` (vision.py),
  `config` constants.
- Produces: `python robocup.py --half 1|2 [--no-ev3]`
  - `--half 1` → shoot **blue**, defend **yellow**
  - `--half 2` → shoot **yellow**, defend **blue**

- [ ] **Step 1: Write `robocup.py`**

```python
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
        return abs(bx - gx) < config.ALIGN_TOL_PX

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
```

- [ ] **Step 2: Commit**

```bash
git add robocup.py
git commit -m "feat: robocup.py state machine with --half flag (blue/yellow goal)"
```

---

## Task 5: `calibrate.py` — live HSV tuning tool

**Files:**
- Create: `calibrate.py`
- Modify: `CLAUDE.md` — add Calibration Log section

**Interfaces:**
- Produces: a camera window with green circle on ball, blue circle on goal,
  and console print of live centroids + areas. Press `b` to toggle goal
  color between blue and yellow. `q` to quit.

- [ ] **Step 1: Write `calibrate.py`**

```python
import cv2
import config, vision

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    raise SystemExit("no camera")

goal_color = "blue"
print("Green = ball, Blue = goal. Press 'b' to toggle goal color, 'q' to quit.")

while True:
    ok, frame = cap.read()
    if not ok:
        break
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    ball = vision.ball_centroid(hsv, vision.ball_spec())
    goal = vision.goal_centroid(hsv, vision.goal_spec(goal_color))

    if ball:
        cv2.circle(frame, ball[:2], 8, (0, 255, 0), -1)
        cv2.putText(frame, f"B {ball[0]},{ball[1]} a={ball[2]}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    if goal:
        cv2.circle(frame, goal[:2], 8, (255, 0, 0), -1)
        cv2.putText(frame, f"G[{goal_color}] {goal[0]},{goal[1]} a={goal[2]}",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

    k = cv2.waitKey(1) & 0xFF
    if k == ord("b"):
        goal_color = "blue" if goal_color == "yellow" else "yellow"
    if k == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
```

- [ ] **Step 2: Add Calibration Log to `CLAUDE.md`**

Append this section at the end of `CLAUDE.md`:

```markdown
## Calibration Log (fill in on each field session)
| Item | Value |
|------|-------|
| Ball HSV (observed) | H:__–__ S:__–__ V:__–__ |
| Blue goal HSV (observed) | H:__–__ S:__–__ V:__–__ |
| Yellow goal HSV (observed) | H:__–__ S:__–__ V:__–__ |
| ALIGN_TOL_PX tuned | __ |
| SHOOT_SPEED / SHOOT_MS for ~3 m | __ / __ ms |
| SHOOT_SPEED / SHOOT_MS for ~8 m | __ / __ ms |
| LOST_TIMEOUT_S tuned | __ s |
| Both goals visible simultaneously? | Y / N |
| Mirror cone warp: goal centroid offset | __ px |
```

- [ ] **Step 3: Commit**

```bash
git add calibrate.py CLAUDE.md
git commit -m "feat: calibrate.py live HSV tuning tool + calibration log in CLAUDE.md"
```

---

## Task 6: Smoke test — `--no-ev3` camera-only run

**Files:**
- No new files.

**Interfaces:**
- Verifies: `robocup.py` runs without hardware and prints commands.

- [ ] **Step 1: Run `robocup.py --no-ev3`**

```bash
python robocup.py --half 1 --no-ev3
```

Expected: console prints `omni vx=... vy=... om=...` lines when ball visible;
prints `>>> SHOOT <<<` when goal is visible and ball has been lost for
> 1.5 s. Press `q` to exit cleanly. No motor errors.

- [ ] **Step 2: Run `--half 2` variant**

```bash
python robocup.py --half 2 --no-ev3
```

Expected: same loop but `goal_spec("yellow")` is used; goal circle appears
on yellow areas, not blue.

- [ ] **Step 3: Commit any fixes found**

```bash
git add -A
git commit -m "fix: no-ev3 smoke test adjustments"
```

---

## Self-Review

- **Spec coverage:**
  - Orange ball detection ✓ Task 3 + Task 5 calibration
  - Blue/yellow goal per `--half` ✓ Task 4 (`--half 1` = blue, `--half 2` = yellow)
  - Dribbler forward (trap) + reverse (shoot) ✓ Task 2
  - Ball-lost → assume in dribbler → shoot ✓ Task 4 (`LOST_TIMEOUT_S` path)
  - Full-field camera + mirror cone ✓ noted in CLAUDE.md open questions
  - 1.5 cm ball-capturing zone ✓ noted in Global Constraints; `SHOOT_MS` is the knob
  - Kicker power test (no self-goal) ✓ open question in CLAUDE.md

- **Placeholders:** None. All code blocks are complete and runnable.
- **Type consistency:**
  - `ball_centroid` / `goal_centroid` return `(cx, cy, area)` tuples in all 3 tasks
  - `Robot.shoot()` uses `config.SHOOT_SPEED` and `config.SHOOT_MS` — defined in Task 1
  - `goal_spec("blue")` returns `GOAL_BLUE_HSV` dict; `goal_spec("yellow")` returns `GOAL_YELLOW_HSV` — both keys exist in `GOAL_BY_NAME` from Task 1
  - `--half 1` = blue is consistent across Task 4 code, CLAUDE.md, and plan header

## Execution Handoff

**Plan complete and saved to `docs/superpowers/plans/2026-07-15-robocup-vision-and-control.md`.**
Two execution options:

1. **Subagent-Driven (recommended)** — I dispatch a fresh subagent per task,
   review between tasks, fast iteration.
2. **Inline Execution** — Execute tasks in this session using
   executing-plans, batch execution with checkpoints.

Which approach?
