# RoboCup Soccer Junior 2026 — Plan (EV3 + Camera)

## Official 2026 specs (from league docs, June 3 final rules)
- **Ball:** 42 ± 1 mm **orange** golf ball, 46 ± 3 g. Not white.
- **Field:** 158 × 219 cm green carpet; 20 mm white boundary lines;
  center circle 60 cm; neutral spots at center + 45 cm from each short edge.
- **Goals:** 60 cm wide × 10 cm high × 74 mm deep box; interior matte-colored —
  **one yellow, one blue**; posts sit over the white boundary line.
- **Walls:** 22 cm tall, matte black.
- **Two 10-minute halves**, 5-min break. Goal = ball touches goal back wall.
- **Ball-capturing zone: 1.5 cm** — shot must clear it.
- **No holding the ball except via a dynamic backspin dribbler.**
- Robots must not be fully inside the penalty area (25 cm wide × 80 cm deep).
- Kicker power test: ball bounced off the far goal must not hit the back wall
  of your own goal.

## Hardware map (confirmed with team, 2026-10-22)
| Port | Component | Notes |
|------|-----------|-------|
| **A + B** | Movement wheels | Differential drive (left / right). Speeds −100…100. |
| **C** | Dribbler motor | Forward = trap/roll; reverse = shoot. |
| **3 + 4** | Two white color sensors | Left + right boundary-line guards. Either one seeing white → full stop. |
| **1** | Pixie cam | Not used in this code path. |
| **2** | Gyro | Not used in this code path. |

- Laptop connects to the EV3 over **Bluetooth** via `ev3_dc`
  (EV3 address in `config.py`: `00:16:53:61:AB:61`).
- `ev3dev2` is installed but is for running code **on the brick** — not used
  for the laptop→Bluetooth path.
- The **white boundary lines** around the field do NOT confuse the camera,
  because the camera detects the **orange** ball — white is a different hue
  band. The white sensors (ports 3 + 4) are the boundary guard, not the
  camera.

## How it works (state machine)
Camera has full-field view (mirror cone). The bot cycles through four states
each frame (~30 fps):

```
SEEK (no ball) ──ball found──► CHASE (drive to + grab ball)
                                  │ ball vanished while CLOSE → instant
                                  │ ball not visible ≥ 1.5 s (far) → wait
                                  ▼
                             SHOOT MODE (aim at goal color, reverse-shoot)
                                  │ ball reappears
                                  ▼
                             AIM & SHOOT (fine-tune, fire when aligned)
```

1. **CHASE** — Track the orange ball. Differential drive turns to face it
   (port A vs B differential), forward speed scales down as the turn grows
   so the bot doesn't overshoot. Dribbler spins forward the whole time.
2. **Ball-lost fast path (your 15 cm optimization):**
   - Ball was **large** (area ≥ `BALL_AREA_CLOSE`) when it vanished →
     it's in the dribbler → **immediately** go to SHOOT MODE. No 1.5 s wait.
   - Ball was **small** when it vanished → it just left the field of view →
     wait `LOST_TIMEOUT_S` (1.5 s) before assuming it's in the dribbler.
3. **AIM & SHOOT** — Ball and goal both visible: fire when horizontal
   error < `ALIGN_TOL_PX` AND ball area < `BALL_MAX_AREA` (not sitting in
   the dribbler zone).
4. **Which goal to shoot at** — decided by the referee's orientation at
   kickoff. Operator picks the wrapper file **before the half starts**:
   - `python half_blue.py`   → shoot **blue**, defend yellow
   - `python half_yellow.py`  → shoot **yellow**, defend blue
   - If the ref assigns the opposite orientation, swap the file.

## Boundary guard (white sensors, ports 3 + 4 — "bot does NOT go beyond the white line")
A background thread polls the two white color sensors at 20 Hz in a forever
loop. The instant *either* sensor reads `WHITECOLOR` (debounced over 3
consecutive reads), it **stops all motors** immediately — no waiting for
the vision loop. The main loop then re-searches for the ball and drives
back toward it.

> **Mounting note:** the two sensors must be physically placed on the left
> and right sides of the bot so that each is looking *down* at the boundary
> line as the robot approaches it. Test on the field; the code is correct,
> the mount position is yours to verify.

## Vision
- **Ball (orange golf):** HSV `H 10–25, S 120–255, V 120–255`, min area 400 px.
  Orange is a narrow hue band — much more lighting-robust than white.
  Red sensor lights on other bots must be covered (2026 rule).
- **Goal:** detect the colored goal **interior** (not the posts).
  - Blue:  H 100–130, S 80–255, V 80–255
  - Yellow: H 25–35, S 80–255, V 180–255
  - Largest colored contour = active goal; centroid = aim point.
- **Mirror cone:** expect a warped/double-image frame. The detector takes
  the **largest** contour (the direct view). If the ghost wins on the field,
  add an aspect-ratio or ROI filter — test with the real cone first.
- **White border:** does NOT need camera detection. The white color sensors
  (ports 3 + 4) are the boundary guard. The camera only looks for orange +
  goal color.

## Motion (differential drive, A + B)
- Forward: A and B at the same speed. Turn: A and B opposite.
  `move(forward, turn)`: A = forward + turn, B = forward − turn (clamped to ±100).
- **Grab:** differential turn toward the ball, forward speed scales down
  as the turn grows (no overshoot). Dribbler spinning forward.
- **Shoot:** `dribbler.start(speed=-100)` for `SHOOT_MS` ms, then coast
  (no hard brake, so the ball doesn't eject). `SHOOT_SPEED = 100` = full power.
- **Ball-lost fast path:** see state machine above.

## The single file: `robocup.py`
Everything is packed into one runnable file (your requirement):
```
robocup.py
├── CONFIG section (all HSV, speeds, ports, sensor constants)
├── Vision:  ball_centroid(), goal_centroid()
├── Robot:   Robot class — differential move/stop, dribbler, shoot, white sensors
├── BoundaryGuard: forever-loop thread — white → stop
├── Bot:     state machine — CHASE / SHOOT_MODE / AIM_SHOOT / fast-path
└── main():  --half flag, --no-ev3 flag
```
`calibrate.py` and `tests/` stay as separate helper files (optional, not
needed on the robot).

## Two-script idea (manual half selection)
- `half_blue.py`   → injects `--half 1`, runs `robocup.main()`
- `half_yellow.py`  → injects `--half 2`, runs `robocup.main()`

You pick which file to run before the half starts. Only the goal color
differs; all the logic lives in `robocup.py`. If the ref assigns the
opposite orientation, just run the other file.

## Calibration knobs (in robocup.py CONFIG section — tune on the field)
- `BALL_HSV` — orange range. `GOAL_BLUE_HSV` / `GOAL_YELLOW_HSV` — goal ranges.
- `LOST_TIMEOUT_S` (1.5 s) — fallback wait when ball vanished while far.
- `BALL_AREA_CLOSE` (4000 px²) — the "ball was close when it disappeared"
  threshold for the instant-dribbler fast path.
- `CHASE_SPEED`, `OMEGA_KP`, `DIBBLE_SPEED`
- `SHOOT_SPEED` (100), `SHOOT_MS` (250) — shot power. Calibrate to 3 m and 8 m.
- `ALIGN_TOL_PX` (40) — aim error threshold before firing.
- `WHITE_SENSOR_PORTS` (`[3, 4]`) — ports of the two white color sensors.

## Deployment (GitHub)
The repo is at **https://github.com/pranjalguptahdfc-art/Robocup2026**
(the default branch is `robocup-vision-control`).

**Files the school PC needs to run the bot:**
```
robocup.py        # the whole program — everything is inlined, no other .py needed
half_blue.py      # first half: shoot blue
half_yellow.py    # second half: shoot yellow
```
`calibrate.py` (tuning) and `tests/` are optional.

**At the school PC:**
```
git clone https://github.com/pranjalguptahdfc-art/Robocup2026.git
cd Robocup2026
pip install ev3_dc opencv-python numpy

python robocup.py --half 1 --no-ev3   # webcam-only smoke test, no robot
python half_blue.py                   # run the first half
python half_yellow.py                 # run the second half
```

**EV3 must be:**
- On and paired to the school PC over Bluetooth (MAC in `EV3_ADDRESS`).
- Motors A + B on the large-motor ports, dribbler on C, white sensors on ports 3 + 4.

## File layout
```
README.md             # How to run on the school PC, calibration, hardware map
robocup.py           # ← THE file to run on the robot. Everything is here.
half_blue.py         # First half: shoot blue.  5 lines, calls robocup.main().
half_yellow.py       # Second half: shoot yellow. 5 lines, calls robocup.main().
calibrate.py         # Live HSV overlay tool (optional, for tuning thresholds)
main.py              # Legacy pink-tracker demo (kept for reference, not used)
tests/test_vision.py # Unit tests for vision.py (synthetic HSV frames)
tests/test_robocup_logic.py # Unit tests for robocup.py state-machine flags
```

## Running procedure (laptop + EV3)
**Before running — check the EV3 hardware:**
- EV3 brick is **powered on** and **paired over Bluetooth** with the laptop.
- Motors: **A + B** = large drive motors, **C** = dribbler, **Ports 3 + 4** = white sensors.

**Sequence:**
1. Camera-only test (no robot needed): `python robocup.py --half 1 --no-ev3`
2. First half (shoot blue): `python half_blue.py`
3. Second half (shoot yellow): `python half_yellow.py`

> The EV3 connection happens automatically inside the script — there is no
> separate "connect" step. `NoEV3: No EV3 device found` = brick off or not paired.
> Stop the loop at any time by pressing **`q`** in the terminal.

## Implementation status (as of 2026-10-22)
All code complete on `robocup-vision-control` branch. 8/8 tests passing.
Pushed to GitHub. Camera-only smoke test not yet run on real hardware.

### Remaining known issues
- **DEFERRED (HIGH, on-field first):** mirror-cone double-detection of the
  ball. The detector takes the largest contour; add an aspect-ratio filter
  only after testing with the real cone on the field.
- **ON-FIELD TEST:** the two white sensors' physical mounting positions —
  the code is correct; you must verify each sensor actually sees the line
  when the robot is close to the boundary.
- **ON-FIELD TUNE:** `SHOOT_SPEED`/`SHOOT_MS` — measure actual shot distance
  for 3 m and 8 m. `BALL_AREA_CLOSE` — set it so "close" means ~15 cm.
- **PLANNED:** goalkeeper bot (second EV3) — see "Two-robot setup" below.
- **LOW:** `main.py` line 103 has a `p#` syntax error (legacy, not used).
- **LOW:** `calibrate.py` — add on-frame text showing the active goal color.

## Two-robot setup (planned)
The match uses **two EV3 robots**:

| Role | Behaviour |
|------|-----------|
| **Field bot** (the one in `robocup.py`) | Chase ball, dribble, shoot at target goal. |
| **Goalkeeper** (second EV3) | Stays on the white line at the front of the penalty area,
  tracking the ball's vertical position along that line to block shots.
  Code not yet written. |

**Goalkeeper logic (design):**
- The keeper's camera tracks the ball's `cy` (vertical pixel).
- The keeper only drives **sideways along the white line** — never forward into the penalty area.
- White sensors (ports 3 + 4) serve as the "stay on the line" constraint:
  the keeper is clamped between the two sensors, so it can't drift off the line.
- When the ball is near the keeper's goal, the keeper positions itself
  directly between the ball and the goal interior.
- Keeper does NOT shoot or dribble — it is a blocker.

## Known limitations
- **Mirror cone:** if the ball appears twice in the frame (direct +
  reflected), the detector takes the largest blob. If the ghost image is
  larger, add an aspect-ratio filter.
- **White sensor mounts:** both sensors (ports 3 + 4) must be angled to
  look *down* at the boundary line as the bot approaches it. Test on the
  field.
- **`ev3_dc` vs `ev3dev2`:** `ev3_dc` connects over Bluetooth from the
  laptop. `ev3dev2` runs code *on the brick* — not used in this project.
`ev3_dc` connects to the EV3 Brain over Bluetooth from the laptop.
`ev3dev2` runs code *on the brick itself* (its own microPython env).
Once the camera-based logic is solid on the laptop, the next step is
moving the control loop onto the EV3 so the bot runs without the tether:
1. Port the vision to run on the brick (cv2 not available on the brick —
   use an on-board camera module or stream pixel data over BLE).
2. Port the motor commands to `ev3dev2`'s motor API on the brick.
3. The HSV thresholds stay the same; only the transport layer changes.

## Testing
- Unit tests (no hardware): `python -m pytest tests/ -v`
- Camera-only smoke test: `python robocup.py --half 1 --no-ev3`
- HSV calibration overlay: `python calibrate.py` (green = ball, blue = goal,
  press `b` to toggle goal color, `q` to quit)

## Calibration Log (fill in on each field session)
| Item | Value |
|------|-------|
| Ball HSV (observed) | H:__–__ S:__–__ V:__–__ |
| Blue goal HSV (observed) | H:__–__ S:__–__ V:__–__ |
| Yellow goal HSV (observed) | H:__–__ S:__–__ V:__–__ |
| ALIGN_TOL_PX tuned | __ |
| BALL_AREA_CLOSE (≈15 cm in px²) | __ |
| SHOOT_MS for ~3 m | __ |
| SHOOT_MS for ~8 m | __ |
| LOST_TIMEOUT_S tuned | __ s |
| White sensor mount position OK? | Y / N |
| Both goals visible simultaneously? | Y / N |
| Mirror cone warp: goal centroid offset | __ px |
