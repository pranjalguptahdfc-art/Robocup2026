# RoboCup Soccer Junior 2026 — Autonomous EV3 Soccer Bot

An autonomous RoboCup soccer robot that uses a top-mounted webcam to find
the orange ball, chase it, aim at the goal, and shoot with a reverse-spin
dribbler. The bot also monitors a white boundary sensor so it does not
leave the field.

---

## Quick Start

**On the school PC (or any laptop with a webcam and the EV3 paired via
Bluetooth):**

```
git clone https://github.com/pranjalguptahdfc-art/Robocup2026.git
cd Robocup2026
pip install ev3_dc opencv-python numpy
```

---

## Files You Need to Run the Bot

These three files are the only ones required on the robot:

| File | What it does |
|------|-------------|
| `robocup.py` | The entire program — all vision, motor control, state machine, and the white-sensor boundary guard. No other `.py` file is imported at runtime. |
| `half_blue.py` | Runs the **first half** (shoot the **blue** goal, defend yellow). |
| `half_yellow.py` | Runs the **second half** (shoot the **yellow** goal, defend blue). |

`calibrate.py` is an optional live-overlay tool for tuning the HSV color
ranges before a match. `tests/` are unit tests (optional).

---

## How to Run

### 1. Test the camera loop first (no robot needed)
This confirms your webcam works and prints the motor commands to the
screen — no EV3 connection:
```
python robocup.py --half 1 --no-ev3
```

### 2. Run with the real EV3
Before starting, make sure:
- The EV3 brick is **powered on** and **paired over Bluetooth** with this laptop.
- The two white boundary sensors are on **ports 3 and 4** of the EV3.
- The two large drive motors are on **ports A and B**.
- The dribbler medium motor is on **port C**.

Then run:
```
python half_blue.py     # first half — shoot blue goal
python half_yellow.py   # second half — shoot yellow goal
```
Stop the program at any time by pressing **`q`** in the terminal.

> The EV3 connects automatically when the script starts — there is no
> separate "connect" step. If you see `NoEV3: No EV3 device found`,
> the EV3 is off or not paired.

### 3. Which file to run for which half
The referee tells you which side is which at kickoff. If you are to
**shoot at the blue goal**, run `half_blue.py`. If you are to shoot at
**the yellow goal**, run `half_yellow.py`.

---

## What the Bot Does

```
SEEK ──ball found──► CHASE (drive toward ball, dribbler spinning forward)
                          │
        ┌─────────────────┴──────────────────┐
        │ ball was CLOSE when it vanished    │ ball vanished while FAR
        │ → instant SHOOT MODE               │ → wait 1.5 s, then SHOOT MODE
        ▼                                     ▼
   SHOOT MODE (aim at goal, drive in, fire dribbler in reverse)
        │
        └── ball reappears → back to CHASE or AIM & SHOOT
```

**White boundary guard (background thread):** Two color sensors on ports 3
and 4 poll at 20 Hz. The instant *either* reads white (debounced over 3
frames), all motors stop — the bot never crosses the white boundary line.

**Ball-lost fast path:** If the ball's image was large when it
disappeared, it is assumed to be in the dribbler and the bot goes
straight to shoot mode without waiting 1.5 s.

---

## Configuration

All tunable constants are at the top of `robocup.py` in the
**CONFIG section**. The most important ones:

| Constant | Default | What it does |
|----------|---------|-------------|
| `EV3_ADDRESS` | `00:16:53:61:AB:61` | Your EV3's Bluetooth MAC. Change if different. |
| `BALL_HSV` | orange range | HSV window for the orange ball. Tune with `calibrate.py`. |
| `GOAL_BLUE_HSV` / `GOAL_YELLOW_HSV` | — | HSV window for each goal's interior color. |
| `BALL_AREA_CLOSE` | `4000` px² | If the ball's blob is this large when it vanishes, it is in the dribbler. Set to match your camera view at ~15 cm. |
| `LOST_TIMEOUT_S` | `1.5` s | Fallback wait when the ball vanished while far away. |
| `CHASE_SPEED` | `50` | Forward speed while chasing the ball (0–100). |
| `OMEGA_KP` | `0.2` | Turn sensitivity. Increase for sharper turning. |
| `SHOOT_SPEED` | `100` | Dribbler reverse speed. 100 = full power. |
| `SHOOT_MS` | `250` | How long the reverse burst lasts, in ms. Calibrate to reach 3 m and 8 m. |
| `ALIGN_TOL_PX` | `40` px | How close the ball and goal must be horizontally before firing. |
| `WHITE_SENSOR_PORTS` | `[3, 4]` | Ports of the two white boundary sensors (left + right). |

---

## Calibration (Before Every Match)

Run the calibration tool to confirm the HSV ranges match the real
field lighting:
```
python calibrate.py
```
- **Green circle** = detected orange ball.
- **Blue circle** = detected goal. Press **`b`** to toggle between blue/yellow goal detection.
- Press **`q`** to quit.

If the circles are not on the right objects, adjust `BALL_HSV`,
`GOAL_BLUE_HSV`, or `GOAL_YELLOW_HSV` in `robocup.py` and rerun.

Also measure and record:
- `SHOOT_MS` needed to reach 3 m and 8 m from the goal
- `BALL_AREA_CLOSE` value that corresponds to ~15 cm on your camera
- `WHITE_SENSOR_PORTS` / sensor mount positions (each sensor must be angled
  down so it sees the white line as the bot approaches it)

---

## Hardware Summary

| Port | Component | Notes |
|------|-----------|-------|
| **A + B** | Large drive motors | Differential drive. Left/right wheel. Speeds −100…100. |
| **C** | Medium dribbler motor | Forward = trap/roll ball. Reverse = shoot. |
| **3** | White color sensor (left) | Left boundary-line guard. |
| **4** | White color sensor (right) | Right boundary-line guard. |
| **Webcam** | Laptop camera | Must have a clear view of the field. Mirror cone optional. |

---

## Testing

Run the unit tests (no hardware required):
```
python -m pytest tests/ -v
```

Camera-only smoke test:
```
python robocup.py --half 1 --no-ev3
```

---

## Known Limitations

- **Mirror cone:** If the ball appears twice in the frame (direct + reflected
  image), the detector takes the largest blob (usually the direct view). If
  the ghost image is larger, an aspect-ratio filter will need to be added.
- **White sensor mounts:** Both sensors (ports 3 + 4) must be physically
  positioned to look *down* at the boundary line as the bot approaches it.
  Test this on the field.
- **`ev3_dc`** connects over Bluetooth from the laptop. `ev3dev2` is a
  separate library for running code *on the EV3 brick itself* — not used here.

---

## Two-Robot Setup (Planned)

The team will run **two EV3 robots** during a match:

| Role | Behaviour |
|------|-----------|
| **Field bot** | The current `robocup.py` — chases the ball, dribbles, and shoots at the target goal. |
| **Goalkeeper** | A second EV3 that stays on the white line at the front of the penalty area, tracking the ball's position along that line to block shots. Not yet coded. |

**Goalkeeper design (for when we build it):**
- Keeper's camera tracks the ball's vertical position (`cy`) in the frame.
- Keeper only drives **sideways along the white line** — never forward into the penalty area.
- The two white sensors (ports 3 + 4) act as the "stay on the line" constraint.
- When the ball is near the keeper's goal, the keeper positions itself directly between the ball and the goal interior.
- The keeper does NOT dribble or shoot — it is a pure blocker.
