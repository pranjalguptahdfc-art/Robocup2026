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

## How it works (state machine)
The bot cycles through four states each frame; camera has full-field view
(via mirror cone), so it sees both goals, the ball, and its own position:

```
SEEK (no ball) ──ball found──► CHASE (drive to + grab ball)
                                  │ ball not visible ≥ 1.5 s
                                  ▼
                             SHOOT MODE (aim at goal color, reverse-shoot)
                                  │ ball reappears
                                  ▼
                             AIM & SHOOT (fine-tune, fire when aligned)
```

1. **CHASE** — Track the orange ball. Turn (port A / omni base) to face it,
   drive forward until ball is within the dribbler, dribbler spinning
   forward (clockwise) to trap/roll.
2. **SHOOT MODE** — Ball not visible ≥ 1.5 s = it's in the dribbler.
   Aim at the active goal color, drive until the bot–ball–goal line aligns,
   then **reverse the dribbler (anticlockwise)** to shoot.
3. **AIM & SHOOT** — Ball and goal both visible: aim through the ball's
   centroid at the goal's centroid; fire when horizontal error < ALIGN_TOL.
4. **Which goal to shoot at** — decided by the referee's orientation at
   kickoff. Operator loads the mode **before the half starts**:
   - `python robocup.py --half 1` → shoot **blue**, defend **yellow**
   - `python robocup.py --half 2` → shoot **yellow**, defend **blue**
   - One code path; `--half` just flips the goal color constant.
   - If the ref assigns the opposite orientation, swap the flag.

## Vision
- **Ball (orange golf):** HSV `H 10–25, S 120–255, V 120–255`, min area ~400 px.
  Orange is a narrow hue band — much more lighting-robust than white.
  Red sensor lights on other bots must be covered (2026 rule) so they don't
  trigger the ball detector.
- **Goal:** detect the colored goal **interior** (not the posts).
  - Blue:  H 100–130, S 80–255, V 80–255
  - Yellow: H 25–35, S 80–255, V 180–255
  - Largest colored contour = active goal; centroid = aim point;
    area → rough distance estimate for shot power.
- **Mirror cone:** expect a warped/double-image frame. Calibration step
  must map goal-centroid → drive angle on the real field.

## Motion
- 3-omni base: ports B/C/D → `vx, vy, omega`; port A = dribbler.
- **Grab:** proportional `omega` on ball x-error, drive forward until ball
  centroid nears screen center; dribbler spinning forward the whole time.
- **Shoot:** `dribbler.start(speed=-SHOOT_SPEED)` for `SHOOT_MS`, stop.
  Shot distance ∝ speed × duration — calibrate to kick a 42 mm ball
  ~10–20 m… no, ~2–3 m on a 158 × 219 cm field.
- **Ball lost ≥ 1.5 s:** assume in dribbler; switch to goal-aiming + shoot.

## Calibration knobs (config.py — tune on the field)
- `BALL_*`, `GOAL_BLUE_*`, `GOAL_YELLOW_*` HSV ranges + min areas
- `LOST_TIMEOUT_S` (start 1.5 s)
- `CHASE_SPEED`, `OMEGA_KP`, `DIBBLE_SPEED`
- `SHOOT_SPEED`, `SHOOT_MS` — two presets: short (3 m) and long (10 m)
- `ALIGN_TOL_PX` — horizontal aim error threshold before firing

## File layout
```
main.py            # legacy pink tracker (kept, not used)
robocup.py        # entry: connect, webcam, state machine, --half flag
robot.py          # EV3 wrapper: omni drive, stop, dribbler fwd/reverse, shoot()
vision.py         # HSV masks: detect_ball, detect_goal(color) → centroid + area
config.py         # all HSV ranges, speeds, timeouts, thresholds
calibrate.py      # live overlay tool: prints ball + goal centroids for tuning
```

## Open questions / next steps
1. Confirm the mirror cone's actual FOV/warp on this field → calibrate
   goal-centroid → drive-angle mapping.
2. Measure shot distance vs `SHOOT_SPEED`/`SHOOT_MS` for 2 m, 5 m, 10 m.
3. Check whether both goals are visible in the same frame; if so, make sure
   the detector only acts on the goal color the `--half` flag selected.
4. Confirm that red sensor lights on other teams' bots are covered —
   they look orange and will false-trigger the ball detector.

## Testing plan
- Each vision function: unit test with synthetic HSV images.
- `robocup.py --no-ev3` runs the camera-only loop and prints motor commands —
  iterate thresholds without the robot wired up.
- Field calibration via `calibrate.py` before every practice session.
