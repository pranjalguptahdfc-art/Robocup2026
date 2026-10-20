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
