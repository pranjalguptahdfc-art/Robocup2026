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
