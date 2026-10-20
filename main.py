import cv2
import numpy as np
import math
import ev3_dc as ev3
 
# ---------------- EV3 SETTINGS ----------------
EV3_ADDRESS = "00:16:53:61:AB:61"
 
# ---------------- PINK DETECTION & FRAME SETTINGS ----------------
LOWER_PINK = np.array([140, 50, 100])
UPPER_PINK = np.array([179, 255, 255])
MIN_PIXELS = 500  # Minimum pixels to consider the object found
 
# ---------------- CONNECT TO EV3 ----------------
print("Connecting to EV3...")
robot = ev3.EV3(protocol=ev3.BLUETOOTH, host=EV3_ADDRESS)
 
# Initialize Large Motors for 3-Omni Movement (Ports B, C, D)
wheel1 = ev3.Motor(ev3.PORT_B, ev3_obj=robot)
wheel2 = ev3.Motor(ev3.PORT_C, ev3_obj=robot)
wheel3 = ev3.Motor(ev3.PORT_D, ev3_obj=robot)
 
# Initialize Medium Motor for Dribbler (Port A)
dribbler = ev3.Motor(ev3.PORT_A, ev3_obj=robot)
 
print("EV3 connected successfully!")
 
# ---------------- START WEBCAM ----------------
camera = cv2.VideoCapture(0)
if not camera.isOpened():
    print("Could not open webcam.")
    raise SystemExit
 
def set_omni_velocity(vx, vy, omega):
    """
    Calculates and sends speed commands for a 3-omni-wheel robot.
    :param vx: Forward/Backward velocity (-100 to 100)
    :param vy: Left/Right strafing velocity (-100 to 100)
    :param omega: Rotational turning speed (-100 to 100)
    """
    # 3-omni wheel kinematic equations
    speed1 = vx + omega
    speed2 = (-0.5 * vx) + (math.sqrt(3)/2 * vy) + omega
    speed3 = (-0.5 * vx) - (math.sqrt(3)/2 * vy) + omega
    # Scale speeds to stay within valid motor power limits (-100 to 100)
    max_val = max(abs(speed1), abs(speed2), abs(speed3), 100)
    scale = 100.0 / max_val
    s1 = int(speed1 * scale)
    s2 = int(speed2 * scale)
    s3 = int(speed3 * scale)
    # Send power commands via ev3_dc
    wheel1.start(speed=s1)
    wheel2.start(speed=s2)
    wheel3.start(speed=s3)
 
def stop_robot():
    """Stops all movement motors."""
    wheel1.stop(brake=True)
    wheel2.stop(brake=True)
    wheel3.stop(brake=True)
 
print("Tracking started. Press 'Q' to quit.")
 
try:
    # Turn on the dribbler motor continuously
    dribbler.start(speed=60)
    while True:
        ret, frame = camera.read()
        if not ret:
            break
 
        # Get frame dimensions to find the center of the screen
        height, width, _ = frame.shape
        center_x = width // 2
 
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        pink_mask = cv2.inRange(hsv, LOWER_PINK, UPPER_PINK)
        # Find contours of the pink object to get its position
        contours, _ = cv2.findContours(pink_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
 
        if contours:
            # Find the largest pink contour (the ball)
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
 
            if area > MIN_PIXELS:
                # Get the bounding box / center of the ball
                M = cv2.moments(largest_contour)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
 
                    # Draw visual indicators on the video feed
                    cv2.circle(frame, (cx, cy), 10, (0, 255, 0), -1)
                    # Calculate tracking error relative to screen center
                    error_x = cx - center_x  # Horizontal offset (-width/2 to width/2)
 
                    # Steering logic for RoboCup:
                    # If ball is to the left/right, adjust omega (spin) to face it.
                    # Move forward towards the ball.
                    vx = 50  # Drive forward speed towards the ball
                    vy = 0   # No strafing for now, or map it if needed
                    omega = int(error_x * 0.2)  # proportional turning to center the ball
 
                    set_omni_velocity(vx, vy, omega)
                    status = f"TRACKING BALL | Error X: {error_x}"
                    color = (0, 255, 0)
                else:
                    stop_robot()
                    status = "SEARCHING..."
                    color = (0, 0, 255)
            else:
                stop_robot()
                status = "BALL TOO SMALL"
                color = (0, 0, 255)
        else:
            stop_robot()
            status = "NO PINK DETECTED - STOPPED"
            color = (0, 0, 255)
 
        # Display status overlay on the camera window
        cv2.putText(frame, status, (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        cv2.imshow("RoboCup Vision Tracking", frame)
        cv2.imshow("Pink Mask", pink_mask)
 
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
 
finally:
    # Cleanup safely when exiting
    try:
        stop_robot()
        dribbler.stop(brake=True)
    except Exception:
        pass
 
    camera.release()
    cv2.destroyAllWindows()
    print("Program terminated successfully.")