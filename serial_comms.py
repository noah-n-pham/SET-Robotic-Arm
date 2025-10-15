import serial
import time
import math

# --- Serial Setup ---
PORT = "COM3"  # windows ver
BAUD = 9600  # same speed as Ro
ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)  # allow Arduino reset

# --- Arm setup ---
L1 = 10.0  # change later
L2 = 10.0


def inverse_kinematics(x, y, L1, L2):
    cos_theta2 = (x ** 2 + y ** 2 - L1 ** 2 - L2 ** 2) / (2 * L1 * L2)  # This equation gives how bent the arm is.
    cos_theta2 = max(-1, min(1, cos_theta2))  # clamps safely, Rounding errors
    theta2 = math.acos(cos_theta2)  # elbow angle in radians

    k1 = L1 + L2 * math.cos(theta2)
    k2 = L2 * math.sin(theta2)
    theta1 = math.atan2(y, x) - math.atan2(k2, k1)

    return math.degrees(theta1), math.degrees(
        theta2)  # math.acos() and math.atan2() give radians. Servos use degrees, so we convert:
    # Ex Result: two numbers like (45.0, 30.0) → meaning: shoulder servo = 45°, elbow servo = 30°.


def send_angles(theta1, theta2):
    msg = f"{theta1:.2f},{theta2:.2f}\n"  # format: 45.00,30.00\n
    ser.write(msg.encode())
    print(f"Sent: {msg.strip()}")


# --- Main test loop ---
# tests
points = [(10, 0), (10, 10), (5, 10), (10, 5)]
for (x, y) in points:
    t1, t2 = inverse_kinematics(x, y, L1, L2)
    send_angles(t1, t2)
    time.sleep(2)

ser.close()
print("Done!")  # done wahoo
