import time
from gz.transport import Node
from gz.msgs import double_pb2, double_v_pb2

# --- Configuration ---
# 1. Thrust Command (Motor Speed)
MOTOR_SPEED_TOPIC = "/model/TVC/motor_speed"
MAX_THRUST_SPEED_RAD_S = 800.0  # From your SDF, maxRotVelocity

# 2. TVC Command (Servo Lengths in meters)
SERVO_COMMAND_TOPIC = "/tvc_servo_command"
# Servo neutral length (~0.0935m) and limits from TVCPlugin.cc
SERVO_NEUTRAL_LEN = 0.0935
SERVO_MIN_LEN = 0.085
SERVO_MAX_LEN = 0.115

# Initialize Gazebo Transport node
node = Node()

# Setup publishers
thrust_pub = node.advertise(MOTOR_SPEED_TOPIC, double_pb2.Double)
servo_pub = node.advertise(SERVO_COMMAND_TOPIC, double_v_pb2.Double_V)

def publish_thrust_and_tvc(motor_speed_rad_s, servo1_len, servo2_len):
    """
    Publishes commands to control both the thrust and the TVC servos.
    """
    
    # 1. Publish Thrust Command (Double message)
    thrust_msg = double_pb2.Double()
    thrust_msg.data = motor_speed_rad_s
    thrust_pub.publish(thrust_msg)
    
    # 2. Publish TVC Servo Command (Double_V message)
    servo_msg = double_v_pb2.Double_V()
    # Ensure values are within physical limits (TVCPlugin clamps them, but good practice here too)
    servo_msg.data.extend([
        max(SERVO_MIN_LEN, min(SERVO_MAX_LEN, servo1_len)),
        max(SERVO_MIN_LEN, min(SERVO_MAX_LEN, servo2_len))
    ])
    servo_pub.publish(servo_msg)

    print(f"Published Thrust: {motor_speed_rad_s:.2f} rad/s | Servos: ({servo1_len:.4f}m, {servo2_len:.4f}m)")


# --- Example Control Sequence ---

# 1. Initial State: Neutral Gimbal, Zero Thrust
print("--- Initializing: Neutral (L0) and Zero Thrust ---")
publish_thrust_and_tvc(
    motor_speed_rad_s=0.0,
    servo1_len=SERVO_NEUTRAL_LEN,
    servo2_len=SERVO_NEUTRAL_LEN
)
time.sleep(1)

# 2. Activate Motor
print("\n--- Step 2: Full Thrust, Neutral Gimbal ---")
# Start motor at max speed (corresponds to max thrust)
publish_thrust_and_tvc(
    motor_speed_rad_s=MAX_THRUST_SPEED_RAD_S,
    servo1_len=SERVO_NEUTRAL_LEN,
    servo2_len=SERVO_NEUTRAL_LEN
)
time.sleep(3) # Let the motor spin up and the rocket lift off

# 3. Apply Pitch Correction (Servo 2)
print("\n--- Step 3: Apply Max Pitch Correction (Extending Servo 2) ---")
# Servo 2 (Pitch control) extends to max length (115mm)
# This should tilt the motor housing in the positive pitch direction (around Y-axis)
publish_thrust_and_tvc(
    motor_speed_rad_s=MAX_THRUST_SPEED_RAD_S,
    servo1_len=SERVO_NEUTRAL_LEN, # Neutral
    servo2_len=SERVO_MAX_LEN     # Max Pitch deflection
)
time.sleep(3)

# 4. Return to Neutral
print("\n--- Step 4: Return to Neutral Gimbal ---")
publish_thrust_and_tvc(
    motor_speed_rad_s=MAX_THRUST_SPEED_RAD_S,
    servo1_len=SERVO_NEUTRAL_LEN,
    servo2_len=SERVO_NEUTRAL_LEN
)
time.sleep(3)

# 5. Shut down motor
print("\n--- Step 5: Shutting down motor ---")
publish_thrust_and_tvc(
    motor_speed_rad_s=0.0,
    servo1_len=SERVO_NEUTRAL_LEN,
    servo2_len=SERVO_NEUTRAL_LEN
)
time.sleep(1)
print("\nControl sequence finished.")
