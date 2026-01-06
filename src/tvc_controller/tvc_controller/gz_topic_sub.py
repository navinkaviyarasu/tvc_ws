import gz.transport as gz
import gz.msgs.pose_v_pb2 as pose_v_pb2

# Callback function when a message is received
def pose_callback(msg):
    # Parse the Pose_V message
    pose_v = pose_v_pb2.Pose_V()
    pose_v.ParseFromString(msg)

    print(f"Received {len(pose_v.pose)} poses:")
    for i, p in enumerate(pose_v.pose):
        print(f"  Pose {i}: x={p.position.x}, y={p.position.y}, z={p.position.z}")
        print(f"          orientation: x={p.orientation.x}, y={p.orientation.y}, z={p.orientation.z}, w={p.orientation.w}")

# Create a Gazebo Transport node
node = gz.Node()

# Subscribe to the Pose_V topic
node.subscribe("/model/tvc_0/pose", pose_callback)

print("Listening to /model/tvc_0/pose ... Press Ctrl+C to exit.")

# Keep the script alive
try:
    while True:
        pass
except KeyboardInterrupt:
    print("Exiting...")