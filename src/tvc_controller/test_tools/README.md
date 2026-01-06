# TVC Test Tools

This folder contains small utilities to run sine-sweep tests against the TVC plugin using `ros_gz_bridge`.

Files:

- `sine_sweep_publisher.py`: ROS2 node that publishes a sine sweep (chirp) as `std_msgs/msg/Float64MultiArray` to `/tvc_servo_command`.
- `record_topics.py`: ROS2 node that subscribes to `/tvc_servo_command`, `/joint_states`, and `/imu/data` and logs them to CSV.

Quick start

1. Start your Gazebo simulation containing the `viper` model.

2. Start the ros_gz_bridge for the servo command topic (example):
```bash
ros2 run ros_gz_bridge parameter_bridge "/tvc_servo_command@gz.msgs.Double_V[std_msgs/msg/Float64MultiArray]"
```

3. In another terminal, record topics to CSV:
```bash
source install/setup.bash   # your ROS2 workspace
python3 src/tvc_controller/test_tools/record_topics.py --out tvc_run1.csv
```

4. In another terminal, run the sine sweep publisher (example: sweep servo 0 from 0.1->5 Hz over 30 s):
```bash
python3 src/tvc_controller/test_tools/sine_sweep_publisher.py --duration 30 --fstart 0.1 --fend 5.0 --amp 0.005 --neutral 0.093 --servo-index 0
```

5. Stop the recorder after the test and analyze `tvc_run1.csv` with your favorite tools (Python/matplotlib). See `analyze_tvc.py` (not included) for a starting point.

If `ros_gz_bridge` cannot create a converter for `gz.msgs.Double_V` (you may see a warning in the bridge logs), use the Gazebo-native publisher instead:

6. Alternative: publish directly to Gazebo (no bridge)
```bash
# In one terminal (run Gazebo with the viper model)
# In another terminal run the native publisher (this script uses the `gz` CLI):
python3 src/tvc_controller/test_tools/gz_sine_sweep.py --duration 30 --fstart 0.1 --fend 5.0 --amp 0.005 --neutral 0.093 --servo-index 0
```

This directly invokes `gz topic` to publish `gz.msgs.Double_V` messages to `/tvc_servo_command`, bypassing ROS2 entirely.

Notes
- Ensure the ROS2 and ros_gz_bridge environments are set up and sourced.
- If your simulation publishes joint states or IMU under different topic names, update `record_topics.py` accordingly.
