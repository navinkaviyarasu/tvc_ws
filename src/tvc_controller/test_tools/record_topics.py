#!/usr/bin/env python3
"""Simple ROS2 recorder for TVC test topics.

Subscribes to:
- `/tvc_servo_command` (Float64MultiArray)
- `/joint_states` (sensor_msgs/JointState) [if available]
- `/imu/data` (sensor_msgs/Imu) [if available]

Writes a CSV log file with timestamps and selected fields.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState, Imu
import argparse
import csv
import os


class TopicRecorder(Node):
    def __init__(self, out_file):
        super().__init__('tvc_topic_recorder')
        self.out_file = out_file
        self.csvf = open(self.out_file, 'w', newline='')
        self.writer = csv.writer(self.csvf)
        # header
        self.writer.writerow(['time', 'topic', 'field1', 'field2', 'field3', 'field4'])

        self.create_subscription(Float64MultiArray, '/tvc_servo_command', self.cb_servo, 10)
        # subscribe to optional topics (safe to subscribe even if not published)
        self.create_subscription(JointState, '/joint_states', self.cb_joint, 10)
        self.create_subscription(Imu, '/imu/data', self.cb_imu, 10)

    def now_sec(self):
        # ROS2 time not required; use system time for cross-system alignment
        import time
        return time.time()

    def cb_servo(self, msg: Float64MultiArray):
        t = self.now_sec()
        a = list(msg.data)
        a += [None] * max(0, 2 - len(a))
        self.writer.writerow([t, 'tvc_servo_command', a[0], a[1], None, None])
        self.csvf.flush()

    def cb_joint(self, msg: JointState):
        t = self.now_sec()
        # store first two joint positions if available
        p = list(msg.position)
        p += [None] * max(0, 2 - len(p))
        self.writer.writerow([t, 'joint_states', p[0], p[1], None, None])
        self.csvf.flush()

    def cb_imu(self, msg: Imu):
        t = self.now_sec()
        # write angular velocity z and linear accel z as an example
        avz = msg.angular_velocity.z
        az = msg.linear_acceleration.z
        self.writer.writerow([t, 'imu', avz, az, None, None])
        self.csvf.flush()

    def destroy_node(self):
        try:
            self.csvf.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description='Record TVC topics to CSV')
    parser.add_argument('--out', default='tvc_test_log.csv', help='Output CSV file')
    args = parser.parse_args()

    # ensure output directory exists
    out_dir = os.path.dirname(os.path.abspath(args.out))
    if out_dir and not os.path.exists(out_dir):
        os.makedirs(out_dir)

    rclpy.init()
    node = TopicRecorder(args.out)
    try:
        node.get_logger().info(f'Recording topics to {args.out}. Ctrl-C to stop.')
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
