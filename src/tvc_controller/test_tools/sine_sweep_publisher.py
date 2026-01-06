#!/usr/bin/env python3
"""Sine sweep publisher for TVC servo command via ROS2.

Publishes `std_msgs/msg/Float64MultiArray` on `/tvc_servo_command`.
This assumes `ros_gz_bridge` parameter_bridge is running to bridge the ROS2 topic
to the Gazebo topic `gz.msgs.Double_V` that the plugin subscribes to.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import argparse
import time
import math


class SineSweepPublisher(Node):
    def __init__(self, args):
        super().__init__('tvc_sine_sweep_publisher')
        self.pub = self.create_publisher(Float64MultiArray, '/tvc_servo_command', 10)
        self.rate_hz = args.rate
        self.duration = args.duration
        self.f0 = args.fstart
        self.f1 = args.fend
        self.amp = args.amp
        self.neutral = args.neutral
        self.servo_index = args.servo_index  # 0 or 1

    def run(self):
        dt = 1.0 / float(self.rate_hz)
        steps = int(self.duration * self.rate_hz)
        t0 = time.time()
        phase = 0.0
        last_time = t0

        for i in range(steps):
            now = time.time()
            t = now - t0
            # linear frequency sweep (chirp): f(t) = f0 + (f1 - f0) * (t / T)
            frac = t / max(self.duration, 1e-9)
            freq = self.f0 + (self.f1 - self.f0) * frac
            # integrate phase: phase += 2*pi*freq*dt
            phase += 2.0 * math.pi * freq * (now - last_time)
            last_time = now

            val = self.neutral
            sinusoid = self.amp * math.sin(phase)
            if self.servo_index == 0:
                data = [self.neutral + sinusoid, self.neutral]
            else:
                data = [self.neutral, self.neutral + sinusoid]

            msg = Float64MultiArray()
            msg.data = data
            self.pub.publish(msg)

            sleep_time = dt - (time.time() - now)
            if sleep_time > 0:
                time.sleep(sleep_time)


def main():
    parser = argparse.ArgumentParser(description='TVC sine sweep publisher')
    parser.add_argument('--duration', type=float, default=30.0, help='Duration (s)')
    parser.add_argument('--fstart', type=float, default=0.1, help='Start frequency (Hz)')
    parser.add_argument('--fend', type=float, default=5.0, help='End frequency (Hz)')
    parser.add_argument('--amp', type=float, default=0.005, help='Sine amplitude (m)')
    parser.add_argument('--neutral', type=float, default=0.093, help='Neutral servo length (m)')
    parser.add_argument('--rate', type=float, default=200.0, help='Publish rate (Hz)')
    parser.add_argument('--servo-index', type=int, default=0, choices=[0,1], help='Which servo to sweep (0 or 1)')
    args = parser.parse_args()

    rclpy.init()
    node = SineSweepPublisher(args)
    try:
        node.get_logger().info(f'Starting sine sweep: {args.fstart}->{args.fend} Hz, dur {args.duration}s')
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
