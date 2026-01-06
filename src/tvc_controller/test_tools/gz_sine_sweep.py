#!/usr/bin/env python3
"""Gazebo-native sine sweep publisher using the `gz topic` CLI.

This script publishes `gz.msgs.Double_V` messages directly to a Gazebo topic
using the `gz topic -t ... -m gz.msgs.Double_V` command. Use this when
`ros_gz_bridge` cannot map `gz.msgs.Double_V` to a ROS2 message.

Notes:
- Requires `gz` CLI available on PATH.
- Default publish rate is conservative (100 Hz) to avoid CLI overhead; lower
  latency can be achieved with a native transport publisher if available.
"""
import argparse
import time
import math
import subprocess
import shlex


def publish_double_v(topic, v1, v2):
    # Compose the gz topic publish command.
    # Note: the `gz topic` CLI expects array elements separated by spaces (no commas):
    #   data: [0.093 0.093]
    # Use comma-separated values; pass the whole payload as a single argument.
    msg = f'data: [{v1:.6f}, {v2:.6f}]'
    args = ['gz', 'topic', '-t', topic, '-m', 'gz.msgs.Double_V', '-p', msg]
    # Run the command and surface errors for debugging
    # Print the exact args so users can copy/paste the working command.
    print(f"Publishing: {' '.join(shlex.quote(a) for a in args)}")
    try:
        res = subprocess.run(args, check=False, capture_output=True, text=True)
        if res.returncode != 0:
            print(f"gz topic publish failed (rc={res.returncode}): {res.stderr.strip()}")
    except FileNotFoundError:
        print("Error: 'gz' CLI not found on PATH. Make sure Ignition/Gazebo is installed and 'gz' is available.")


def run_sine_sweep(topic, duration, fstart, fend, amp, neutral, rate, servo_index):
    dt = 1.0 / rate
    steps = int(duration * rate)
    t0 = time.time()
    phase = 0.0
    last_time = t0

    for i in range(steps):
        now = time.time()
        t = now - t0
        frac = t / max(duration, 1e-9)
        freq = fstart + (fend - fstart) * frac
        phase += 2.0 * math.pi * freq * (now - last_time)
        last_time = now

        sinusoid = amp * math.sin(phase)
        if servo_index == 0:
            v1 = neutral + sinusoid
            v2 = neutral
        else:
            v1 = neutral
            v2 = neutral + sinusoid

        publish_double_v(topic, v1, v2)

        # sleep to maintain rate
        sleep_time = dt - (time.time() - now)
        if sleep_time > 0:
            time.sleep(sleep_time)


def main():
    parser = argparse.ArgumentParser(description='Gazebo-native sine sweep publisher')
    parser.add_argument('--topic', default='/tvc_servo_command', help='Gazebo topic to publish to')
    parser.add_argument('--duration', type=float, default=30.0, help='Duration in seconds')
    parser.add_argument('--fstart', type=float, default=0.1, help='Start frequency (Hz)')
    parser.add_argument('--fend', type=float, default=5.0, help='End frequency (Hz)')
    parser.add_argument('--amp', type=float, default=0.005, help='Sine amplitude (m)')
    parser.add_argument('--neutral', type=float, default=0.093, help='Neutral servo length (m)')
    parser.add_argument('--rate', type=float, default=100.0, help='Publish rate (Hz)')
    parser.add_argument('--servo-index', type=int, default=0, choices=[0,1], help='Which servo to sweep')
    args = parser.parse_args()

    print(f'Publishing to {args.topic} for {args.duration}s at {args.rate}Hz (servo {args.servo_index})')
    run_sine_sweep(args.topic, args.duration, args.fstart, args.fend, args.amp, args.neutral, args.rate, args.servo_index)


if __name__ == '__main__':
    main()
