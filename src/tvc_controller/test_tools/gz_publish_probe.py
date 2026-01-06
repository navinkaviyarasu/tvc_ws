#!/usr/bin/env python3
"""
Try multiple `gz topic` payload formats until one succeeds, then publish values at a rate.

Useful when different Ignition/GZ versions expect slightly different CLI payload syntax.

Usage:
  python3 gz_publish_probe.py --topic /tvc_servo_command --v1 0.093 --v2 0.093 --rate 50 --duration 5

The script will try several candidate message string formats and report which one worked.
"""
import subprocess
import time
import argparse
import shlex


CANDIDATES = [
    'data: [{v1:.6f} {v2:.6f}]',            # common: space-separated array
    'data: [{v1:.6f}, {v2:.6f}]',           # comma-separated array
    '{{data: [{v1:.6f} {v2:.6f}]}}',        # wrapped in braces
    '{{data: [{v1:.6f}, {v2:.6f}]}}',       # wrapped + commas
    'data: [{v1:.6f}] [ {v2:.6f} ]',        # odd variant
    'data:[{v1:.6f} {v2:.6f}]',             # no space after colon
]


def try_publish_once(topic, v1, v2):
    """Try candidate formats once and return the working format string or None."""
    for cand in CANDIDATES:
        msg = cand.format(v1=v1, v2=v2)
        args = ['gz', 'topic', '-t', topic, '-m', 'gz.msgs.Double_V', '-p', msg]
        try:
            res = subprocess.run(args, capture_output=True, text=True)
        except FileNotFoundError:
            print("Error: 'gz' CLI not found on PATH.")
            return None

        if res.returncode == 0:
            print(f"Publish succeeded with format: {msg}")
            return msg
        else:
            # print debug for failed attempt
            print(f"Format failed (rc={res.returncode}): {msg}  stderr: {res.stderr.strip()}")

    return None


def publish_loop(topic, msg_template, v1, v2, rate, duration):
    dt = 1.0 / rate
    steps = int(duration * rate)
    for i in range(steps):
        msg = msg_template.format(v1=v1, v2=v2)
        args = ['gz', 'topic', '-t', topic, '-m', 'gz.msgs.Double_V', '-p', msg]
        try:
            res = subprocess.run(args, capture_output=True, text=True)
            if res.returncode != 0:
                print(f"Publish failed during loop (rc={res.returncode}): {res.stderr.strip()}")
                return False
        except FileNotFoundError:
            print("Error: 'gz' CLI not found on PATH.")
            return False

        time.sleep(dt)

    return True


def main():
    parser = argparse.ArgumentParser(description='Probe gz topic payload formats and publish Double_V')
    parser.add_argument('--topic', default='/tvc_servo_command')
    parser.add_argument('--v1', type=float, default=0.093)
    parser.add_argument('--v2', type=float, default=0.093)
    parser.add_argument('--rate', type=float, default=50.0)
    parser.add_argument('--duration', type=float, default=5.0)
    args = parser.parse_args()

    print(f"Probing `gz topic` payload formats for topic {args.topic}...")
    working = try_publish_once(args.topic, args.v1, args.v2)
    if not working:
        print("No candidate format succeeded. Please run `which gz` and `gz --version` and share the output.")
        return

    print(f"Starting publish loop ({args.duration}s @ {args.rate}Hz) using format: {working}")
    ok = publish_loop(args.topic, working, args.v1, args.v2, args.rate, args.duration)
    if ok:
        print("Publish loop completed successfully.")
    else:
        print("Publish loop failed.")


if __name__ == '__main__':
    main()
