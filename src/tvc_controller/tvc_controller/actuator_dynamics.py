#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math
import matplotlib.pyplot as plt
from datetime import datetime
import matplotlib.dates as mdates

class TVCActuatorDynamics(Node):
    def __init__(self):
        super().__init__('tvc_actuator_dynamics')

        self.time_constant = 0.05
        self.max_rate = 0.25
        self.max_angle = math.radians(15)

        self.current_pitch = 0.0
        self.current_roll = 0.0
        self.target_pitch = 0.25
        self.target_roll = 0.0
        self.start_time = self.get_clock().now().nanoseconds * 1e-9 #seconds

        self.timestamps = []
        self.pitch_values = []

        # Plot setup
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], label='Current Pitch')
        self.ax.set_title("First-Order Lag (Pitch)")
        self.ax.set_xlabel("Time")
        self.ax.set_ylabel("Pitch (rad)")
        self.ax.legend()
        self.ax.grid(True)

        self.target_pitch_values = []  # List to store input values
        self.line_target, = self.ax.plot([], [], label='Target Pitch', linestyle='--')

        # 100 Hz timer
        self.create_timer(0.01, self.update_dynamics)

    def update_dynamics(self):
        now = self.get_clock().now().nanoseconds * 1e-9 #seconds
        dt = now - getattr(self, 'last_time', now)
        self.last_time = now

        elapsed_time = now - self.start_time

        # === Step Input Logic ===
        # if elapsed_time < 2.0:
        #     self.target_pitch = 0.0
        # elif elapsed_time < 4.0:
        #     self.target_pitch = math.radians(5)
        # elif elapsed_time < 6.0:
        #     self.target_pitch = math.radians(-10)
        # else:
        #     self.target_pitch = 0.0

        # Clamp target
        self.target_pitch = max(-self.max_angle, min(self.max_angle, self.target_pitch))

        # Apply dynamics
        self.current_pitch = self.apply_dynamics(
            self.current_pitch, self.target_pitch, dt)
        
        # Plotting
        self.timestamps.append(datetime.now())
        # self.timestamps.append(now)
        self.pitch_values.append(self.current_pitch)
        self.target_pitch_values.append(self.target_pitch)

        print(f"Time:{self.timestamps}, pitch_values:{self.pitch_values}, target:{self.target_pitch}\n")

        self.line.set_data(self.timestamps, self.pitch_values)
        self.line_target.set_data(self.timestamps, self.target_pitch_values)
        # self.ax.relim()
        self.ax.autoscale_view()
        self.fig.autofmt_xdate()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def apply_dynamics(self, current, target, dt):
        error = target - current
        ideal_rate = error / self.time_constant
        rate = max(-self.max_rate, min(self.max_rate, ideal_rate))
        return current + rate * dt


def main(args=None):
    rclpy.init(args=args)
    node = TVCActuatorDynamics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        plt.ioff()
        plt.show()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
