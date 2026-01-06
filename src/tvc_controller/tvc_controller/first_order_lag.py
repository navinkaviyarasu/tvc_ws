import numpy as np
import matplotlib.pyplot as plt

# System parameters
K = 1.0          # Gain
tau = 2.0        # Time constant (seconds)
dt = 0.01        # Time step (seconds)
T = 10           # Total simulation time (seconds)

# Time vector
time = np.arange(0, T, dt)

# Input (step input: u(t) = 1 for all t >= 0)
u = np.ones_like(time)

# Output initialization
y = np.zeros_like(time)

# Simulation using Euler method
for i in range(1, len(time)):
    dy_dt = (K * u[i-1] - y[i-1]) / tau
    y[i] = y[i-1] + dy_dt * dt

# Plotting
plt.figure(figsize=(10, 5))
plt.plot(time, u, label='Input u(t)', linestyle='--')
plt.plot(time, y, label='Output y(t)', color='blue')
plt.title('First Order Lag Response to Step Input')
plt.xlabel('Time [s]')
plt.ylabel('Amplitude')
plt.grid(True)
plt.legend()
plt.show()