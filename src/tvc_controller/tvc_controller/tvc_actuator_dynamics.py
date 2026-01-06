import math

class ActuatorModel:
    def __init__(self):
      
        self.time_constant = 0.05            # s
        self.max_rate = 0.261799            # rad/s (15deg/sec)
        # self.max_angle = math.radians(15)   # ±15°

        self.current = 0.0
        # self.target = 0.0

    # def set_target(self, target):
    #     # self.target = max(-self.max_angle, min(self.max_angle, target))
    #     self.target = target

    def update(self,target, dt):
        self.target = target
        error = self.target - self.current
        ideal_rate = error / self.time_constant
        rate = max(-self.max_rate, min(self.max_rate, ideal_rate))
        self.current += rate * dt
        return self.current

    # def __init__(self):
    #     self.max_rate = 5.0      # rad/s
    #     self.current = 0.0
    #     self.target = 0.0
    #     self.delay = 0.05        # seconds of command delay
    #     self.last_cmd_time = None

    # def set_target(self, target, now):
    #     """Store target and timestamp"""
    #     self.target = target
    #     self.last_cmd_time = now

    def time_delay(self, dt, now):

        if self.last_cmd_time is None:
            return self.current

        # Check if delay elapsed
        if now - self.last_cmd_time < self.delay:
            return self.current

        # Compute direction and move at constant rate
        error = self.target - self.current
        if abs(error) < 1e-6:
            return self.current  # Already at target

        # Determine step (constant speed)
        step = self.max_rate * dt
        if abs(error) <= step:
            self.current = self.target
        else:
            self.current += step * (1 if error > 0 else -1)

        return self.current
