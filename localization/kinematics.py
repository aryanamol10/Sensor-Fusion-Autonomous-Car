import math

class DifferentialDriveKinematics:
    def __init__(self, wheel_base, wheel_radius, ticks_per_rev):
        """
        Initialize the kinematic model for a differential drive robot.
        wheel_base: distance between the centers of the two wheels (meters)
        wheel_radius: radius of the wheels (meters)
        ticks_per_rev: number of encoder ticks per wheel revolution
        """
        self.wheel_base = wheel_base
        self.wheel_radius = wheel_radius
        self.ticks_per_rev = ticks_per_rev
        self.meters_per_tick = (2 * math.pi * self.wheel_radius) / self.ticks_per_rev
        
        # State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0

    def update(self, left_ticks, right_ticks, dt):
        """
        Update the odometry based on wheel encoder ticks.
        Returns:
            tuple: (x, y, theta, v, omega) - Pose and velocities
        """
        # Calculate tick differences
        delta_left = left_ticks - self.prev_left_ticks
        delta_right = right_ticks - self.prev_right_ticks
        
        self.prev_left_ticks = left_ticks
        self.prev_right_ticks = right_ticks
        
        # Calculate distance traveled by each wheel
        dist_left = delta_left * self.meters_per_tick
        dist_right = delta_right * self.meters_per_tick
        
        # Calculate center distance and change in heading
        dist_center = (dist_right + dist_left) / 2.0
        delta_theta = (dist_right - dist_left) / self.wheel_base
        
        # Update pose
        self.x += dist_center * math.cos(self.theta + delta_theta / 2.0)
        self.y += dist_center * math.sin(self.theta + delta_theta / 2.0)
        self.theta += delta_theta
        
        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Calculate velocities
        v = dist_center / dt if dt > 0 else 0.0
        omega = delta_theta / dt if dt > 0 else 0.0
        
        return self.x, self.y, self.theta, v, omega
