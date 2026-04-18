import math
from .pid import PIDController

class MotorController:
    def __init__(self, wheel_base):
        """
        Translates target waypoints into motor commands (e.g., left/right wheel speeds)
        using PID controllers for distance and heading.
        wheel_base: distance between wheels.
        """
        self.wheel_base = wheel_base
        
        # PID for linear velocity (throttle) based on distance to target
        self.linear_pid = PIDController(kp=1.5, ki=0.0, kd=0.1, output_limits=(-1.0, 1.0))
        
        # PID for angular velocity (steering) based on heading error
        self.angular_pid = PIDController(kp=2.0, ki=0.0, kd=0.2, output_limits=(-2.0, 2.0))
        
    def compute_commands(self, current_x, current_y, current_theta, target_x, target_y, dt):
        """
        Compute the left and right wheel velocities to reach the target waypoint.
        """
        if target_x is None or target_y is None:
            return 0.0, 0.0
            
        # Calculate errors
        dx = target_x - current_x
        dy = target_y - current_y
        
        distance_error = math.hypot(dx, dy)
        
        # If we are very close to the target, stop or coast
        if distance_error < 0.05:
            return 0.0, 0.0
            
        target_heading = math.atan2(dy, dx)
        
        heading_error = target_heading - current_theta
        # Normalize to [-pi, pi]
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
        
        # Compute control outputs
        # Linear velocity setpoint is the distance error
        linear_v = self.linear_pid.compute(setpoint=distance_error, current_value=0.0, dt=dt)
        
        # Angular velocity setpoint is the heading error
        angular_w = self.angular_pid.compute(setpoint=heading_error, current_value=0.0, dt=dt)
        
        # Translate linear and angular velocity to left/right wheel velocities (differential drive)
        # v = (v_right + v_left) / 2
        # w = (v_right - v_left) / L
        
        v_right = linear_v + (angular_w * self.wheel_base / 2.0)
        v_left = linear_v - (angular_w * self.wheel_base / 2.0)
        
        # Note: In a real system you would map these velocities to PWM signals
        # and send them to a motor driver board (e.g., L298N) using RPi.GPIO
        
        return v_left, v_right
