class PIDController:
    def __init__(self, kp, ki, kd, output_limits=(None, None)):
        """
        A simple PID controller.
        kp: Proportional gain
        ki: Integral gain
        kd: Derivative gain
        output_limits: Tuple of (min, max) for the output
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.min_out, self.max_out = output_limits
        
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, setpoint, current_value, dt):
        """
        Compute the PID output.
        """
        if dt <= 0.0:
            return 0.0
            
        error = setpoint - current_value
        
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        
        # Anti-windup and output clamping
        if self.max_out is not None:
            output = min(output, self.max_out)
        if self.min_out is not None:
            output = max(output, self.min_out)
            
        self.prev_error = error
        
        return output

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0
