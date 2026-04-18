import numpy as np
from filterpy.kalman import ExtendedKalmanFilter

class RobotEKF:
    def __init__(self, dt):
        """
        Extended Kalman Filter for fusing Odometry (v, omega) and IMU (yaw rate).
        State vector: [x, y, theta]
        """
        self.dt = dt
        self.ekf = ExtendedKalmanFilter(dim_x=3, dim_z=3) # z = [x_odom, y_odom, yaw_rate_imu]
        
        # Initial state
        self.ekf.x = np.array([0., 0., 0.])
        
        # Initial covariance matrix
        self.ekf.P = np.eye(3) * 0.1
        
        # Process noise covariance (trust the model less over time)
        self.ekf.Q = np.diag([0.1, 0.1, 0.05])
        
        # Measurement noise covariance
        # [x_odom_noise, y_odom_noise, imu_yaw_rate_noise]
        self.ekf.R = np.diag([0.5, 0.5, 0.01])
        
    def predict(self, v, omega):
        """
        Predict step based on control inputs (from odometry velocity).
        """
        theta = self.ekf.x[2]
        
        # State transition function f(x, u)
        # x_new = x + v * dt * cos(theta)
        # y_new = y + v * dt * sin(theta)
        # theta_new = theta + omega * dt
        
        # Update state
        self.ekf.x[0] += v * self.dt * np.cos(theta)
        self.ekf.x[1] += v * self.dt * np.np.sin(theta)
        self.ekf.x[2] += omega * self.dt
        
        # Normalize theta
        self.ekf.x[2] = np.arctan2(np.sin(self.ekf.x[2]), np.cos(self.ekf.x[2]))
        
        # Jacobian of f with respect to state x (F matrix)
        F = np.array([
            [1.0, 0.0, -v * self.dt * np.sin(theta)],
            [0.0, 1.0,  v * self.dt * np.cos(theta)],
            [0.0, 0.0, 1.0]
        ])
        
        # Predict covariance
        self.ekf.P = F @ self.ekf.P @ F.T + self.ekf.Q

    def update(self, z_odom, z_imu_yaw_rate):
        """
        Update step with measurements.
        z_odom: [x, y] from kinematics
        z_imu_yaw_rate: omega from IMU gyro Z
        """
        # Measurement vector [x, y, theta_derived_from_imu]
        # In this simplified model, we directly integrate IMU yaw rate to get a heading measurement
        # A more robust EKF would use the IMU yaw rate directly in a different z-vector formulation,
        # but for simplicity we fuse x, y from odometry and theta from (previous_theta + yaw_rate*dt)
        
        theta_meas = self.ekf.x[2] + z_imu_yaw_rate * self.dt
        z = np.array([z_odom[0], z_odom[1], theta_meas])
        
        # Measurement function H maps state to measurement (1:1 mapping here)
        H = np.eye(3)
        
        # Innovation
        y = z - (H @ self.ekf.x)
        y[2] = np.arctan2(np.sin(y[2]), np.cos(y[2])) # Normalize angle difference
        
        # Innovation covariance
        S = H @ self.ekf.P @ H.T + self.ekf.R
        
        # Kalman Gain
        K = self.ekf.P @ H.T @ np.linalg.inv(S)
        
        # Update state
        self.ekf.x = self.ekf.x + K @ y
        self.ekf.x[2] = np.arctan2(np.sin(self.ekf.x[2]), np.cos(self.ekf.x[2]))
        
        # Update covariance
        I = np.eye(3)
        self.ekf.P = (I - K @ H) @ self.ekf.P

    def get_pose(self):
        return self.ekf.x[0], self.ekf.x[1], self.ekf.x[2]
