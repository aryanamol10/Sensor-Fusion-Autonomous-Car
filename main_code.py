import time
import threading

# Import our custom modules
from sensors import IMUDriver, EncoderDriver, LidarDriver
from localization import DifferentialDriveKinematics, RobotEKF
from mapping import OccupancyGrid
from planning import AStarPlanner, TrajectoryManager
from control import MotorController

class AutonomousCar:
    def __init__(self):
        # 1. Hardware Interfaces (Mock or real depending on hardware availability)
        self.imu = IMUDriver()
        # Ensure you replace pins and ticks_per_m with actual robot spec
        self.encoders = EncoderDriver(left_pin_a=17, left_pin_b=27, 
                                      right_pin_a=22, right_pin_b=23, 
                                      ticks_per_meter=1000)
        self.lidar = LidarDriver(port='/dev/ttyUSB0')

        # 2. Localization
        # Wheel base: 0.2m, Wheel radius: 0.05m
        self.dt = 0.1 # 10Hz loop
        self.kinematics = DifferentialDriveKinematics(wheel_base=0.2, wheel_radius=0.05, ticks_per_rev=200)
        self.ekf = RobotEKF(dt=self.dt)

        # 3. Mapping & Planning
        self.mapper = OccupancyGrid(width_m=10.0, height_m=10.0, resolution_m=0.1, origin_x_m=-5.0, origin_y_m=-5.0)
        self.planner = AStarPlanner(resolution=0.1, robot_radius=0.15)
        # Target velocity: 0.5 m/s
        self.trajectory = TrajectoryManager(target_velocity=0.5)

        # 4. Control
        self.controller = MotorController(wheel_base=0.2)
        
        self.running = False
        self.goal = (3.0, 3.0) # Goal waypoint

    def planning_thread(self):
        """
        Background thread to handle Lidar mapping and A* path planning.
        Runs slower than the main control loop.
        """
        while self.running:
            start_time = time.time()
            
            # Get current pose from EKF
            robot_x, robot_y, robot_theta = self.ekf.get_pose()
            
            # Get latest Lidar scan
            scan = self.lidar.get_data()
            if scan:
                # Update map and inflate
                self.mapper.update_from_scan(robot_x, robot_y, robot_theta, scan, max_range=4.0)
                self.mapper.inflate_obstacles(radius_m=0.15)
                
                # Replan A* Path
                path = self.planner.plan(robot_x, robot_y, self.goal[0], self.goal[1], self.mapper)
                
                if path:
                    # Send to trajectory manager to assign accurate time intervals
                    self.trajectory.set_path(path)
                    print(f"Replanned Path with {len(path)} waypoints")
            
            # Sleep to maintain mapping rate (~2Hz)
            elapsed = time.time() - start_time
            time.sleep(max(0.0, 0.5 - elapsed))

    def control_loop(self):
        """
        High frequency loop to update odometry, EKF, and PID controllers.
        """
        while self.running:
            loop_start = time.time()

            # --- A. Sensor Reading ---
            enc_data = self.encoders.get_data()
            imu_data = self.imu.get_data()
            
            # --- B. Localization ---
            # 1. Update Odometry
            odom_x, odom_y, odom_theta, v, omega = self.kinematics.update(
                enc_data['left_ticks'], 
                enc_data['right_ticks'], 
                self.dt
            )
            
            # 2. Update EKF Predict and Update
            self.ekf.predict(v, omega)
            
            if imu_data:
                # Update with IMU yaw rate
                self.ekf.update(z_odom=[odom_x, odom_y], z_imu_yaw_rate=imu_data['yaw_rate'])
                
            curr_x, curr_y, curr_theta = self.ekf.get_pose()
            
            # --- C. Control ---
            # Ask trajectory manager where we should be right now
            target_x, target_y = self.trajectory.get_target_waypoint()
            
            # Compute PID outputs based on target
            v_left, v_right = self.controller.compute_commands(
                curr_x, curr_y, curr_theta, 
                target_x, target_y, 
                self.dt
            )
            
            # TODO: Send v_left, v_right to physical motor drivers
            # e.g., using RPi.GPIO PWM.
            # print(f"Commands: L={v_left:.2f}, R={v_right:.2f}")

            if self.trajectory.is_finished() and target_x is not None:
                print("Reached Goal or Finished Path Segment.")
                # We could stop here or wait for next goal

            # Maintain strict loop frequency (10Hz)
            elapsed = time.time() - loop_start
            sleep_time = self.dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def start(self):
        print("Starting Autonomous System...")
        self.running = True
        
        plan_thread = threading.Thread(target=self.planning_thread)
        plan_thread.start()
        
        try:
            self.control_loop()
        except KeyboardInterrupt:
            print("Shutting down...")
            self.stop()
            plan_thread.join()

    def stop(self):
        self.running = False
        self.lidar.stop()

if __name__ == "__main__":
    car = AutonomousCar()
    car.start()
