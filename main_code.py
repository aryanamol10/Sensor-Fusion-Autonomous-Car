import time
import threading
import json
import uvicorn
from fastapi import FastAPI, WebSocket
from fastapi.middleware.cors import CORSMiddleware
import asyncio

# Import our custom modules
from sensors import IMUDriver, EncoderDriver, LidarDriver
from localization import DifferentialDriveKinematics, RobotEKF
from mapping import OccupancyGrid
from control import MotorController

class AutonomousCar:
    def __init__(self):
        # Configuration
        self.SAFE_DISTANCE = 0.3 # meters
        self.EXPLORE_SPEED = 0.3 # m/s (lowered for safety)
        
        # 1. Hardware Interfaces
        self.imu = IMUDriver()
        self.encoders = EncoderDriver(left_pin_a=17, left_pin_b=27, 
                                      right_pin_a=22, right_pin_b=23, 
                                      ticks_per_meter=1000)
        self.lidar = LidarDriver(port='/dev/ttyUSB0')

        # 2. Localization
        self.dt = 0.1 # 10Hz loop
        self.kinematics = DifferentialDriveKinematics(wheel_base=0.2, wheel_radius=0.05, ticks_per_rev=200)
        self.ekf = RobotEKF(dt=self.dt)

        # 3. Mapping
        self.mapper = OccupancyGrid(width_m=20.0, height_m=20.0, resolution_m=0.1, origin_x_m=-10.0, origin_y_m=-10.0)

        # 4. Control
        self.controller = MotorController(wheel_base=0.2)
        
        # 5. State Machine
        self.state = "EXPLORING" # EXPLORING, RECOVERING, STOPPED
        self.running = False
        self.current_state_data = {}
        
        # UI Bridge (FastAPI)
        self.app = FastAPI()
        self.app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])
        self._setup_routes()

    def _setup_routes(self):
        @self.app.websocket("/ws")
        async def websocket_endpoint(websocket: WebSocket):
            await websocket.accept()
            try:
                while self.running:
                    # Construct message
                    grid_data = self.mapper.get_grid_data().tolist()
                    pose = self.ekf.get_pose()
                    msg = {
                        "pose": {"x": pose[0], "y": pose[1], "theta": pose[2]},
                        "map": grid_data,
                        "state": self.state,
                        "config": {
                            "origin_x": self.mapper.origin_x,
                            "origin_y": self.mapper.origin_y,
                            "resolution": self.mapper.resolution
                        }
                    }
                    await websocket.send_json(msg)
                    await asyncio.sleep(0.5) # 2Hz UI update
            except Exception as e:
                print(f"WebSocket closed: {e}")

    def navigation_thread(self):
        """
        Background thread to handle Lidar mapping and basic exploration/avoidance.
        """
        while self.running:
            start_time = time.time()
            
            # 1. Update Map
            robot_x, robot_y, robot_theta = self.ekf.get_pose()
            scan = self.lidar.get_data()
            if scan:
                self.mapper.update_from_scan(robot_x, robot_y, robot_theta, scan, max_range=4.0)
                
                # 2. Obstacle Detection (Check front arc -45 to +45)
                obstacles_in_front = [s for s in scan if abs(s[1]) < 45 and (s[2]/1000.0) < self.SAFE_DISTANCE]
                
                if obstacles_in_front and self.state == "EXPLORING":
                    print("Obstacle detected! Initiating recovery...")
                    self.state = "RECOVERING"
            
            # Maintain mapping rate (~5Hz)
            elapsed = time.time() - start_time
            time.sleep(max(0.0, 0.2 - elapsed))

    def control_loop(self):
        """
        High frequency loop to update odometry, EKF, and simple navigation.
        """
        recovery_start_time = 0
        
        while self.running:
            loop_start = time.time()

            # --- A. Sensor & Localization ---
            enc_data = self.encoders.get_data()
            imu_data = self.imu.get_data()
            odom_x, odom_y, odom_theta, v, omega = self.kinematics.update(enc_data['left_ticks'], enc_data['right_ticks'], self.dt)
            self.ekf.predict(v, omega)
            if imu_data:
                self.ekf.update(z_odom=[odom_x, odom_y], z_imu_yaw_rate=imu_data['yaw_rate'])
            curr_x, curr_y, curr_theta = self.ekf.get_pose()
            
            # --- B. Navigation Behavior ---
            target_v = 0
            target_omega = 0
            
            if self.state == "EXPLORING":
                target_v = self.EXPLORE_SPEED
                target_omega = 0
            elif self.state == "RECOVERING":
                # Simple recovery: Back up then turn
                if recovery_start_time == 0:
                    recovery_start_time = time.time()
                
                elapsed_recovery = time.time() - recovery_start_time
                if elapsed_recovery < 1.0: # Back up for 1s
                    target_v = -self.EXPLORE_SPEED
                    target_omega = 0
                elif elapsed_recovery < 2.0: # Turn for 1s
                    target_v = 0
                    target_omega = 1.0 # rad/s
                else:
                    # Reset recovery
                    self.state = "EXPLORING"
                    recovery_start_time = 0
            
            # Simple open-loop velocity control for now (or use MotorController)
            # v_left, v_right = self.controller.compute_velocity_commands(target_v, target_omega)
            # TODO: Send commands to hardware
            
            elapsed = time.time() - loop_start
            sleep_time = self.dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def start(self):
        print("Starting Autonomous Floor-Sweep System...")
        self.running = True
        
        # Start Threads
        nav_thread = threading.Thread(target=self.navigation_thread, daemon=True)
        nav_thread.start()
        
        control_thread = threading.Thread(target=self.control_loop, daemon=True)
        control_thread.start()
        
        # Start FastAPI in main thread
        print("UI WebSocket starting on http://localhost:8000")
        uvicorn.run(self.app, host="0.0.0.0", port=8000)

    def stop(self):
        self.running = False
        self.lidar.stop()

if __name__ == "__main__":
    car = AutonomousCar()
    try:
        car.start()
    except KeyboardInterrupt:
        car.stop()
