import numpy as np
import math

class OccupancyGrid:
    def __init__(self, width_m, height_m, resolution_m, origin_x_m=0.0, origin_y_m=0.0):
        """
        Initialize the 2D occupancy grid.
        """
        self.resolution = resolution_m
        self.width_cells = int(width_m / resolution_m)
        self.height_cells = int(height_m / resolution_m)
        self.origin_x = origin_x_m
        self.origin_y = origin_y_m
        
        # Persistent Log-Odds Grid
        # 0 = Unknown, positive = likely occupied, negative = likely free
        self.grid = np.zeros((self.width_cells, self.height_cells), dtype=np.float32)
        
        # Configuration for floor-sweep (defaults)
        #ADJUST THESE
        self.lidar_height = 0.2  # meters
        self.lidar_tilt_deg = 30.0 # degrees down from horizontal
        self.floor_threshold = 0.02 # meters (allowance for floor noise)

    def world_to_grid(self, x, y):
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        return gx, gy

    def grid_to_world(self, gx, gy):
        x = (gx + 0.5) * self.resolution + self.origin_x
        y = (gy + 0.5) * self.resolution + self.origin_y
        return x, y

    def update_from_scan(self, robot_x, robot_y, robot_theta, scan_data, max_range=4.0):
        """
        Update grid using persistent mapping with floor-sweep projection.
        """
        tilt_rad = math.radians(self.lidar_tilt_deg)
        
        for quality, angle_deg, dist_mm in scan_data:
            dist_m = dist_mm / 1000.0
            if dist_m <= 0.05 or dist_m > max_range:
                continue
                
            angle_rad = math.radians(angle_deg)
            
            # --- 1. Project Lidar point to 3D Robot Frame ---
            # Forward component in Lidar plane: dist_m * cos(angle)
            # Sideways component in Lidar plane: dist_m * sin(angle)
            
            lx = dist_m * math.cos(angle_rad)
            ly = dist_m * math.sin(angle_rad)
            
            # Rotate for tilt (X-Z rotation)
            # x_robot = lx * cos(tilt)
            # z_robot = height - lx * sin(tilt)
            rx = lx * math.cos(tilt_rad)
            ry = ly
            rz = self.lidar_height - lx * math.sin(tilt_rad)
            
            # --- 2. Determine if hit is Floor or Obstacle ---
            is_obstacle = rz > self.floor_threshold
            
            # --- 3. Project to World Coordinates ---
            cos_th = math.cos(robot_theta)
            sin_th = math.sin(robot_theta)
            
            world_x = robot_x + (rx * cos_th - ry * sin_th)
            world_y = robot_y + (rx * sin_th + ry * cos_th)
            
            gx, gy = self.world_to_grid(world_x, world_y)
            
            if 0 <= gx < self.width_cells and 0 <= gy < self.height_cells:
                if is_obstacle:
                    # Increment occupancy (Log-odds increment)
                    self.grid[gx, gy] = min(5.0, self.grid[gx, gy] + 0.5)
                else:
                    # Mark as floor (decrement occupancy)
                    self.grid[gx, gy] = max(-5.0, self.grid[gx, gy] - 0.2)

    def get_grid_data(self):
        """Return a binary representation for path/UI: 1 = occupied, 0 = free/unknown."""
        return (self.grid > 1.0).astype(np.int8)

    def is_occupied(self, gx, gy):
        if 0 <= gx < self.width_cells and 0 <= gy < self.height_cells:
            return self.grid[gx, gy] > 1.0
        return True
