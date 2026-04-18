import numpy as np
import math

class OccupancyGrid:
    def __init__(self, width_m, height_m, resolution_m, origin_x_m=0.0, origin_y_m=0.0):
        """
        Initialize the 2D occupancy grid.
        width_m, height_m: Size of the map in meters
        resolution_m: Size of each grid cell in meters
        origin_x_m, origin_y_m: World coordinates of the bottom-left corner of the grid
        """
        self.resolution = resolution_m
        self.width_cells = int(width_m / resolution_m)
        self.height_cells = int(height_m / resolution_m)
        self.origin_x = origin_x_m
        self.origin_y = origin_y_m
        
        # Grid initialized to 0 (unknown/free). Obstacles will be 1 (or probability).
        # We use a simple binary grid for A*: 0 = free, 1 = occupied.
        self.grid = np.zeros((self.width_cells, self.height_cells), dtype=np.int8)

    def world_to_grid(self, x, y):
        """Convert world coordinates (m) to grid indices."""
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        return gx, gy

    def grid_to_world(self, gx, gy):
        """Convert grid indices to world coordinates (center of the cell)."""
        x = (gx + 0.5) * self.resolution + self.origin_x
        y = (gy + 0.5) * self.resolution + self.origin_y
        return x, y

    def update_from_scan(self, robot_x, robot_y, robot_theta, scan_data, max_range=5.0):
        """
        Update grid using a new Lidar scan.
        scan_data: list of tuples (quality, angle_deg, distance_mm)
        """
        # Clear grid (or decay in a more advanced probabilistic model)
        # For simple A*, we might clear the grid and re-populate with current scan to handle dynamic obstacles.
        # Alternatively, we just maintain a persistent map and raytrace free space.
        # Here we do a simple additive approach where we clear the grid each update for local path planning.
        self.grid.fill(0)
        
        for quality, angle_deg, dist_mm in scan_data:
            dist_m = dist_mm / 1000.0
            
            # Skip invalid or out of range points
            if dist_m <= 0.0 or dist_m > max_range:
                continue
                
            angle_rad = math.radians(angle_deg)
            
            # Calculate world coordinates of the obstacle
            # Lidar angle is relative to robot heading
            obs_x = robot_x + dist_m * math.cos(robot_theta + angle_rad)
            obs_y = robot_y + dist_m * math.sin(robot_theta + angle_rad)
            
            gx, gy = self.world_to_grid(obs_x, obs_y)
            
            if 0 <= gx < self.width_cells and 0 <= gy < self.height_cells:
                self.grid[gx, gy] = 1 # Mark as occupied

    def inflate_obstacles(self, radius_m):
        """
        Inflate obstacles by the robot's radius to ensure collision-free paths for a point mass planner.
        """
        inflation_cells = int(math.ceil(radius_m / self.resolution))
        if inflation_cells == 0:
            return
            
        inflated_grid = np.copy(self.grid)
        
        # Simple square inflation for speed, or circular
        # Using a convolution or simple nested loops
        for x in range(self.width_cells):
            for y in range(self.height_cells):
                if self.grid[x, y] == 1:
                    # Inflate around this cell
                    x_min = max(0, x - inflation_cells)
                    x_max = min(self.width_cells, x + inflation_cells + 1)
                    y_min = max(0, y - inflation_cells)
                    y_max = min(self.height_cells, y + inflation_cells + 1)
                    inflated_grid[x_min:x_max, y_min:y_max] = 1
                    
        self.grid = inflated_grid

    def is_occupied(self, gx, gy):
        if 0 <= gx < self.width_cells and 0 <= gy < self.height_cells:
            return self.grid[gx, gy] > 0
        return True # Treat out of bounds as occupied
