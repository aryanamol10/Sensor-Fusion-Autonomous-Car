import math
import time

class TrajectoryManager:
    def __init__(self, target_velocity):
        """
        Manages the timing and interpolation of path waypoints.
        target_velocity: Desired speed of the robot in m/s
        """
        self.target_velocity = target_velocity
        self.path = []
        self.timed_path = [] # list of (x, y, target_time)
        self.start_time = 0.0

    def set_path(self, path):
        """
        Takes a raw sequence of (x, y) waypoints from A* and assigns target timestamps
        based on the target_velocity.
        """
        self.path = path
        self.timed_path = []
        
        if not path:
            return
            
        current_time = 0.0
        self.timed_path.append((path[0][0], path[0][1], current_time))
        
        for i in range(1, len(path)):
            prev_x, prev_y = path[i-1]
            curr_x, curr_y = path[i]
            
            dist = math.hypot(curr_x - prev_x, curr_y - prev_y)
            time_to_travel = dist / self.target_velocity
            current_time += time_to_travel
            
            self.timed_path.append((curr_x, curr_y, current_time))
            
        self.start_time = time.time()

    def get_target_waypoint(self):
        """
        Based on the current elapsed time, returns the interpolated target (x, y) 
        we should be at right now to maintain accurate timing intervals.
        """
        if not self.timed_path:
            return None, None
            
        elapsed = time.time() - self.start_time
        
        # If we exceeded the total time, just return the last waypoint
        if elapsed >= self.timed_path[-1][2]:
            return self.timed_path[-1][0], self.timed_path[-1][1]
            
        # Find which segment we are currently in
        for i in range(1, len(self.timed_path)):
            if elapsed < self.timed_path[i][2]:
                prev_x, prev_y, prev_t = self.timed_path[i-1]
                curr_x, curr_y, curr_t = self.timed_path[i]
                
                # Interpolate between prev and curr based on time
                segment_duration = curr_t - prev_t
                time_in_segment = elapsed - prev_t
                ratio = time_in_segment / segment_duration
                
                target_x = prev_x + ratio * (curr_x - prev_x)
                target_y = prev_y + ratio * (curr_y - prev_y)
                
                return target_x, target_y
                
        return self.timed_path[-1][0], self.timed_path[-1][1]

    def is_finished(self):
        if not self.timed_path:
            return True
        return (time.time() - self.start_time) >= self.timed_path[-1][2]
