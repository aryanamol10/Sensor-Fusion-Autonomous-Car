import heapq
import math

class Node:
    def __init__(self, x, y, cost, heuristic, parent_index):
        self.x = x
        self.y = y
        self.cost = cost
        self.heuristic = heuristic
        self.parent_index = parent_index

    def __lt__(self, other):
        return (self.cost + self.heuristic) < (other.cost + other.heuristic)

class AStarPlanner:
    def __init__(self, resolution, robot_radius):
        """
        resolution: Grid resolution (m/cell)
        robot_radius: Robot radius (m) used for distance checks if not using pre-inflated map
        """
        self.resolution = resolution
        self.robot_radius = robot_radius
        
        # 8-connected grid
        self.motion = [
            [-1, 0, 1.0], [0, -1, 1.0], [1, 0, 1.0], [0, 1, 1.0],
            [-1, -1, math.sqrt(2)], [-1, 1, math.sqrt(2)],
            [1, -1, math.sqrt(2)], [1, 1, math.sqrt(2)]
        ]

    def plan(self, start_x, start_y, goal_x, goal_y, occupancy_grid):
        """
        A* search to find shortest path from start to goal on the occupancy grid.
        start_x, start_y: Start coordinates (m)
        goal_x, goal_y: Goal coordinates (m)
        occupancy_grid: OccupancyGrid object
        
        Returns:
            list of tuples: [(x1, y1), (x2, y2), ...] path in world coordinates.
        """
        start_gx, start_gy = occupancy_grid.world_to_grid(start_x, start_y)
        goal_gx, goal_gy = occupancy_grid.world_to_grid(goal_x, goal_y)
        
        if occupancy_grid.is_occupied(goal_gx, goal_gy):
            print("Goal is occupied!")
            return []
            
        start_node = Node(start_gx, start_gy, 0.0, self._calc_heuristic(start_gx, start_gy, goal_gx, goal_gy), -1)
        
        open_set = {self._calc_index(start_gx, start_gy, occupancy_grid): start_node}
        closed_set = {}
        pq = [(start_node.cost + start_node.heuristic, self._calc_index(start_gx, start_gy, occupancy_grid))]
        
        while pq:
            _, current_id = heapq.heappop(pq)
            
            if current_id in closed_set:
                continue
                
            current_node = open_set[current_id]
            closed_set[current_id] = current_node
            del open_set[current_id]
            
            # Check if reached goal
            if current_node.x == goal_gx and current_node.y == goal_gy:
                return self._calc_final_path(current_node, closed_set, occupancy_grid)
                
            # Expand neighbors
            for move in self.motion:
                next_x = current_node.x + move[0]
                next_y = current_node.y + move[1]
                
                # Check bounds and occupancy
                if occupancy_grid.is_occupied(next_x, next_y):
                    continue
                    
                next_cost = current_node.cost + move[2]
                next_id = self._calc_index(next_x, next_y, occupancy_grid)
                
                if next_id in closed_set:
                    continue
                    
                if next_id in open_set:
                    if open_set[next_id].cost > next_cost:
                        open_set[next_id].cost = next_cost
                        open_set[next_id].parent_index = current_id
                        # Needs to re-heapify, simple push is enough for standard Python heapq since we ignore closed
                        heapq.heappush(pq, (next_cost + open_set[next_id].heuristic, next_id))
                else:
                    h = self._calc_heuristic(next_x, next_y, goal_gx, goal_gy)
                    new_node = Node(next_x, next_y, next_cost, h, current_id)
                    open_set[next_id] = new_node
                    heapq.heappush(pq, (next_cost + h, next_id))
                    
        print("A* could not find a path")
        return []

    def _calc_heuristic(self, x1, y1, x2, y2):
        return math.hypot(x1 - x2, y1 - y2)

    def _calc_index(self, gx, gy, occupancy_grid):
        # Unique index for grid cells
        return gy * occupancy_grid.width_cells + gx

    def _calc_final_path(self, goal_node, closed_set, occupancy_grid):
        path = []
        current = goal_node
        while current.parent_index != -1:
            wx, wy = occupancy_grid.grid_to_world(current.x, current.y)
            path.append((wx, wy))
            current = closed_set[current.parent_index]
        # Add start node
        wx, wy = occupancy_grid.grid_to_world(current.x, current.y)
        path.append((wx, wy))
        
        path.reverse()
        return path
