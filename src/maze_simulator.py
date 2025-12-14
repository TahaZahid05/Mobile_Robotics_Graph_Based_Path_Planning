import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
from dstar_lite import DStarLite
from collections import deque
import random

class MazeSimulator:
    def __init__(self, rows, cols, start, goal, initial_obstacles, sensor_range = 3):
        self.rows = rows
        self.cols = cols
        self.start = start
        self.goal = goal
        self.initial_obstacles = initial_obstacles

        # EXPERIMENT PARAMETER: How far the robot can see
        self.sensor_range = sensor_range
        
        self.dstar = DStarLite(start[0], start[1], goal[0], goal[1], 
                               initial_obstacles, rows, cols)
        
        self.path = []
        self.current_pos = start
        self.user_obstacles = []
        
        self.fig, self.ax = plt.subplots(figsize=(12, 10))
        
        # -- PLOTTING HANDLES --
        self.path_line = None       # The blue line (history)
        self.planned_line = None    # The orange dashed line (future plan)
        self.robot_marker = None    # The blue dot
        
        self.setup_plot()
        
    def setup_plot(self):
        """Initializes the plotting environment, grid, and static elements."""
        self.ax.clear()
        self.ax.set_xlim(-0.5, self.cols - 0.5)
        self.ax.set_ylim(-0.5, self.rows - 0.5)
        self.ax.set_aspect('equal')
        self.ax.invert_yaxis()
        
        # Grid lines (light grey)
        for i in range(self.rows + 1):
            self.ax.axhline(i - 0.5, color='gray', linewidth=0.5, alpha=0.3)
        for i in range(self.cols + 1):
            self.ax.axvline(i - 0.5, color='gray', linewidth=0.5, alpha=0.3)
        
        # Draw initial obstacles (black squares)
        for obs in self.initial_obstacles:
            rect = patches.Rectangle((obs[1] - 0.5, obs[0] - 0.5), 1, 1, 
                                     linewidth=1, edgecolor='black', 
                                     facecolor='black', alpha=0.8)
            self.ax.add_patch(rect)
        
        # Draw start (S) and goal (G)
        start_rect = patches.Rectangle((self.start[1] - 0.5, self.start[0] - 0.5), 
                                       1, 1, linewidth=2, edgecolor='darkgreen', 
                                       facecolor='lightgreen', alpha=0.8)
        self.ax.add_patch(start_rect)
        self.ax.text(self.start[1], self.start[0], 'S', ha='center', va='center', 
                    fontsize=20, fontweight='bold', color='darkgreen')
        
        goal_rect = patches.Rectangle((self.goal[1] - 0.5, self.goal[0] - 0.5), 
                                     1, 1, linewidth=2, edgecolor='darkred', 
                                     facecolor='lightcoral', alpha=0.8)
        self.ax.add_patch(goal_rect)
        self.ax.text(self.goal[1], self.goal[0], 'G', ha='center', va='center', 
                    fontsize=20, fontweight='bold', color='darkred')
        
        # Initialize plot lines
        self.path_line, = self.ax.plot([], [], 'b-', linewidth=3, alpha=0.6, label='History')
        self.planned_line, = self.ax.plot([], [], 'orange', linestyle='--', linewidth=2, alpha=0.8, label='Planned Path')
        self.robot_marker, = self.ax.plot([], [], 'bo', markersize=15, 
                                          markeredgecolor='darkblue', markeredgewidth=2, label='Robot')

        self.ax.set_title('D* Lite - Click to add obstacles, Press SPACE to start', 
                         fontsize=14, fontweight='bold')
        self.ax.legend(loc='upper right')
        
    def onclick(self, event):
        """Handles mouse clicks to add dynamic obstacles."""
        if event.inaxes != self.ax: return
        col = int(round(event.xdata))
        row = int(round(event.ydata))
        
        if not (0 <= row < self.rows and 0 <= col < self.cols): return
        if (row, col) == self.start or (row, col) == self.goal: return
        
        # Check if obstacle exists
        if (row, col) in self.initial_obstacles or (row, col) in [(o.x, o.y) for o in self.user_obstacles]:
            return
        
        print(f"Added obstacle at ({row}, {col})")
        self.user_obstacles.append(self.dstar.grid[row][col])
        
        rect = patches.Rectangle((col - 0.5, row - 0.5), 1, 1, 
                                linewidth=1, edgecolor='red', 
                                facecolor='red', alpha=0.8)
        self.ax.add_patch(rect)
        self.fig.canvas.draw()
        

    def get_obstacles_in_sensor_range(self, current_pos, sensor_range=3):
        """
        Simulates the robot's sensor. 
        Returns obstacles that are within 'sensor_range' of the current position.
        """
        new_obstacles = []
        for obs in self.user_obstacles:
            dist = abs(obs.x - current_pos[0]) + abs(obs.y - current_pos[1])
            if dist <= sensor_range:
                new_obstacles.append(obs)
        # Remove detected obstacles from the list so we don't detect them twice
        for obs in new_obstacles:
            self.user_obstacles.remove(obs)
        return new_obstacles

    def extract_future_path(self):
        """
        Simulates the robot's logic to trace the path from current position to Goal
        using the current D* Lite 'g' values.
        """
        future_path = []
        curr = self.dstar.start # Start tracing from current robot position
        
        # Limit steps to prevent infinite loops if cost map is inconsistent temporarily
        max_steps = self.rows * self.cols
        steps = 0
        
        while (curr.x, curr.y) != self.goal and steps < max_steps:
            future_path.append((curr.x, curr.y))
            
            neighbors = self.dstar.get_neighbors(curr)
            best_next = None
            min_val = float('inf')
            
            # Find neighbor with lowest (EdgeCost + CostToGoal)
            for neighbor in neighbors:
                cost = self.dstar.calculate_cost(curr, neighbor)
                g_val = self.dstar.g[neighbor.x][neighbor.y]
                val = cost + g_val
                
                if val < min_val:
                    min_val = val
                    best_next = neighbor
            
            if best_next is None or min_val == float('inf'):
                break 
            
            curr = best_next
            steps += 1
            
        future_path.append(self.goal) # Add goal
        return future_path

    def update_visualization(self):
        # 1. Update History (Blue Line)
        if len(self.path) > 1:
            path_arr = np.array(self.path)
            self.path_line.set_data(path_arr[:, 1], path_arr[:, 0])
        
        # 2. Update Robot Position
        current = self.path[-1]
        self.robot_marker.set_data([current[1]], [current[0]]) # Pass as sequence
        
        # 3. Update Future Plan (Orange Dashed Line)
        future_path = self.extract_future_path()
        if len(future_path) > 1:
            future_arr = np.array(future_path)
            self.planned_line.set_data(future_arr[:, 1], future_arr[:, 0])
        else:
            self.planned_line.set_data([], [])

        self.fig.canvas.draw()
        plt.pause(0.1)

    def run_dstar(self):
        print("\n=== Starting D* Lite Path Planning ===")
        print(f"Start: {self.start}, Goal: {self.goal}")
        
        self.fig.canvas.mpl_disconnect(self.cid)
        
        step = 0
        self.path = [self.start]
        
        # --- INITIALIZATION ---
        self.dstar.initialize()
        self.dstar.computeShortestPath()
        
        self.update_visualization()
        plt.pause(1.0) 
        
        # --- MAIN LOOP ---
        while (self.dstar.start.x, self.dstar.start.y) != self.goal:
            step += 1
            
            # 0. Check if path exists
            if self.dstar.g[self.dstar.start.x][self.dstar.start.y] == float('inf'):
                print("\nNo path exists!")
                self.ax.set_title('No Path Found!', fontsize=14, fontweight='bold', color='red')
                self.fig.canvas.draw()
                return
            
            # 1. Sense Obstacles
            current_pos = (self.dstar.start.x, self.dstar.start.y)
            new_obstacles = self.get_obstacles_in_sensor_range(current_pos, sensor_range=3)
            
            # 2. Replan if needed
            if new_obstacles:
                print(f"\nDetected {len(new_obstacles)} new obstacle(s)! Replanning...")
                for obs in new_obstacles:
                    self.dstar.obstacles.add((obs.x, obs.y))
                
                # Update D* Lite with new information
                self.dstar.scan_for_changes(new_obstacles)
                
                # Visualize replanning process
                self.ax.set_title('Replanning...', fontsize=14, fontweight='bold', color='orange')
                self.update_visualization() 
                plt.pause(0.5) 
                print("Replanning complete!")
                self.ax.set_title(f'Step {step}: Moving...', fontsize=14, fontweight='bold')
            
            # 3. Move Robot
            temp_val = float('inf')
            best_node = None
            lst_start_neighbors = self.dstar.get_neighbors(self.dstar.start)
            
            for i in lst_start_neighbors:
                edge_cost = self.dstar.calculate_cost(self.dstar.start, i)
                if ((edge_cost + self.dstar.g[i.x][i.y]) < temp_val):
                    temp_val = (edge_cost + self.dstar.g[i.x][i.y])
                    best_node = i
            
            if temp_val == float('inf'):
                print("\nStuck! No valid neighbors!")
                self.ax.set_title('Robot Stuck!', fontsize=14, fontweight='bold', color='red')
                self.fig.canvas.draw()
                return
            
            # Update robot position
            self.dstar.start = best_node
            self.path.append((best_node.x, best_node.y))
            
            self.update_visualization()
        
        print(f"\nGoal reached in {step} steps!")
        self.ax.set_title(f'Goal Reached! (Steps: {step})', 
                         fontsize=14, fontweight='bold', color='green')
        self.fig.canvas.draw()
    
    def onkey(self, event):
        if event.key == ' ':
            print("\nStarting pathfinding")
            self.run_dstar()
    
    def run(self):
        print("\n" + "="*60)
        print("D* LITE INTERACTIVE MAZE SIMULATOR")
        print("Instructions:")
        print("1. Click to add obstacles (red)")
        print("2. Press SPACEBAR to start")
        print("3. BLUE line = History, ORANGE line = Planned Path")
        print("="*60 + "\n")
        
        self.cid = self.fig.canvas.mpl_connect('button_press_event', self.onclick)
        self.kid = self.fig.canvas.mpl_connect('key_press_event', self.onkey)
        plt.show()


def generate_solvable_maze(rows, cols, start, goal, obstacle_density=0.2):
    """
    Generates a random maze that is guaranteed to be solvable.
    Strategy:
    1. Randomly place obstacles based on density.
    2. Check if a path exists using BFS.
    3. If blocked, 'carve' a path by removing obstacles from Start to Goal.
    """
    obstacles = set()
    total_cells = rows * cols
    num_obstacles = int(total_cells * obstacle_density)
    
    # Random placement
    while len(obstacles) < num_obstacles:
        x = np.random.randint(0, rows)
        y = np.random.randint(0, cols)
        if (x, y) != start and (x, y) != goal:
            obstacles.add((x, y))
            
    # BFS to check connectivity
    def has_path():
        queue = deque([start])
        visited = {start}
        while queue:
            cx, cy = queue.popleft()
            if (cx, cy) == goal: return True
            for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                nx, ny = cx + dx, cy + dy
                if (0 <= nx < rows and 0 <= ny < cols and (nx, ny) not in obstacles and (nx, ny) not in visited):
                    visited.add((nx, ny))
                    queue.append((nx, ny))
        return False

    if has_path():
        print(f"Map generated (Density: {obstacle_density:.2f}) - Naturally solvable.")
        return list(obstacles)

    # Force a path if it doesn't exist
    curr = start
    while curr != goal:
        cx, cy = curr
        neighbors = []
        for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            nx, ny = cx + dx, cy + dy
            if 0 <= nx < rows and 0 <= ny < cols: neighbors.append((nx, ny))
        
        neighbors.sort(key=lambda p: abs(p[0]-goal[0]) + abs(p[1]-goal[1]))
        next_step = neighbors[0]
        if len(neighbors) > 1 and random.random() < 0.3: next_step = neighbors[1]
        if next_step in obstacles: obstacles.remove(next_step)
        curr = next_step
        
    return list(obstacles)

if __name__ == "__main__":
    # Configuration
    ROWS = 20
    COLS = 25
    START = (1, 1)
    GOAL = (18, 23)
    SENSOR_RANGE = 3
    OBSTACLE_DENSITY = 0.3
    
    INITIAL_OBSTACLES = generate_solvable_maze(ROWS, COLS, START, GOAL, OBSTACLE_DENSITY)
    simulator = MazeSimulator(ROWS, COLS, START, GOAL, INITIAL_OBSTACLES, SENSOR_RANGE)
    simulator.run()