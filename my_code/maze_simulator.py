import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import numpy as np
from dstar_lite import DStarLite, Node

class MazeSimulator:
    def __init__(self, rows, cols, start, goal, initial_obstacles):
        self.rows = rows
        self.cols = cols
        self.start = start
        self.goal = goal
        self.initial_obstacles = initial_obstacles
        
        # Initialize D* Lite
        self.dstar = DStarLite(start[0], start[1], goal[0], goal[1], 
                               initial_obstacles, rows, cols)
        
        # Track the path
        self.path = []
        self.current_pos = start
        
        # User-added obstacles
        self.user_obstacles = []
        
        # Setup the plot
        self.fig, self.ax = plt.subplots(figsize=(12, 10))
        self.setup_plot()
        
    def setup_plot(self):
        """Setup the matplotlib plot"""
        self.ax.clear()
        self.ax.set_xlim(-0.5, self.cols - 0.5)
        self.ax.set_ylim(-0.5, self.rows - 0.5)
        self.ax.set_aspect('equal')
        self.ax.invert_yaxis()
        
        # Grid lines
        for i in range(self.rows + 1):
            self.ax.axhline(i - 0.5, color='gray', linewidth=0.5, alpha=0.3)
        for i in range(self.cols + 1):
            self.ax.axvline(i - 0.5, color='gray', linewidth=0.5, alpha=0.3)
        
        # Draw initial obstacles
        for obs in self.initial_obstacles:
            rect = patches.Rectangle((obs[1] - 0.5, obs[0] - 0.5), 1, 1, 
                                     linewidth=1, edgecolor='black', 
                                     facecolor='black', alpha=0.8)
            self.ax.add_patch(rect)
        
        # Draw start (green) and goal (red)
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
        
        self.ax.set_title('D* Lite Path Planning - Click to add obstacles, Press SPACE to start', 
                         fontsize=14, fontweight='bold')
        self.ax.set_xlabel('Column', fontsize=12)
        self.ax.set_ylabel('Row', fontsize=12)
        
    def onclick(self, event):
        """Handle mouse clicks to add obstacles"""
        if event.inaxes != self.ax:
            return
        
        col = int(round(event.xdata))
        row = int(round(event.ydata))
        
        # Check bounds
        if not (0 <= row < self.rows and 0 <= col < self.cols):
            return
        
        # Don't allow obstacles on start or goal
        if (row, col) == self.start or (row, col) == self.goal:
            print(f"Cannot place obstacle on start/goal position!")
            return
        
        # Check if already an obstacle
        if (row, col) in self.initial_obstacles or (row, col) in [(o.x, o.y) for o in self.user_obstacles]:
            print(f"Obstacle already exists at ({row}, {col})")
            return
        
        # Add obstacle
        print(f"Added obstacle at ({row}, {col})")
        self.user_obstacles.append(self.dstar.grid[row][col])
        
        # Draw the new obstacle
        rect = patches.Rectangle((col - 0.5, row - 0.5), 1, 1, 
                                linewidth=1, edgecolor='red', 
                                facecolor='red', alpha=0.8)
        self.ax.add_patch(rect)
        self.fig.canvas.draw()
        
    def get_user_obstacles(self):
        """Called by D* Lite to get newly added obstacles"""
        if self.user_obstacles:
            new_obs = self.user_obstacles.copy()
            self.user_obstacles.clear()
            return new_obs
        return []
    
    def get_obstacles_in_sensor_range(self, current_pos, sensor_range=3):
        """Detect obstacles within sensor range of current position"""
        new_obstacles = []
        
        for obs in self.user_obstacles:
            # Calculate distance from current position
            dist = max(abs(obs.x - current_pos[0]), abs(obs.y - current_pos[1]))
            
            if dist <= sensor_range:
                new_obstacles.append(obs)
        
        # Remove detected obstacles from pending list
        for obs in new_obstacles:
            self.user_obstacles.remove(obs)
        
        return new_obstacles
    
    def run_dstar(self):
        """Run D* Lite algorithm step by step"""
        print("\n=== Starting D* Lite Path Planning ===")
        print(f"Start: {self.start}, Goal: {self.goal}")
        
        # Store original onclick handler and disconnect it
        self.fig.canvas.mpl_disconnect(self.cid)
        
        # Run the main D* Lite algorithm
        step = 0
        self.path = [self.start]
        
        # Call the main function with our obstacle callback
        original_start = self.dstar.start
        
        # We'll manually step through to visualize
        self.dstar.last = self.dstar.start
        self.dstar.initialize()
        self.dstar.computeShortestPath()
        
        while (self.dstar.start.x, self.dstar.start.y) != self.goal:
            step += 1
            
            if self.dstar.g[self.dstar.start.x][self.dstar.start.y] == float('inf'):
                print("\n❌ No path exists!")
                self.ax.set_title('No Path Found!', fontsize=14, fontweight='bold', color='red')
                self.fig.canvas.draw()
                return
            
            # Find best next node
            temp_val = float('inf')
            best_node = None
            lst_start_neighbors = self.dstar.get_neighbors(self.dstar.start)
            
            for i in lst_start_neighbors:
                edge_cost = self.dstar.calculate_cost(self.dstar.start, i)
                if ((edge_cost + self.dstar.g[i.x][i.y]) < temp_val):
                    temp_val = (edge_cost + self.dstar.g[i.x][i.y])
                    best_node = i
            
            if temp_val == float('inf'):
                print("\n❌ Stuck! No valid neighbors!")
                self.ax.set_title('Robot Stuck!', fontsize=14, fontweight='bold', color='red')
                self.fig.canvas.draw()
                return
            
            # Move to best node
            self.dstar.start = best_node
            self.path.append((best_node.x, best_node.y))
            
            # Draw current position
            self.draw_path()
            print(f"Step {step}: Moved to ({best_node.x}, {best_node.y})")
            
            # Check for new obstacles
            current_pos = (self.dstar.start.x, self.dstar.start.y)
            new_obstacles = self.get_obstacles_in_sensor_range(current_pos, sensor_range=3)
            
            if new_obstacles:
                print(f"\n⚠️  Detected {len(new_obstacles)} new obstacle(s)! Replanning...")
                
                # Add to internal obstacle set
                for obs in new_obstacles:
                    self.dstar.obstacles.add((obs.x, obs.y))
                
                # Replan
                self.dstar.scan_for_changes(new_obstacles)
                print("✓ Replanning complete!")
            
            plt.pause(0.3)
        
        print(f"\n✓ Goal reached in {step} steps!")
        print(f"Path length: {len(self.path)}")
        self.ax.set_title(f'Goal Reached! (Steps: {step})', 
                         fontsize=14, fontweight='bold', color='green')
        self.fig.canvas.draw()
    
    def draw_path(self):
        """Draw the current path"""
        # Clear previous path drawings
        for artist in self.ax.lines + self.ax.collections:
            artist.remove()
        
        if len(self.path) > 1:
            path_array = np.array(self.path)
            self.ax.plot(path_array[:, 1], path_array[:, 0], 
                        'b-', linewidth=3, alpha=0.6, label='Path')
            
        # Draw current position
        current = self.path[-1]
        self.ax.plot(current[1], current[0], 'bo', markersize=15, 
                    markeredgecolor='darkblue', markeredgewidth=2, label='Robot')
        
        self.ax.legend(loc='upper right')
        self.fig.canvas.draw()
    
    def onkey(self, event):
        """Handle keyboard events"""
        if event.key == ' ':  # Spacebar to start
            print("\n🚀 Starting pathfinding...")
            self.run_dstar()
    
    def run(self):
        """Run the interactive simulation"""
        print("\n" + "="*60)
        print("D* LITE INTERACTIVE MAZE SIMULATOR")
        print("="*60)
        print("\nInstructions:")
        print("1. Click on cells to add obstacles (red)")
        print("2. Press SPACEBAR to start the robot")
        print("3. Robot will find path from Start (S) to Goal (G)")
        print("4. Watch as D* Lite replans around obstacles!")
        print("="*60 + "\n")
        
        # Connect event handlers
        self.cid = self.fig.canvas.mpl_connect('button_press_event', self.onclick)
        self.kid = self.fig.canvas.mpl_connect('key_press_event', self.onkey)
        
        plt.show()


def generate_random_maze(rows, cols, obstacle_density=0.2):
    """Generate random obstacles for the maze"""
    obstacles = set()
    num_obstacles = int(rows * cols * obstacle_density)
    
    while len(obstacles) < num_obstacles:
        x = np.random.randint(0, rows)
        y = np.random.randint(0, cols)
        obstacles.add((x, y))
    
    return list(obstacles)


def generate_corridor_maze(rows, cols):
    """Generate a maze with corridors"""
    obstacles = []
    
    # Create some walls with gaps
    for i in range(2, rows - 2, 3):
        for j in range(cols):
            if j % 4 != 0:  # Leave gaps every 4 cells
                obstacles.append((i, j))
    
    return obstacles


if __name__ == "__main__":
    # Configuration
    ROWS = 20
    COLS = 25
    START = (1, 1)
    GOAL = (18, 23)
    
    # Choose maze type:
    # 1. Random maze
    INITIAL_OBSTACLES = generate_random_maze(ROWS, COLS, obstacle_density=0.15)
    
    # 2. Corridor maze
    # INITIAL_OBSTACLES = generate_corridor_maze(ROWS, COLS)
    
    # 3. Custom maze - uncomment and add your own
    # INITIAL_OBSTACLES = [(5, 5), (5, 6), (5, 7), (10, 10), (10, 11)]
    
    # Make sure start and goal are not obstacles
    # INITIAL_OBSTACLES = [obs for obs in INITIAL_OBSTACLES 
    #                      if obs != START and obs != GOAL]
    
    # Create and run simulator
    simulator = MazeSimulator(ROWS, COLS, START, GOAL, INITIAL_OBSTACLES)
    simulator.run()