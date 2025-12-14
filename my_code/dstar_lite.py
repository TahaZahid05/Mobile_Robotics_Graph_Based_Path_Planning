import matplotlib.pyplot as plt
import heapq
import numpy as np
import random
import sys
import math


class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __lt__(self, other):
        return False
    
    def __eq__(self, other):
        if not isinstance(other, Node):
            return False
        return self.x == other.x and self.y == other.y
    
    def __hash__(self):
        return hash((self.x, self.y))

class DStarLite:
    def __init__(self, start_x, start_y, end_x, end_y, initial_obstacles, row, col):
        self.grid = [[Node(x,y) for y in range(col)] for x in range(row)]
        self.start = self.grid[start_x][start_y]
        self.end = self.grid[end_x][end_y]
        self.U = []
        self.k = 0
        self.rhs = [[float('inf') for _ in range(col)] for _ in range(row)]
        self.g = [[float('inf') for _ in range(col)] for _ in range(row)]

        self.entry_finder = {}
        self.counter = 0  # Unique sequence count for tie-breaking

        self.obstacles = set()
        for (obs_x, obs_y) in initial_obstacles:
            self.obstacles.add((obs_x, obs_y))
        

    def push(self, task, priority):
        """Add a new task or update the priority of an existing task"""
        if task in self.entry_finder:
            self.remove(task)
        count = self.counter
        self.counter += 1
        entry = [priority, count, task]
        self.entry_finder[task] = entry
        heapq.heappush(self.U, entry)

    def remove(self, task):
        """Mark an existing task as removed"""
        entry = self.entry_finder.pop(task)
        entry[-1] = None  # Mark as removed

    def pop(self):
        """Remove and return the lowest priority task"""
        while self.U:
            priority, count, task = heapq.heappop(self.U)
            if task is not None:
                del self.entry_finder[task]
                return task, priority
        raise KeyError("empty heap")
    
    def contains(self, task):
        """Check if task is in the priority queue"""
        return task in self.entry_finder
    
    def peek(self):
        """Return the lowest priority task without removing it"""
        if not self.U:
            return None, [float('inf'), float('inf')]

        while self.U:
            entry = self.U[0]
            priority, count, task = entry
            
            if task is not None:
                return task, priority
                
            heapq.heappop(self.U)  # Remove invalid entries
        
        return None, [float('inf'), float('inf')]



    def calculate_heuristic(self, c1, c2):
        term_1 = abs(c1.x - c2.x)
        term_2 = abs(c1.y - c2.y)
        return max(term_1, term_2)

    def calculate_cost(self, u, v):
        if (v.x, v.y) in self.obstacles or (u.x, u.y) in self.obstacles:
            return float('inf')
        return 1
    
    def calculate_key(self, s):
        term_1 = min(self.g[s.x][s.y], self.rhs[s.x][s.y]) + self.calculate_heuristic(self.start, s) + self.k
        term_2 = min(self.g[s.x][s.y], self.rhs[s.x][s.y])
        return [term_1, term_2]


    def initialize(self):
        self.U.clear()
        self.entry_finder.clear()
        self.counter = 0
        self.k = 0
        self.rhs[self.end.x][self.end.y] = 0
        val = self.calculate_heuristic(self.start, self.end)
        self.push(self.end, [val, 0])

    def get_neighbors(self, u):
        neighbors = []
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

        for dx, dy in directions:
            nx, ny = u.x + dx, u.y + dy
            
            if 0 <= nx < len(self.grid) and 0 <= ny < len(self.grid[0]):
                neighbors.append(self.grid[nx][ny])
                
        return neighbors
    
    def updateVertex(self, s):
        if (self.g[s.x][s.y] != self.rhs[s.x][s.y]):
            if self.contains(s):
                self.remove(s)
            self.push(s, self.calculate_key(s))
        elif (self.g[s.x][s.y] == self.rhs[s.x][s.y] and self.contains(s)):
            self.remove(s)

    def key_less_than(self, k1, k2):
        return k1[0] < k2[0] or (k1[0] == k2[0] and k1[1] < k2[1])

    def computeShortestPath(self):
        while True:
            task, k_old = self.peek()
            if task is None:
                break
            
            k_start = self.calculate_key(self.start)
            
            if (not self.key_less_than(k_old, k_start) and 
                self.rhs[self.start.x][self.start.y] == self.g[self.start.x][self.start.y]):
                break
            
            u, k_old = self.pop()
            k_new = self.calculate_key(u)

            if self.key_less_than(k_old, k_new):
                self.push(u, k_new)

            elif self.g[u.x][u.y] > self.rhs[u.x][u.y]:
                self.g[u.x][u.y] = self.rhs[u.x][u.y]

                neighbors_lst = self.get_neighbors(u)

                for curr in neighbors_lst:
                    if curr != self.end:
                        edge_cost = self.calculate_cost(curr, u)
                        self.rhs[curr.x][curr.y] = min(self.rhs[curr.x][curr.y], 
                                                    edge_cost + self.g[u.x][u.y])
                    self.updateVertex(curr)
            else:
                g_old = self.g[u.x][u.y]
                self.g[u.x][u.y] = float('inf')

                neighbors_lst = self.get_neighbors(u)

                for curr in (neighbors_lst + [u]):
                    edge_cost = self.calculate_cost(curr, u)
                    if self.rhs[curr.x][curr.y] == (edge_cost + g_old) or curr == u:
                        if curr != self.end:
                            temp_rhs = float('inf')

                            curr_neighbors_lst = self.get_neighbors(curr)

                            for j in curr_neighbors_lst:
                                edge_cost = self.calculate_cost(curr, j)
                                temp_rhs = min(temp_rhs, (edge_cost + self.g[j.x][j.y]))

                            self.rhs[curr.x][curr.y] = temp_rhs

                    self.updateVertex(curr)


    def scan_for_changes(self, new_obstacles):
        if not new_obstacles:
            return
        
        self.k += self.calculate_heuristic(self.last, self.start)
        self.last = self.start

        for v in new_obstacles:
            # self.updateVertex(v)
            for u in self.get_neighbors(v):
                c_old = 1 
                c_new = float('inf')

                if c_old > c_new:
                    if u != self.end:
                        self.rhs[u.x][u.y] = min(self.rhs[u.x][u.y], c_new + self.g[v.x][v.y])

                elif self.rhs[u.x][u.y] == (c_old + self.g[v.x][v.y]):
                    if u != self.end:
                        new_rhs = float('inf')

                        for s_prime in self.get_neighbors(u):
                            cost = self.calculate_cost(u, s_prime)
                            new_rhs = min(new_rhs, cost + self.g[s_prime.x][s_prime.y])

                        self.rhs[u.x][u.y] = new_rhs
                self.updateVertex(u)
        self.computeShortestPath()


    def main(self, get_user_obstacles_func):
        self.last = self.start

        self.initialize()
        self.computeShortestPath()
        while (self.start != self.end):
            if self.g[self.start.x][self.start.y] == float('inf'):
                print("No path found!")
                break

            temp_val = float('inf')
            best_node = None
            lst_start_neighbors = self.get_neighbors(self.start)

            for i in lst_start_neighbors:
                edge_cost = self.calculate_cost(self.start, i)
                if ((edge_cost + self.g[i.x][i.y]) < temp_val):
                    temp_val = (edge_cost + self.g[i.x][i.y])
                    best_node = i

            if temp_val == float('inf'):
                print("Stuck! No path found!")
                break 

            self.start = best_node

            new_obstacles = get_user_obstacles_func()

            if new_obstacles:
                for obs in new_obstacles:
                    self.obstacles.add((obs.x, obs.y))
                
                self.scan_for_changes(new_obstacles)