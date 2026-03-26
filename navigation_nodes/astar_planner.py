import numpy as np
import heapq

class AStarPlanner:
    def __init__(self, grid_size=(100, 100), resolution=1.0, grid_offset=(40, 40)):
        self.grid_size = grid_size
        self.resolution = resolution
        self.grid_offset = grid_offset

    def heuristic(self, a, b):
        return np.hypot(b[0] - a[0], b[1] - a[1])

    def inflate_grid(self, grid, radius=1):
        inflated = grid.copy()
        rows, cols = grid.shape

        for r in range(rows):
            for c in range(cols):
                if grid[r][c] == 1:
                    for dx in range(-radius, radius+1):
                        for dy in range(-radius, radius+1):
                            nx, ny = r+dx, c+dy
                            if 0 <= nx < rows and 0 <= ny < cols:
                                inflated[nx][ny] = max(inflated[nx][ny], 0.7)
        return inflated

    def plan(self, start, goal, occupancy_grid):
        start_node = self.world_to_grid(start)
        goal_node = self.world_to_grid(goal)

        # 🔥 adaptive inflation
        grid = self.inflate_grid(occupancy_grid, radius=2)

        path = self._a_star(start_node, goal_node, grid)

        if path is None:
            print("⚠️ A* failed → using fallback")
            return self.fallback_path(start, goal)

        return self.smooth_path(path)

    def _a_star(self, start, goal, grid):
        open_list = []
        heapq.heappush(open_list, (0, start))

        came_from = {}
        g_score = {start: 0}
        closed = set()

        while open_list:
            _, current = heapq.heappop(open_list)

            if current == goal:
                return self.reconstruct(came_from, current)

            closed.add(current)

            for dx, dy in [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]:
                neighbor = (current[0]+dx, current[1]+dy)

                if not self.in_bounds(neighbor):
                    continue

                # 🔥 soft obstacle cost
                cost = grid[neighbor[0]][neighbor[1]]
                if cost >= 1:
                    continue

                tentative_g = g_score[current] + np.hypot(dx, dy) + cost*5

                if neighbor in closed:
                    continue

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_list, (f, neighbor))

        return None

    def reconstruct(self, came_from, current):
        path = []
        while current in came_from:
            path.append(self.grid_to_world(current))
            current = came_from[current]
        return path[::-1]

    def smooth_path(self, path):
        if len(path) < 3:
            return path

        smooth = [path[0]]
        for i in range(1, len(path)-1):
            prev = np.array(path[i-1])
            curr = np.array(path[i])
            nextp = np.array(path[i+1])

            if np.linalg.norm((nextp - prev)) > 1.5:
                smooth.append(tuple(curr))

        smooth.append(path[-1])
        return smooth

    def fallback_path(self, start, goal):
        # simple straight fallback
        path = []
        steps = 10
        for i in range(steps):
            x = start[0] + (goal[0]-start[0])*(i/steps)
            y = start[1] + (goal[1]-start[1])*(i/steps)
            path.append((x, y))
        return path

    def world_to_grid(self, pos):
        return (
            int((pos[0] + self.grid_offset[0]) / self.resolution),
            int((pos[1] + self.grid_offset[1]) / self.resolution)
        )

    def grid_to_world(self, node):
        return (
            node[0]*self.resolution - self.grid_offset[0],
            node[1]*self.resolution - self.grid_offset[1]
        )

    def in_bounds(self, node):
        return 0 <= node[0] < self.grid_size[0] and 0 <= node[1] < self.grid_size[1]
