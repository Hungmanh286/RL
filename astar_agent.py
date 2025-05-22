import heapq

# A* search to find path from start to goal on grid
# map_grid: 2D list of 0 (free) and 1 (obstacle)
# start, goal: tuples (r, c)
def run_astar(map_grid, start, goal):
    n_rows = len(map_grid)
    n_cols = len(map_grid[0]) if n_rows > 0 else 0

    def heuristic(a, b):
        # Manhattan distance
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    # Priority queue with entries (f_score, g_score, position)
    open_set = []
    heapq.heappush(open_set, (heuristic(start, goal), 0, start))

    came_from = {}
    g_score = {start: 0}

    # Directions: move char and delta
    moves = [(-1, 0, 'U'), (1, 0, 'D'), (0, -1, 'L'), (0, 1, 'R')]

    while open_set:
        f, g, current = heapq.heappop(open_set)
        # If reached goal, reconstruct path
        if current == goal:
            # build path from start to goal
            path = []
            node = goal
            while node != start:
                path.append(node)
                node = came_from[node]
            path.append(start)
            path.reverse()
            # choose first step
            if len(path) > 1:
                next_node = path[1]
                dr = next_node[0] - start[0]
                dc = next_node[1] - start[1]
                for ddr, ddc, mv in moves:
                    if (ddr, ddc) == (dr, dc):
                        return mv, len(path)-1
            return 'S', 0

        # explore neighbors
        for dr, dc, mv in moves:
            neighbor = (current[0] + dr, current[1] + dc)
            # valid cell
            if 0 <= neighbor[0] < n_rows and 0 <= neighbor[1] < n_cols and map_grid[neighbor[0]][neighbor[1]] == 0:
                tentative_g = g + 1
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, tentative_g, neighbor))
                    came_from[neighbor] = current
    # no path
    return 'S', float('inf')

class AStarAgents:
    """
    Multi-agent delivery using A* for pathfinding instead of BFS.
    """
    def __init__(self):
        self.robots = []
        self.packages = []
        self.packages_free = []
        self.robots_target = []
        self.n_robots = 0
        self.map = None
        self.is_init = False

    def init_agents(self, state):
        """
        Initialize internal state from environment state.
        state: dict from env.get_state()
        """
        self.map = state['map']
        self.n_robots = len(state['robots'])
        # robot tuples: (r, c, carrying_id)
        self.robots = [(r-1, c-1, carrying) for r, c, carrying in state['robots']]
        self.robots_target = ['free'] * self.n_robots
        # packages: list of tuples (pkg_id, start_r, start_c, goal_r, goal_c, deadline)
        self.packages = [(pid, sr-1, sc-1, tr-1, tc-1, stime, ddl) for pid, sr, sc, tr, tc, stime, ddl in state['packages']]
        self.packages_free = [True] * len(self.packages)

    def update_inner_state(self, state):
        # update robot positions and carrying status
        for i, robot_info in enumerate(state['robots']):
            r, c, carrying = robot_info
            prev = self.robots[i]
            self.robots[i] = (r-1, c-1, carrying)
            # if dropped
            if prev[2] != 0 and carrying == 0:
                self.robots_target[i] = 'free'
        # add new waiting packages
        for p in state['packages']:
            pid, sr, sc, tr, tc, st, ddl = p
            self.packages.append((pid, sr-1, sc-1, tr-1, tc-1, st, ddl))
            self.packages_free.append(True)

    def update_move_to_target(self, robot_id, pkg_idx, phase='start'):
        rpos = (self.robots[robot_id][0], self.robots[robot_id][1])
        pkg = self.packages[pkg_idx]
        # choose target coordinate
        target = (pkg[1], pkg[2]) if phase == 'start' else (pkg[3], pkg[4])
        move, dist = run_astar(self.map, rpos, target)
        pkg_act = 0
        if dist == 0:
            pkg_act = 1 if phase=='start' else 2
        return move, str(pkg_act)

    def get_actions(self, state):
        if not self.is_init:
            self.is_init = True
            self.update_inner_state(state)
        else:
            self.update_inner_state(state)
        actions = []
        # assign tasks for each robot
        for i in range(self.n_robots):
            r = self.robots[i]
            if self.robots_target[i] != 'free':
                pkg_id = self.robots_target[i]
                if r[2] != 0:
                    move, act = self.update_move_to_target(i, pkg_id-1, 'target')
                else:
                    move, act = self.update_move_to_target(i, pkg_id-1, 'start')
            else:
                # find nearest free package
                best_j, best_d = None, float('inf')
                for j, pkg in enumerate(self.packages):
                    if not self.packages_free[j]: continue
                    d = abs(pkg[1] - r[0]) + abs(pkg[2] - r[1])
                    if d < best_d:
                        best_d, best_j = d, j
                if best_j is not None:
                    self.packages_free[best_j] = False
                    self.robots_target[i] = self.packages[best_j][0]
                    move, act = self.update_move_to_target(i, best_j, 'start')
                else:
                    move, act = 'S', '0'
            actions.append((move, act))
        return actions
