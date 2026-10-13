"""
Offline maze solver comparison: DFS, BFS and A* on a discovered maze.

The flown code (challenge_1_maze_navigation.py) uses DFS for the Discovery Phase and
A* for the Racing Phase. This script runs all three algorithms, plus BFS, on the same
maze.json so the trade-offs can be compared without a drone.

Usage:
    python simulation/maze_solvers.py --maze maze.json --start 2,0 --goal 0,5
    python simulation/maze_solvers.py --maze maze.json --start 2,0 --goal 0,5 --plot docs/images/maze_solvers.png
"""
import argparse
import json
from collections import deque
from heapq import heappush, heappop

DIRECTIONS = ["N", "E", "S", "W"]
DX = {"N": 0, "E": 1, "S": 0, "W": -1}
DY = {"N": 1, "E": 0, "S": -1, "W": 0}
BACK = {"N": "S", "S": "N", "E": "W", "W": "E"}


def load_maze(path):
    """Return (width, height, {(x, y): {"N": bool, "E": bool, "S": bool, "W": bool}}).
    True means a wall, the same convention the drone code uses."""
    with open(path) as f:
        data = json.load(f)
    cells = {}
    for key, walls in data["cells"].items():
        x, y = map(int, key.split(","))
        cells[(x, y)] = walls
    return data["width"], data["height"], cells


def neighbors(maze, width, height, cell):
    """Open neighbours of a cell, in N, E, S, W order."""
    x, y = cell
    for d in DIRECTIONS:
        if maze[cell][d]:
            continue
        nx, ny = x + DX[d], y + DY[d]
        if 0 <= nx < width and 0 <= ny < height and (nx, ny) in maze:
            yield (nx, ny)


def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def rebuild(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    return path[::-1]


# ---------------------------------------------------------------- DFS
def dfs_discovery(maze, width, height, start):
    """Simulate the Discovery Phase of challenge 1.

    The drone walks to the first unvisited open neighbour (N, E, S, W order) and, when
    it reaches a dead end, physically flies back. Returns the visit order and the total
    number of one-cell moves (every tree edge is flown twice)."""
    order, moves = [], 0
    seen = set()

    def visit(cell):
        nonlocal moves
        seen.add(cell)
        order.append(cell)
        for nb in neighbors(maze, width, height, cell):
            if nb not in seen:
                moves += 1          # fly forward into the neighbour
                visit(nb)
                moves += 1          # physical backtrack
    visit(start)
    return order, moves


def dfs_path(maze, width, height, start, goal):
    """Depth-first path search. Finds a path, not necessarily the shortest one."""
    stack = [(start, [start])]
    seen = set()
    expanded = []
    while stack:
        cell, path = stack.pop()
        if cell in seen:
            continue
        seen.add(cell)
        expanded.append(cell)
        if cell == goal:
            return path, expanded
        # reversed so that N is explored first, like the recursive version
        for nb in reversed(list(neighbors(maze, width, height, cell))):
            if nb not in seen:
                stack.append((nb, path + [nb]))
    return [], expanded


# ---------------------------------------------------------------- BFS
def bfs_path(maze, width, height, start, goal):
    """Breadth-first search. Always returns a shortest path on an unweighted grid."""
    queue = deque([start])
    came_from, seen = {}, {start}
    expanded = []
    while queue:
        cell = queue.popleft()
        expanded.append(cell)
        if cell == goal:
            return rebuild(came_from, cell), expanded
        for nb in neighbors(maze, width, height, cell):
            if nb not in seen:
                seen.add(nb)
                came_from[nb] = cell
                queue.append(nb)
    return [], expanded


# ---------------------------------------------------------------- A*
def astar_path(maze, width, height, start, goal):
    """A* with a Manhattan-distance heuristic, the same logic as the race code."""
    open_set = []
    heappush(open_set, (0, start))
    came_from, g = {}, {start: 0}
    expanded = []
    while open_set:
        _, cell = heappop(open_set)
        if cell in expanded:
            continue
        expanded.append(cell)
        if cell == goal:
            return rebuild(came_from, cell), expanded
        for nb in neighbors(maze, width, height, cell):
            cost = g[cell] + 1
            if nb not in g or cost < g[nb]:
                g[nb] = cost
                came_from[nb] = cell
                heappush(open_set, (cost + manhattan(nb, goal), nb))
    return [], expanded


# ---------------------------------------------------------------- plotting
def draw_maze(ax, maze, width, height, path=None, expanded=None, title="", order=None):
    for x in range(width):
        for y in range(height):
            cell = (x, y)
            face = "#f2f2f2"
            if cell not in maze or all(maze[cell].values()):
                face = "#cfcfcf"             # never visited by the drone
            elif expanded and cell in expanded:
                face = "#cfe3f7"
            ax.add_patch(plt.Rectangle((x, y), 1, 1, facecolor=face, edgecolor="none"))
            if cell in maze:
                w = maze[cell]
                if w["N"]: ax.plot([x, x + 1], [y + 1, y + 1], "k", lw=2.2)
                if w["S"]: ax.plot([x, x + 1], [y, y], "k", lw=2.2)
                if w["E"]: ax.plot([x + 1, x + 1], [y, y + 1], "k", lw=2.2)
                if w["W"]: ax.plot([x, x], [y, y + 1], "k", lw=2.2)
    if order:
        for i, (x, y) in enumerate(order, 1):
            ax.text(x + 0.5, y + 0.5, str(i), ha="center", va="center", fontsize=8, color="#1f4e79")
    if path:
        xs = [c[0] + 0.5 for c in path]
        ys = [c[1] + 0.5 for c in path]
        ax.plot(xs, ys, color="#d62728", lw=3, solid_capstyle="round")
        ax.plot(xs[0], ys[0], "o", color="#2ca02c", ms=11)
        ax.plot(xs[-1], ys[-1], "*", color="#ff7f0e", ms=16)
    ax.add_patch(plt.Rectangle((0, 0), width, height, fill=False, edgecolor="#555", lw=1.2, ls="--"))
    ax.set_xlim(-0.05, width + 0.05)
    ax.set_ylim(-0.05, height + 0.05)
    ax.set_aspect("equal")
    ax.axis("off")
    ax.set_title(title, fontsize=10)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--maze", default="maze.json")
    p.add_argument("--start", default="2,0")
    p.add_argument("--goal", default="0,5")
    p.add_argument("--plot", default=None, help="save a comparison figure to this path")
    a = p.parse_args()

    width, height, maze = load_maze(a.maze)
    start = tuple(map(int, a.start.split(",")))
    goal = tuple(map(int, a.goal.split(",")))

    order, moves = dfs_discovery(maze, width, height, start)
    print(f"Maze {width}x{height}, start {start}, goal {goal}")
    print(f"DFS discovery: {len(order)} cells visited, {moves} one-cell moves "
          f"(2 x {len(order) - 1} tree edges, includes physical backtracking)\n")

    results = {}
    for name, fn in (("DFS", dfs_path), ("BFS", bfs_path), ("A*", astar_path)):
        path, expanded = fn(maze, width, height, start, goal)
        results[name] = (path, expanded)
    print(f"{'Algorithm':<10}{'Path cells':>11}{'Moves':>7}{'Cells expanded':>16}{'Shortest?':>11}")
    best = min(len(r[0]) for r in results.values() if r[0])
    for name, (path, expanded) in results.items():
        print(f"{name:<10}{len(path):>11}{max(len(path) - 1, 0):>7}{len(expanded):>16}"
              f"{'yes' if len(path) == best else 'no':>11}")

    if a.plot:
        global plt
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(1, 4, figsize=(17, 4.6))
        draw_maze(axes[0], maze, width, height, None, None, "DFS discovery order\n(cell numbers = visit order)", order)
        for ax, name in zip(axes[1:], ("DFS", "BFS", "A*")):
            path, expanded = results[name]
            draw_maze(ax, maze, width, height, path, set(expanded),
                      f"{name} path: {len(path) - 1} moves, {len(expanded)} cells expanded")
        fig.suptitle("Maze solvers on a 6 x 6 maze discovered by the drone "
                     "(black = wall, grey = never visited, blue = expanded)", fontsize=11)
        fig.tight_layout(rect=[0, 0, 1, 0.93])
        fig.savefig(a.plot, dpi=130)
        print(f"\nFigure saved to {a.plot}")


if __name__ == "__main__":
    main()
