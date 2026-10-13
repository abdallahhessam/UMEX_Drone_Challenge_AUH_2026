"""
UMEX Drone Challenge – Maze Navigation (4 x 5)
Discovery Phase  : DFS (Physical Backtracking)
Racing Phase     : A*
Maze Size        : Width = 4, Height = 5
"""

import pyhula
# import time
import json
from heapq import heappush, heappop

# ===================== CONFIG =====================
DISCOVERY_MODE = True      # True = Discovery | False = Racing
MAZE_WIDTH  = 4            # X: 0..3
MAZE_HEIGHT = 5            # Y: 0..4

CELL_DISTANCE = 60
START = (0, 0)
GOAL  = (0, 4)

FIXED_HEADING = "N"        # Drone never rotates
# =================================================

api = pyhula.UserApi()

# ================= DIRECTIONS =====================
DIRECTIONS = ["N", "E", "S", "W"]
DX = {"N": 0, "E": 1, "S": 0, "W": -1}
DY = {"N": 1, "E": 0, "S": -1, "W": 0}

BACK = {"N": "S", "S": "N", "E": "W", "W": "E"}

# ================= CELL ===========================
class Cell:
    def __init__(self):
        self.walls = {"N": True, "E": True, "S": True, "W": True}
        self.visited = False

maze = {}

# ================= CENTERING ======================
def center_in_cell():
    pos = api.get_coordinate()   # e.g. (x, y, z)

    curr_x = pos[0]
    curr_y = pos[1]
    curr_z = pos[2]

    print(curr_x, curr_y, curr_z)

    # Compute target cell center
    target_x = (curr_x // CELL_DISTANCE) * CELL_DISTANCE + 15
    target_y = (curr_y // CELL_DISTANCE) * CELL_DISTANCE + 15
    target_z = curr_z  # keep current altitude

    # Threshold to avoid jitter
    if abs(target_x - curr_x) > 2 or abs(target_y - curr_y) > 2:
        api.single_fly_straight_flight(target_x, target_y, target_z)

    # time.sleep(1)


def get_cell(x, y):
    if (x, y) not in maze:
        maze[(x, y)] = Cell()
    return maze[(x, y)]

# ================= MOVEMENT =======================
def move_relative(direction):
    if direction == "N":
        api.single_fly_forward(CELL_DISTANCE)
    elif direction == "E":
        api.single_fly_right(CELL_DISTANCE)
    elif direction == "W":
        api.single_fly_left(CELL_DISTANCE)
    elif direction == "S":
        api.single_fly_back(CELL_DISTANCE)

    # time.sleep(2)

    center_in_cell()

# ================= SENSORS ========================
def update_walls(cell):
    b = api.Plane_getBarrier()

    cell.walls["N"] = b["forward"]
    cell.walls["S"] = b["back"]
    cell.walls["W"] = b["left"]
    cell.walls["E"] = b["right"]

# ================= DFS DISCOVERY ==================
def dfs(x, y):
    cell = get_cell(x, y)
    cell.visited = True
    update_walls(cell)

    for d in DIRECTIONS:
        if not cell.walls[d]:
            nx, ny = x + DX[d], y + DY[d]

            if 0 <= nx < MAZE_WIDTH and 0 <= ny < MAZE_HEIGHT:
                neighbor = get_cell(nx, ny)

                if not neighbor.visited:
                    # Move to neighbor
                    move_relative(d)

                    dfs(nx, ny)

                    # Physical backtrack
                    move_relative(BACK[d])

# ================= DEBUG PRINT ====================
def print_maze_debug():
    print("\n====== MAZE DEBUG OUTPUT ======")
    total = MAZE_WIDTH * MAZE_HEIGHT
    visited_count = 0

    for y in range(MAZE_HEIGHT):
        for x in range(MAZE_WIDTH):
            cell_id = y * MAZE_WIDTH + x
            cell = get_cell(x, y)

            if cell.visited:
                visited_count += 1

            print(
                f"Cell {cell_id:02d} | Pos ({x},{y}) | "
                f"Visited: {cell.visited} | "
                f"Walls: N={cell.walls['N']} "
                f"E={cell.walls['E']} "
                f"S={cell.walls['S']} "
                f"W={cell.walls['W']}"
            )

    print(f"\nVisited Cells: {visited_count}/{total}")
    print("================================\n")

# ================= MAZE FILE ======================
def save_maze():
    data = {
        "width": MAZE_WIDTH,
        "height": MAZE_HEIGHT,
        "cells": {
            f"{x},{y}": cell.walls
            for (x, y), cell in maze.items()
        }
    }
    with open("maze.json", "w") as f:
        json.dump(data, f, indent=2)

def load_maze():
    global maze
    with open("maze.json", "r") as f:
        data = json.load(f)

    maze = {}
    for k, walls in data["cells"].items():
        x, y = map(int, k.split(","))
        cell = Cell()
        cell.walls = walls
        maze[(x, y)] = cell

# ================= A* SEARCH ======================
def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def astar(start, goal):
    open_set = []
    heappush(open_set, (0, start))
    came_from = {}
    g = {start: 0}

    while open_set:
        _, current = heappop(open_set)

        if current == goal:
            return reconstruct_path(came_from, current)

        cell = maze[current]
        for d in DIRECTIONS:
            if not cell.walls[d]:
                nx, ny = current[0] + DX[d], current[1] + DY[d]
                if 0 <= nx < MAZE_WIDTH and 0 <= ny < MAZE_HEIGHT:
                    neighbor = (nx, ny)
                    cost = g[current] + 1

                    if neighbor not in g or cost < g[neighbor]:
                        g[neighbor] = cost
                        f = cost + manhattan(neighbor, goal)
                        came_from[neighbor] = current
                        heappush(open_set, (f, neighbor))

def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    return path[::-1]

# ================= RACING EXECUTION ===============
def execute_path(path):
    for i in range(1, len(path)):
        dx = path[i][0] - path[i - 1][0]
        dy = path[i][1] - path[i - 1][1]

        for d in DIRECTIONS:
            if DX[d] == dx and DY[d] == dy:
                move_relative(d)
                break

# ================= MAIN ===========================
def main():
    if not api.connect():
        print("❌ Connection failed")
        return
    # api.Plane_cmd_switch_QR(0)
    # time.sleep(2)
    #
    # api.single_fly_barrier_aircraft(True)
    # time.sleep(2)

    api.single_fly_takeoff()
    # time.sleep(2)

    center_in_cell()

    if DISCOVERY_MODE:
        print("🔍 Discovery Phase (DFS)")
        dfs(START[0], START[1])

        print_maze_debug()
        save_maze()
        print("💾 Maze saved to maze.json")

    else:
        print("🏁 Racing Phase (A*)")
        load_maze()
        path = astar(START, GOAL)
        execute_path(path)

    api.single_fly_touchdown()
    print("🛬 Mission Complete")

# ================= ENTRY ==========================
if __name__ == "__main__":
    main()
