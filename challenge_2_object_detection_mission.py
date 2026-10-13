# ================================================================
# UMEX Drone Challenge 2 – Race Phase (Object Detection in Maze)
# Lateral Movement + Orientation-specific Turning
# ================================================================

import os
import time
import json
import threading
from heapq import heappush, heappop

import cv2
import pyhula
import tkinter as tk
from tkinter import ttk, messagebox

from hula_video import hula_video
from onnxdetector import onnxdetector

# ===================== CONSTANTS =====================
CELL_DISTANCE = 60
SNAPSHOT_DIR = "snapshots"
MAZE_JSON = "maze.json"

DIRECTIONS = ["N", "E", "S", "W"]
DX = {"N": 0, "E": 1, "S": 0, "W": -1}
DY = {"N": 1, "E": 0, "S": -1, "W": 0}

LEFT = {"N": "W", "W": "S", "S": "E", "E": "N"}
RIGHT = {"N": "E", "E": "S", "S": "W", "W": "N"}

# ===================== CLASSES =======================

class DetectedObject:
    def __init__(self, label, position, image_path):
        self.label = label
        self.position = position
        self.image_path = image_path
        self.timestamp = time.time()

class Cell:
    def __init__(self):
        self.walls = {"N": False, "E": False, "S": False, "W": False}

# ===================== GUI ==========================

class RaceGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Drone Challenge 2 – CxI Interface")
        self.root.geometry("900x600")

        self.start_x = tk.IntVar(value=0)
        self.start_y = tk.IntVar(value=0)
        self.bearing = tk.StringVar(value="N")

        self.object_entries = []
        self.on_start_callback = None

        self._build_ui()

    def _build_ui(self):
        input_frame = tk.LabelFrame(self.root, text="Mission Inputs")
        input_frame.pack(side=tk.LEFT, fill=tk.Y, padx=10, pady=10)

        tk.Label(input_frame, text="Start X").grid(row=0, column=0)
        tk.Entry(input_frame, textvariable=self.start_x, width=5).grid(row=0, column=1)

        tk.Label(input_frame, text="Start Y").grid(row=1, column=0)
        tk.Entry(input_frame, textvariable=self.start_y, width=5).grid(row=1, column=1)

        tk.Label(input_frame, text="Initial Bearing").grid(row=2, column=0)
        ttk.Combobox(input_frame, values=DIRECTIONS, textvariable=self.bearing, width=5).grid(row=2, column=1)

        tk.Label(input_frame, text="Object Targets (x, y, look direction)").grid(row=3, column=0, columnspan=2, pady=(10, 0))

        self.obj_frame = tk.Frame(input_frame)
        self.obj_frame.grid(row=4, column=0, columnspan=2)

        tk.Button(input_frame, text="Add Object", command=self.add_object_row).grid(row=5, column=0, columnspan=2, pady=5)

        tk.Button(input_frame, text="Start Mission", bg="#4CAF50", fg="white", command=self._start_pressed).grid(row=6, column=0, columnspan=2, pady=10)

        output_frame = tk.LabelFrame(self.root, text="Detected Objects")
        output_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10, pady=10)

        self.detected_list = tk.Listbox(output_frame, height=10)
        self.detected_list.pack(fill=tk.X, padx=5, pady=5)

        self.snapshot_label = tk.Label(output_frame)
        self.snapshot_label.pack(pady=10)

        self.add_object_row()

    def add_object_row(self):
        r = len(self.object_entries)
        ex = tk.Entry(self.obj_frame, width=4)
        ey = tk.Entry(self.obj_frame, width=4)
        ed = ttk.Combobox(self.obj_frame, values=DIRECTIONS, width=3)
        ed.set("N")
        ex.grid(row=r, column=0); ey.grid(row=r, column=1); ed.grid(row=r, column=2)
        self.object_entries.append((ex, ey, ed))

    def _start_pressed(self):
        try:
            targets = {}
            for ex, ey, ed in self.object_entries:
                if ex.get() and ey.get():
                    targets[(int(ex.get()), int(ey.get()))] = ed.get()

            config = {"start": (self.start_x.get(), self.start_y.get()), "bearing": self.bearing.get(), "targets": targets}
            threading.Thread(target=self.on_start_callback, args=(config,), daemon=True).start()
        except Exception as e:
            messagebox.showerror("Input Error", str(e))

    def add_detected(self, det_obj):
        self.detected_list.insert(tk.END, f"{det_obj.label} at {det_obj.position}")
        img = cv2.imread(det_obj.image_path)
        img = cv2.resize(img, (200, 200))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        photo = tk.PhotoImage(master=self.root, data=cv2.imencode(".png", img)[1].tobytes())
        self.snapshot_label.configure(image=photo)
        self.snapshot_label.image = photo

    def run(self):
        self.root.mainloop()

# ===================== VISION ======================

class VisionSystem:
    def __init__(self, api):
        self.vid = hula_video(hula_api=api, display=False)
        self.detector = onnxdetector(model="detect_3_object_12_11.onnx", label="object.txt", confidence_thres=0.3)

    def start(self):
        self.vid.video_mode_on()
        self.vid.startrecording()

    def stop(self):
        self.vid.stoprecording()
        self.vid.close()

    def detect(self):
        frame = self.vid.get_video()
        return self.detector.detect(frame)

# ===================== NAVIGATION ==================

class Navigator:
    def __init__(self, json_path):
        with open(json_path, "r") as f:
            data = json.load(f)
        self.w, self.h = data["width"], data["height"]
        self.maze = {(x, y): Cell() for x in range(self.w) for y in range(self.h)}
        for key, walls in data["cells"].items():
            x, y = map(int, key.split(","))
            for d in DIRECTIONS:
                self.maze[(x, y)].walls[d] = walls[d]

    def manhattan(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def astar(self, start, goal):
        open_set = []
        heappush(open_set, (0, start))
        came, g = {}, {start: 0}
        while open_set:
            _, cur = heappop(open_set)
            if cur == goal: return self.reconstruct(came, cur)
            for d in DIRECTIONS:
                if self.maze[cur].walls[d]: continue
                nx, ny = cur[0] + DX[d], cur[1] + DY[d]
                if 0 <= nx < self.w and 0 <= ny < self.h:
                    nb = (nx, ny)
                    cost = g[cur] + 1
                    if nb not in g or cost < g[nb]:
                        g[nb] = cost
                        came[nb] = cur
                        heappush(open_set, (cost + self.manhattan(nb, goal), nb))
        return []

    def reconstruct(self, came, cur):
        path = [cur]
        while cur in came:
            cur = came[cur]
            path.append(cur)
        return path[::-1]

# ===================== MISSION =====================

class MissionController:
    def __init__(self, gui):
        self.gui = gui
        self.api = pyhula.UserApi()
        self.navigator = Navigator(MAZE_JSON)
        self.vision = None
        self.detected = {}
        self.bearing = "N"

    def turn_to(self, target):
        """Rotates the drone to face a global direction."""
        while self.bearing != target:
            if RIGHT[self.bearing] == target:
                self.api.single_fly_turnright(90)
                self.bearing = RIGHT[self.bearing]
            elif LEFT[self.bearing] == target:
                self.api.single_fly_turnleft(90)
                self.bearing = LEFT[self.bearing]
            else: # 180 degrees
                self.api.single_fly_turnright(180)
                self.bearing = RIGHT[RIGHT[self.bearing]]
            time.sleep(1.5)

    def move_relative(self, target_global_dir):
        """Moves the drone relative to its current orientation using strafing."""
        # Map: current_bearing -> { desired_global_step: drone_command }
        rel_map = {
            "N": {"N": "fwd",   "S": "back",  "E": "right", "W": "left"},
            "E": {"N": "left",  "S": "right", "E": "fwd",   "W": "back"},
            "S": {"N": "back",  "S": "fwd",   "E": "left",  "W": "right"},
            "W": {"N": "right", "S": "left",  "E": "back",  "W": "fwd"}
        }

        cmd = rel_map[self.bearing][target_global_dir]

        if cmd == "fwd":   self.api.single_fly_forward(CELL_DISTANCE)
        elif cmd == "back": self.api.single_fly_back(CELL_DISTANCE)
        elif cmd == "right": self.api.single_fly_right(CELL_DISTANCE)
        elif cmd == "left":  self.api.single_fly_left(CELL_DISTANCE)

        time.sleep(1)

    def detect_here(self, cell, look_dir):
        self.turn_to(look_dir)
        # obj_dict is the 'highest_score_obj' dictionary from your onnxdetector
        obj_dict, frame = self.vision.detect()

        # 1. Check if the dictionary exists (meaning something was detected)
        if obj_dict is not None:
            # 2. Extract the string label from the dictionary
            label = obj_dict.get('label')

            # 3. Use the string label for hashing/comparison
            if label and label not in self.detected:
                os.makedirs(SNAPSHOT_DIR, exist_ok=True)
                path = f"{SNAPSHOT_DIR}/{cell[0]}_{cell[1]}_{label}.jpg"
                cv2.imwrite(path, frame)

                det = DetectedObject(label, cell, path)
                self.detected[label] = det
                self.gui.add_detected(det)

    def execute_path(self, path, look_dir):
        for i in range(1, len(path)):
            dx, dy = path[i][0] - path[i-1][0], path[i][1] - path[i-1][1]
            for d in DIRECTIONS:
                if DX[d] == dx and DY[d] == dy:
                    self.move_relative(d)
                    break
        self.detect_here(path[-1], look_dir)

    def run_mission(self, config):
        if not self.api.connect():
            print("❌ Connection failed"); return

        self.bearing = config["bearing"]
        self.api.Plane_cmd_switch_QR(0)
        time.sleep(2)  # Allow time for vision system to initialize

        self.api.single_fly_takeoff()
        time.sleep(2)

        self.api.single_fly_straight_flight(15, 15, 100)
        time.sleep(2)  # Hover for observation

        self.vision = VisionSystem(self.api)
        self.vision.start()

        current = config["start"]
        targets = dict(config["targets"])

        while targets:
            tgt = min(targets, key=lambda t: self.navigator.manhattan(current, t))
            path = self.navigator.astar(current, tgt)
            if not path:
                targets.pop(tgt); continue

            self.execute_path(path, targets[tgt])
            current = tgt
            targets.pop(tgt)

        self.vision.stop()
        self.api.single_fly_touchdown()
        print("✅ Mission complete")

# ===================== MAIN ========================

if __name__ == "__main__":
    gui = RaceGUI()
    controller = MissionController(gui)
    gui.on_start_callback = controller.run_mission
    gui.run()