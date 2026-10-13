# ================================================================
# UMEX Drone Challenge 2 – Race Phase (Object Detection in Maze)
# Single integrated script: GUI + Navigation + Vision + A*
# ================================================================
# Requirements:
# - pyhula
# - hula_video
# - onnxdetector
# - opencv-python
# - tkinter (standard)
# ================================================================

import os
import time
import json
import math
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

DIRECTIONS = ["N", "E", "S", "W"]
DX = {"N": 0, "E": 1, "S": 0, "W": -1}
DY = {"N": 1, "E": 0, "S": -1, "W": 0}

BACK = {"N": "S", "S": "N", "E": "W", "W": "E"}

# ====================================================

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

        self.maze_size = tk.IntVar(value=5)
        self.start_x = tk.IntVar(value=0)
        self.start_y = tk.IntVar(value=0)
        self.bearing = tk.StringVar(value="N")

        self.object_entries = []
        self.on_start_callback = None

        self._build_ui()

    def _build_ui(self):
        input_frame = tk.LabelFrame(self.root, text="Mission Inputs")
        input_frame.pack(side=tk.LEFT, fill=tk.Y, padx=10, pady=10)

        tk.Label(input_frame, text="Maze Size (N x N)").grid(row=0, column=0)
        tk.Entry(input_frame, textvariable=self.maze_size, width=5).grid(row=0, column=1)

        tk.Label(input_frame, text="Start X").grid(row=1, column=0)
        tk.Entry(input_frame, textvariable=self.start_x, width=5).grid(row=1, column=1)

        tk.Label(input_frame, text="Start Y").grid(row=2, column=0)
        tk.Entry(input_frame, textvariable=self.start_y, width=5).grid(row=2, column=1)

        tk.Label(input_frame, text="Initial Bearing").grid(row=3, column=0)
        ttk.Combobox(input_frame, values=["N","E","S","W"], textvariable=self.bearing, width=5).grid(row=3, column=1)

        tk.Label(input_frame, text="Object Locations (x,y,label)").grid(row=4, column=0, columnspan=2, pady=(10,0))

        self.obj_frame = tk.Frame(input_frame)
        self.obj_frame.grid(row=5, column=0, columnspan=2)

        tk.Button(input_frame, text="Add Object", command=self.add_object_row).grid(row=6, column=0, columnspan=2, pady=5)

        tk.Button(input_frame, text="Start Mission", bg="#4CAF50", fg="white",
                  command=self._start_pressed).grid(row=7, column=0, columnspan=2, pady=10)

        output_frame = tk.LabelFrame(self.root, text="Detected Objects")
        output_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10, pady=10)

        self.detected_list = tk.Listbox(output_frame, height=10)
        self.detected_list.pack(fill=tk.X, padx=5, pady=5)

        self.snapshot_label = tk.Label(output_frame)
        self.snapshot_label.pack(pady=10)

        self.add_object_row()

    def add_object_row(self):
        row = len(self.object_entries)
        ex = tk.Entry(self.obj_frame, width=4)
        ey = tk.Entry(self.obj_frame, width=4)
        el = tk.Entry(self.obj_frame, width=8)
        ex.grid(row=row, column=0)
        ey.grid(row=row, column=1)
        el.grid(row=row, column=2)
        self.object_entries.append((ex, ey, el))

    def _start_pressed(self):
        try:
            obj_locations = {}
            for ex, ey, el in self.object_entries:
                if ex.get() and ey.get() and el.get():
                    obj_locations[(int(ex.get()), int(ey.get()))] = el.get()

            config = {
                "size": self.maze_size.get(),
                "start": (self.start_x.get(), self.start_y.get()),
                "bearing": self.bearing.get(),
                "objects": obj_locations
            }

            if self.on_start_callback:
                threading.Thread(target=self.on_start_callback, args=(config,), daemon=True).start()

        except Exception as e:
            messagebox.showerror("Input Error", str(e))

    def add_detected(self, det_obj: DetectedObject):
        self.detected_list.insert(tk.END, f"{det_obj.label} at {det_obj.position}")
        img = cv2.imread(det_obj.image_path)
        img = cv2.resize(img, (200, 200))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        photo = tk.PhotoImage(master=self.root, data=cv2.imencode('.png', img)[1].tobytes())
        self.snapshot_label.configure(image=photo)
        self.snapshot_label.image = photo

    def run(self):
        self.root.mainloop()


# ===================== VISION ======================

class VisionSystem:
    def __init__(self, api):
        self.vid = hula_video(hula_api=api, display=False)
        self.detector = onnxdetector(model="detect_3_object_12_11.onnx",
                                     label="object.txt",
                                     confidence_thres=0.3)

    def start(self):
        self.vid.video_mode_on()
        self.vid.startrecording()

    def stop(self):
        self.vid.stoprecording()
        self.vid.close()

    def detect(self):
        frame = self.vid.get_video()
        obj, frame = self.detector.detect(frame)
        return obj, frame


# ===================== NAVIGATION ==================

class Navigator:
    def __init__(self, maze_size):
        self.w = maze_size
        self.h = maze_size
        self.maze = {(x,y): Cell() for x in range(self.w) for y in range(self.h)}

    def manhattan(self, a, b):
        return abs(a[0]-b[0]) + abs(a[1]-b[1])

    def astar(self, start, goal):
        open_set = []
        heappush(open_set, (0, start))
        came = {}
        g = {start: 0}

        while open_set:
            _, cur = heappop(open_set)
            if cur == goal:
                return self.reconstruct(came, cur)

            for d in DIRECTIONS:
                nx, ny = cur[0] + DX[d], cur[1] + DY[d]
                if 0 <= nx < self.w and 0 <= ny < self.h:
                    nb = (nx, ny)
                    cost = g[cur] + 1
                    if nb not in g or cost < g[nb]:
                        g[nb] = cost
                        f = cost + self.manhattan(nb, goal)
                        came[nb] = cur
                        heappush(open_set, (f, nb))

        return []

    def reconstruct(self, came, cur):
        path = [cur]
        while cur in came:
            cur = came[cur]
            path.append(cur)
        return path[::-1]


# ===================== MISSION =====================

class MissionController:
    def __init__(self, gui: RaceGUI):
        self.gui = gui
        self.api = pyhula.UserApi()
        self.vision = None
        self.navigator = None
        self.detected = {}

    def center_in_cell(self):
        pos = self.api.get_coordinate()
        tx = (pos[0] // CELL_DISTANCE) * CELL_DISTANCE + 15
        ty = (pos[1] // CELL_DISTANCE) * CELL_DISTANCE + 15
        tz = pos[2]
        self.api.single_fly_straight_flight(tx, ty, tz)

    def move_relative(self, direction):
        if direction == "N": self.api.single_fly_forward(CELL_DISTANCE)
        if direction == "S": self.api.single_fly_back(CELL_DISTANCE)
        if direction == "E": self.api.single_fly_right(CELL_DISTANCE)
        if direction == "W": self.api.single_fly_left(CELL_DISTANCE)
        time.sleep(0.5)
        self.center_in_cell()

    def detect_here(self, cell_pos):
        obj, frame = self.vision.detect()
        if obj and cell_pos not in self.detected:
            if not os.path.exists(SNAPSHOT_DIR):
                os.makedirs(SNAPSHOT_DIR)
            path = f"{SNAPSHOT_DIR}/{cell_pos[0]}_{cell_pos[1]}_{obj}.jpg"
            cv2.imwrite(path, frame)

            det = DetectedObject(obj, cell_pos, path)
            self.detected[cell_pos] = det
            self.gui.add_detected(det)

    def execute_path(self, path):
        for i in range(1, len(path)):
            dx = path[i][0] - path[i-1][0]
            dy = path[i][1] - path[i-1][1]
            for d in DIRECTIONS:
                if DX[d] == dx and DY[d] == dy:
                    self.move_relative(d)
                    self.detect_here(path[i])
                    break

    def run_mission(self, config):
        size = config["size"]
        start = config["start"]
        objects = config["objects"]

        self.navigator = Navigator(size)

        if not self.api.connect():
            print("Connection failed")
            return

        self.api.single_fly_takeoff()
        self.center_in_cell()

        self.vision = VisionSystem(self.api)
        self.vision.start()

        current = start
        targets = list(objects.keys())

        while targets:
            tgt = min(targets, key=lambda t: abs(t[0]-current[0])+abs(t[1]-current[1]))
            path = self.navigator.astar(current, tgt)
            self.execute_path(path)
            current = tgt
            targets.remove(tgt)

        self.vision.stop()
        self.api.single_fly_touchdown()
        print("Mission complete")


# ===================== MAIN ========================

if __name__ == "__main__":
    gui = RaceGUI()
    controller = MissionController(gui)
    gui.on_start_callback = controller.run_mission
    gui.run()
