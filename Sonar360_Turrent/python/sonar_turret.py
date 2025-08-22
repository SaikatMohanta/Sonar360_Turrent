# sonar_turret.py
import serial
import json
import time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from sklearn.cluster import DBSCAN
from collections import deque
import threading

class SonarTurret:
    """
    SonarTurret: A 360° ultrasonic-based surveillance system
    - Reads sensor data from Arduino
    - Adjusts distances using temperature & humidity
    - Tracks objects in real-time (velocity & direction)
    - Clusters objects using DBSCAN
    - Provides live visualization (2D or 3D)
    """

    def __init__(self, port='COM3', baud_rate=115200, history_length=20, visualization="3D"):
        self.port = port
        self.baud_rate = baud_rate
        self.serial_conn = None

        # Sensor positions (equilateral triangle, 9 cm side)
        self.sensor_positions = {
            'A': (0, 0, 0),
            'B': (9, 0, 0),
            'C': (-4.5, 7.79, 0)
        }

        # Sensor angles in radians
        self.sensor_angles = {
            'A': 4*np.pi/3,  # 240°
            'B': 0,          # 0°
            'C': 2*np.pi/3   # 120°
        }

        # Data storage
        self.history_length = history_length
        self.data_history = deque(maxlen=history_length)
        self.point_history = {}
        self.objects = {}

        # Clustering
        self.clustering = DBSCAN(eps=15, min_samples=3)

        # Visualization
        self.visualization = visualization
        self.fig, self.ax = None, None
        self.scatter, self.text_elements, self.vector_arrows = None, [], []

        # Threading
        self.running = False
        self.data_thread = None

    def connect(self):
        """Connect to Arduino via serial port"""
        try:
            self.serial_conn = serial.Serial(self.port, self.baud_rate, timeout=1)
            print(f"Connected to {self.port} at {self.baud_rate} baud")
            time.sleep(2)  # let Arduino reset
            return True
        except Exception as e:
            print(f"[ERROR] Serial connection failed: {e}")
            return False

    def read_serial_data(self):
        """Continuously read and parse JSON data from Arduino"""
        if not self.serial_conn:
            print("[ERROR] Serial connection not established")
            return

        buffer, reading = "", False
        while self.running:
            try:
                line = self.serial_conn.readline().decode('utf-8').strip()
                if line == "<START>":
                    reading, buffer = True, ""
                elif line == "<END>":
                    reading = False
                    try:
                        data = json.loads(buffer)
                        self.process_data(data)
                    except json.JSONDecodeError:
                        print(f"[WARN] Bad JSON: {buffer}")
                elif reading:
                    buffer += line
            except Exception as e:
                print(f"[ERROR] Reading serial: {e}")
                time.sleep(0.1)

    def process_data(self, data):
        """Convert raw Arduino data into world coordinates + tracking"""
        temp, humidity = data['temp'], data['humidity']
        speed_of_sound = 331.4 + (0.6 * temp) + (0.0124 * humidity)

        # Prepare processed data
        processed = {
            'timestamp': time.time(),
            'temp': temp,
            'humidity': humidity,
            'speed_of_sound': speed_of_sound,
            'sensors': data['sensors'],
            'points': self.calculate_points(data['sensors'])
        }
        self.data_history.append(processed)
        self.update_point_tracking(processed)

        if len(self.data_history) >= 5:
            self.perform_clustering()

    def calculate_points(self, sensor_data):
        """Convert sensor readings into Cartesian coordinates"""
        points = []
        for sensor_id, distance in sensor_data.items():
            if distance > 0:  # valid reading
                angle = self.sensor_angles[sensor_id]
                pos = self.sensor_positions[sensor_id]
                x = pos[0] + distance * np.cos(angle)
                y = pos[1] + distance * np.sin(angle)
                z = 0
                points.append({'sensor': sensor_id, 'distance': distance, 'position': (x, y, z)})
        return points

    def update_point_tracking(self, processed):
        """Track points over time and estimate velocity/direction"""
        now = processed['timestamp']
        for pt in processed['points']:
            pos = pt['position']
            closest_id, min_dist = None, 15
            for pid, pdata in self.point_history.items():
                last_pos = pdata['positions'][-1]
                dist = np.linalg.norm(np.array(pos) - np.array(last_pos))
                if dist < min_dist:
                    min_dist, closest_id = dist, pid
            if closest_id:
                pdata = self.point_history[closest_id]
                pdata['positions'].append(pos)
                pdata['timestamps'].append(now)
                if len(pdata['positions']) >= 2:
                    dt = pdata['timestamps'][-1] - pdata['timestamps'][-2]
                    if dt > 0:
                        dx, dy, dz = np.array(pdata['positions'][-1]) - np.array(pdata['positions'][-2])
                        v = np.linalg.norm([dx, dy, dz]) / dt
                        pdata['velocity'], pdata['direction'] = v, (dx/dt, dy/dt, dz/dt)
            else:
                new_id = f"P{len(self.point_history)}"
                self.point_history[new_id] = {
                    'positions': [pos], 'timestamps': [now],
                    'velocity': 0, 'direction': (0, 0, 0)
                }
        # cleanup old points (>5s)
        self.point_history = {k: v for k, v in self.point_history.items() if now - v['timestamps'][-1] <= 5}

    def perform_clustering(self):
        """Cluster points into objects using DBSCAN"""
        positions, ids = [], []
        for pid, pdata in self.point_history.items():
            positions.append(pdata['positions'][-1])
            ids.append(pid)
        if len(positions) < 3:
            return
        clusters = self.clustering.fit_predict(np.array(positions))
        new_objects = {}
        for i, cid in enumerate(clusters):
            if cid == -1: continue
            if cid not in new_objects:
                new_objects[cid] = {'points': [], 'center': None, 'velocity': 0, 'is_moving': False}
            pt = self.point_history[ids[i]]
            new_objects[cid]['points'].append(pt)
        for cid, obj in new_objects.items():
            obj['center'] = np.mean([p['positions'][-1] for p in obj['points']], axis=0)
            vels = [p['velocity'] for p in obj['points']]
            obj['velocity'] = np.mean(vels)
            obj['is_moving'] = obj['velocity'] > 5
        self.objects = new_objects

    def setup_visualization(self):
        """Initialize Matplotlib visualization"""
        self.fig = plt.figure(figsize=(10, 8))
        if self.visualization == "3D":
            self.ax = self.fig.add_subplot(111, projection='3d')
            self.ax.set_zlim(0, 100)
            self.ax.set_zlabel('Z (cm)')
        else:
            self.ax = self.fig.add_subplot(111)
        self.ax.set_xlim(-100, 100)
        self.ax.set_ylim(-100, 100)
        self.ax.set_xlabel('X (cm)')
        self.ax.set_ylabel('Y (cm)')
        self.ax.set_title('Sonar Turret Visualization')
        self.scatter = self.ax.scatter([], [], [])
        return self.fig

    def update_visualization(self, frame):
        """Update visualization with tracked objects"""
        if not self.objects: return self.scatter,
        positions, colors = [], []
        for oid, obj in self.objects.items():
            c = obj['center']
            positions.append(c)
            colors.append(1.0 if obj['is_moving'] else 0.0)
        if positions:
            arr = np.array(positions)
            if self.visualization == "3D":
                self.scatter._offsets3d = (arr[:,0], arr[:,1], arr[:,2])
            else:
                self.scatter.set_offsets(arr[:, :2])
            self.scatter.set_array(np.array(colors))
        return self.scatter,

    def start(self):
        """Start sonar system (threads + visualization)"""
        if self.connect():
            self.running = True
            self.data_thread = threading.Thread(target=self.read_serial_data, daemon=True)
            self.data_thread.start()
            ani = FuncAnimation(self.setup_visualization(), self.update_visualization, interval=100, blit=True)
            plt.show()

    def stop(self):
        """Stop sonar system"""
        self.running = False
        if self.serial_conn:
            self.serial_conn.close()
        print("System stopped")
