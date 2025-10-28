import tkinter as tk
from tkinter import messagebox
import socket
import threading
import time

# Camera / vision stack
import cv2
import urllib.request
import numpy as np

# For WLAN IP detection
import psutil

# YOLO
import torch  # noqa: F401 (ultralytics imports torch internally; keeping explicit to ensure install)
from ultralytics import YOLO

# ---- Load your trained YOLO model (keep your file name) ----
MODEL_PATH = "best_15schilder.pt"   # trained by Patrick
model = YOLO(MODEL_PATH)

DEFAULT_PORT = 5001
DEFAULT_AX = 10
DEFAULT_AY = 30
MAX_ABS = 250

class RobotControlGUI:
    """
    ESP32 connects to this app (TCP server).
    Use arrow keys or buttons; YOLO can steer/stop based on detections.
    """

    def __init__(self, root):
        self.root = root
        self.root.title("Robot Gamepad + YOLO")
        self.root.geometry("460x520")
        self.root.configure(bg="#2c3e50")

        # --- Networking / control state ---
        self.server_sock = None
        self.conn = None
        self.addr = None
        self.port = DEFAULT_PORT

        # Detect your WLAN IPv4 (used for display only)
        self.detected_ip = self.get_wlan_ip() or "0.0.0.0"

        self.status = tk.StringVar(value="Waiting for ESP32…")
        self.server_info = tk.StringVar(value=f"Server binding: 0.0.0.0:{self.port}  |  Your IP: {self.detected_ip}")

        self.vy = 0
        self.vx = 0
        self.ay = DEFAULT_AY
        self.ax = DEFAULT_AX

        self.framecount = 0
        self.results = []

        # default camera IP (XIAO ESP32S3 cam, '/capture' endpoint)
        self.cameraip = "192.168.1.201"
        self.cameraurl = None

        # ------------- UI -------------
        info = tk.Label(root, textvariable=self.server_info, font=("Helvetica", 10), bg="#2c3e50", fg="#ecf0f1")
        info.pack(pady=(8, 0))

        self.status_label = tk.Label(root, textvariable=self.status, font=("Helvetica", 14, "bold"),
                                     bg="#2c3e50", fg="#ecf0f1")
        self.status_label.pack(pady=6)

        # Camera controls
        row_cam = tk.Frame(root, bg="#2c3e50")
        row_cam.pack(pady=(6, 0))
        tk.Label(row_cam, text="Camera IP:", bg="#2c3e50", fg="#ecf0f1").grid(row=0, column=0, padx=4)
        self.entry_ip = tk.Entry(row_cam, width=16)
        self.entry_ip.insert(0, self.cameraip)
        self.entry_ip.grid(row=0, column=1)
        tk.Button(row_cam, text="Start Video", command=self.start_video,
                  bg="#3498db", fg="white", activebackground="#2980b9").grid(row=0, column=2, padx=6)

        # Controller frame
        button_config = {"font": ("Helvetica", 12, "bold"), "width": 5, "height": 2}
        controller_frame = tk.Frame(root, bg="#34495e", bd=5, relief="ridge")
        controller_frame.pack(pady=12, padx=12)

        tk.Button(controller_frame, text="↑", command=self.drivy_up,
                  bg="#3498db", fg="white", activebackground="#2980b9", **button_config
                  ).grid(row=0, column=1, pady=(0, 5))
        tk.Button(controller_frame, text="←", command=self.drivy_left,
                  bg="#3498db", fg="white", activebackground="#2980b9", **button_config
                  ).grid(row=1, column=0, padx=(0, 5))
        tk.Button(controller_frame, text="→", command=self.drivy_right,
                  bg="#3498db", fg="white", activebackground="#2980b9", **button_config
                  ).grid(row=1, column=2, padx=(5, 0))
        tk.Button(controller_frame, text="↓", command=self.drivy_down,
                  bg="#3498db", fg="white", activebackground="#2980b9", **button_config
                  ).grid(row=2, column=1, pady=(5, 0))
        tk.Button(controller_frame, text="Stop", command=self.drivy_stop,
                  bg="#e67e22", fg="white", activebackground="#d35400", **button_config
                  ).grid(row=1, column=1, pady=(5, 0))

        # Key bindings
        self.root.bind("<Up>", self.button_up)
        self.root.bind("<Down>", self.button_down)
        self.root.bind("<Left>", self.button_left)
        self.root.bind("<Right>", self.button_right)
        self.root.bind("<space>", self.button_space)
        self.root.bind("s", self.button_stop)

        # Start socket server (threaded, autoreconnect)
        self.start_socket()

    # ---------- IP detection ----------
    def get_wlan_ip(self):
        """Find WLAN IPv4 address of this machine."""
        for name, addrs in psutil.net_if_addrs().items():
            if "wl" in name.lower() or "wlan" in name.lower() or "wi-fi" in name.lower():
                for addr in addrs:
                    if addr.family == socket.AF_INET:
                        return addr.address
        # Fallback: pick first non-loopback IPv4
        for name, addrs in psutil.net_if_addrs().items():
            for addr in addrs:
                if addr.family == socket.AF_INET and not addr.address.startswith("127."):
                    return addr.address
        return None

    # ---------- Camera / YOLO ----------
    def start_video(self):
        self.cameraip = self.entry_ip.get().strip()
        self.cameraurl = f"http://{self.cameraip}/capture"
        # Create window once
        cv2.namedWindow("Camera Test", cv2.WINDOW_AUTOSIZE)
        self.run_video()

    def run_video(self):
        try:
            # Pull single JPEG frame
            img_resp = urllib.request.urlopen(self.cameraurl, timeout=2.0)
            imgnp = np.frombuffer(img_resp.read(), dtype=np.uint8)
            frame = cv2.imdecode(imgnp, cv2.IMREAD_COLOR)
            if frame is None:
                raise ValueError("cv2.imdecode returned None")

            # YOLO every 3rd frame to save CPU
            if self.framecount % 3 == 0:
                self.results = model(frame, verbose=False)

            if self.framecount > 3 and self.results:
                for result in self.results:
                    for box in result.boxes:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        conf = float(box.conf[0].item())
                        cls = int(box.cls[0].item())
                        label = f"{model.names[cls]}: {conf:.2f}"

                        # Draw
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.putText(frame, label, (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                        # Simple reactions
                        if conf > 0.6 and model.names[cls] == "Stop":
                            self.drivy_stop()
                        elif conf > 0.6 and model.names[cls] == "Speed Limit 100":
                            self.vy = MAX_ABS if self.vy >= 0 else 0
                            self.send_command(self.vx, self.vy)
                        elif conf > 0.6 and model.names[cls] == "Speed Limit 50":
                            self.vy = 125 if self.vy >= 0 else 0
                            self.send_command(self.vx, self.vy)
                            xmean = (x1 + x2) / 2
                            # steer roughly to center
                            if xmean < frame.shape[1] / 2 - 50:
                                self.drivy_left()
                            elif xmean > frame.shape[1] / 2 + 50:
                                self.drivy_right()
                            else:
                                self.vx = 0
                                self.vy = 125
                                self.send_command(self.vx, self.vy)

            cv2.imshow("Camera Test", frame)
            cv2.waitKey(1)
            self.framecount += 1

        except Exception as e:
            # Don’t crash on camera hiccups; show status once
            self.status.set(f"Camera error: {e}")

        # schedule next frame
        self.root.after(40, self.run_video)

    # ---------- Buttons / Keys ----------
    def button_up(self, _):     self.drivy_up()
    def button_down(self, _):   self.drivy_down()
    def button_left(self, _):   self.drivy_left()
    def button_right(self, _):  self.drivy_right()
    def button_space(self, _):  self.drivy_stop()
    def button_stop(self, _):   self.drivy_stop()

    def drivy_up(self):
        self.vy = min(MAX_ABS, max(0, self.vy + self.ay))
        self.send_command(self.vx, self.vy)

    def drivy_down(self):
        self.vy = max(-MAX_ABS, min(0, self.vy - self.ay))
        self.send_command(self.vx, self.vy)

    def drivy_left(self):
        self.vx = max(-MAX_ABS, min(0, self.vx - self.ax))
        self.send_command(self.vx, self.vy)

    def drivy_right(self):
        self.vx = min(MAX_ABS, max(0, self.vx + self.ax))
        self.send_command(self.vx, self.vy)

    def drivy_stop(self):
        self.vx = 0
        self.vy = 0
        self.send_command(self.vx, self.vy)

    # ---------- Socket server (ESP32 connects here) ----------
    def start_socket(self):
        t = threading.Thread(target=self.accept_loop, daemon=True)
        t.start()

    def accept_loop(self):
        # Bind on all interfaces so ESP32 can reach us via your WLAN IP
        bind_ip = "0.0.0.0"
        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_sock.bind((bind_ip, self.port))
        self.server_sock.listen(1)

        self.server_info.set(f"Server binding: {bind_ip}:{self.port}  |  Your IP: {self.detected_ip}")
        self.status.set("Waiting for ESP32…")

        while True:
            try:
                conn, addr = self.server_sock.accept()
                conn.setblocking(True)
                self.conn, self.addr = conn, addr
                self.status.set(f"Connected: {addr[0]}:{addr[1]}")
                # Immediately send a STOP to be safe
                self.safe_send("0 0\n")
                # Block here until client closes, then loop to accept again
                self.read_discard_loop(conn)
            except Exception as e:
                self.status.set(f"Socket error: {e}")
                time.sleep(0.5)

    def read_discard_loop(self, conn):
        try:
            while True:
                # We don't really expect incoming data; just detect disconnect.
                data = conn.recv(128)
                if not data:
                    break
        except Exception:
            pass
        finally:
            try:
                conn.close()
            except Exception:
                pass
            self.conn = None
            self.addr = None
            self.status.set("Disconnected. Waiting for ESP32…")

    def safe_send(self, line: str):
        if self.conn is None:
            return
        try:
            self.conn.sendall(line.encode("utf-8"))
        except Exception:
            # drop the dead connection and wait for re-accept
            try:
                self.conn.close()
            except Exception:
                pass
            self.conn = None
            self.addr = None
            self.status.set("Disconnected. Waiting for ESP32…")

    def send_command(self, vx, vy):
        # clip and send "vx vy\n"
        vx = max(-MAX_ABS, min(MAX_ABS, int(vx)))
        vy = max(-MAX_ABS, min(MAX_ABS, int(vy)))
        self.safe_send(f"{vx} {vy}\n")

    def on_closing(self):
        cv2.destroyAllWindows()
        try:
            if self.conn:
                self.conn.close()
            if self.server_sock:
                self.server_sock.close()
        except Exception:
            pass
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = RobotControlGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()
