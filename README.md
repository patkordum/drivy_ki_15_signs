# Drivy Robot Control with YOLO and ESP32

A Python–ESP32 robot control system using a socket connection and YOLO object detection.  
The Python GUI (`drivy_ki_stop_100_50.py`) communicates with an ESP32-based car over WiFi and reacts automatically to detected traffic signs such as **Stop**, **Speed Limit 50**, and **Speed Limit 100**.

---

## Project Overview

This project enables an autonomous or manually controlled robot car that uses a trained YOLO model for real-time traffic-sign detection and adapts its movement accordingly.  
The ESP32 controls the motors and LEDs, while the Python application runs YOLOv8 detection and sends steering commands over a TCP socket.

---

## File Structure

```
├── drivy_socket_client_pin4high/
│   └── drivy_socket_client_pin4high.ino      # ESP32 firmware
├── drivy_ki_stop_100_50.py                   # Python GUI and YOLO detection
├── best_15schilder.pt                        # Trained YOLOv8 model (15 traffic signs)
├── requirements.txt                          # Python dependencies
```

---

## Hardware and Software Requirements

**Hardware**
- ESP32 DevKit V1 or XIAO ESP32-S3 Sense
- Motor driver (e.g., L298N or TB6612)
- 4 DC motors
- Optional LEDs for front/rear lights
- Optional camera (ESP32-CAM or ESP32-S3 Sense providing `/capture` endpoint)

**Software**
- Python 3.10 or newer
- Arduino IDE with ESP32 board support
- YOLOv8 model trained via [Ultralytics](https://github.com/ultralytics/ultralytics)

---

## Features

- Tkinter-based GUI for directional control (arrow keys or buttons)
- Live video stream with YOLOv8 detection
- Automatic stop and speed adaptation depending on detected signs
- Real-time TCP communication between Python server and ESP32 client
- Steering adjustment to center detected objects
- Automatic reconnect if the ESP32 disconnects

---

## Installation (Python)

1. Clone this repository:

   ```bash
   git clone https://github.com/<your-username>/drivy-yolo-esp32.git
   cd drivy-yolo-esp32
   ```

2. Create and activate a virtual environment:

   ```bash
   python -m venv venv
   source venv/bin/activate       # On Windows: venv\Scripts\activate
   ```

3. Install dependencies:

   ```bash
   pip install -r requirements.txt
   ```

4. Ensure `best_15schilder.pt` is located in the same directory as the Python script.

---

## Running the Python GUI

1. Connect your PC (or Raspberry Pi) and the ESP32 to the same WiFi network.  
   The ESP32 sketch uses port **5001** and connects to your computer’s IP.

2. Start the control interface:

   ```bash
   python drivy_ki_stop_100_50.py
   ```

3. In the GUI:
   - Enter your camera IP (for example: `192.168.1.201`)
   - Click **Start Video** to begin live detection
   - Use arrow keys or on-screen buttons to drive manually
   - YOLO automatically issues stop or speed-limit reactions when detecting relevant signs

---

## Uploading the ESP32 Firmware

1. Open `drivy_socket_client_pin4high.ino` in Arduino IDE.
2. Set your WiFi credentials

3. Set the Python host IP address:
   ```cpp
   const char* host = "192.168.1.101";
   ```
4. Select **Board: DOIT ESP32 DevKit V1** (or your ESP32 variant).
5. Upload the sketch to your ESP32.

After booting, the ESP32 connects automatically to the Python server and receives motion commands.

---

## Communication Protocol

The Python server sends motor commands as plain text lines:

```
vx vy\n
```

where

- `vx` controls steering (−250 to 250)
- `vy` controls forward/backward speed (−250 to 250)

Example messages:
```
0 150      # Forward
-80 100    # Turn left
0 0        # Stop
```

## Requirements

Contents of `requirements.txt`:

```
ultralytics==8.3.5
torch>=2.2.0,<2.5
torchvision>=0.17,<0.20
opencv-python==4.10.0.84
numpy==1.26.4
psutil>=5.9,<6.0
pillow>=10.2,<11.0
```

---

## Troubleshooting

| Problem | Possible Solution |
|----------|-------------------|
| Camera feed not showing | Verify the camera IP and `/capture` endpoint |
| ESP32 fails to connect | Ensure the `host` IP in the `.ino` file matches your PC’s IP |
| YOLO performance slow | Lower detection frequency in the code (`if self.framecount % 3 == 0:`) |
| Socket disconnects | The server auto-restarts; check WiFi signal strength |

---

---

## License

MIT License © 2025
