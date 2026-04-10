import sys
import asyncio
import struct
import queue
import threading
import time
import socket
import json

# ==========================================
# MODULE TOGGLES
# Enable or disable data output routes here:
# ==========================================
ENABLE_PLOT = False        # Launch PyQtGraph dashboard UI
ENABLE_UDP = False         # Stream JSON to local UDP socket (for Godot)
ENABLE_GAMEPAD = True     # Emulate Xbox 360 controller via vgamepad

# ==========================================
# CONSTANTS & CONFIGURATION
# ==========================================
UDP_IP = "127.0.0.1"
UDP_PORT = 4242

MOVEMENT_SVC_UUID = "f000aa80-0451-4000-b000-000000000000"
MOVEMENT_DATA_UUID = "f000aa81-0451-4000-b000-000000000000"
MOVEMENT_CONFIG_UUID = "f000aa82-0451-4000-b000-000000000000"
MOVEMENT_PERIOD_UUID = "f000aa83-0451-4000-b000-000000000000"
BATTERY_LEVEL_UUID = "00002a19-0000-1000-8000-00805f9b34fb"
IO_DATA_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"

MAX_POINTS = 100

# ==========================================
# GLOBAL HARDWARE / NETWORK INITS
# ==========================================
sock = None
if ENABLE_UDP:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

gamepad = None
if ENABLE_GAMEPAD:
    try:
        import vgamepad as vg
        gamepad = vg.VX360Gamepad()
    except ImportError:
        print("WARNING: vgamepad is not installed. Please run `pip install vgamepad`.")
        ENABLE_GAMEPAD = False

# ==========================================
# PYQTGRAPH VISUALIZER UI
# ==========================================
if ENABLE_PLOT:
    from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QWidget, QLabel
    from PyQt5.QtCore import QTimer, Qt
    import pyqtgraph as pg

    class SensorTagVisualizer(QMainWindow):
        def __init__(self):
            super().__init__()
            self.setWindowTitle("SensorTag CC2650 Visualizer (PyQtGraph)")
            self.resize(800, 700)

            central_widget = QWidget()
            self.setCentralWidget(central_widget)
            main_layout = QVBoxLayout(central_widget)

            # Top Bar
            top_layout = QHBoxLayout()
            
            self.freq_label = QLabel("Live Frequency: Calculating...")
            font = self.freq_label.font()
            font.setPointSize(14)
            font.setBold(True)
            self.freq_label.setFont(font)
            top_layout.addWidget(self.freq_label, alignment=Qt.AlignLeft)

            self.battery_label = QLabel("Battery: --%")
            font_batt = self.battery_label.font()
            font_batt.setPointSize(14)
            font_batt.setBold(True)
            self.battery_label.setFont(font_batt)
            self.battery_label.setStyleSheet("color: green;")
            top_layout.addWidget(self.battery_label, alignment=Qt.AlignRight)

            main_layout.addLayout(top_layout)

            # PyqtGraph setup
            self.plot_accel = pg.PlotWidget(title="Accelerometer (G)")
            self.plot_accel.addLegend()
            self.plot_accel.setYRange(-3, 3)
            main_layout.addWidget(self.plot_accel)

            self.plot_gyro = pg.PlotWidget(title="Gyroscope (deg/s)")
            self.plot_gyro.addLegend()
            self.plot_gyro.setYRange(-250, 250)
            main_layout.addWidget(self.plot_gyro)

            # Plot curves
            self.curves_accel = {
                'X': self.plot_accel.plot(pen='r', name='X'),
                'Y': self.plot_accel.plot(pen='g', name='Y'),
                'Z': self.plot_accel.plot(pen='b', name='Z')
            }
            self.curves_gyro = {
                'X': self.plot_gyro.plot(pen='r', name='X'),
                'Y': self.plot_gyro.plot(pen='g', name='Y'),
                'Z': self.plot_gyro.plot(pen='b', name='Z')
            }

            # Data buffers
            self.data_accel = {'X': [0]*MAX_POINTS, 'Y': [0]*MAX_POINTS, 'Z': [0]*MAX_POINTS}
            self.data_gyro = {'X': [0]*MAX_POINTS, 'Y': [0]*MAX_POINTS, 'Z': [0]*MAX_POINTS}

            # Queues from BLE Thread
            self.data_queue = queue.Queue()
            self.timestamp_queue = queue.Queue()
            self.battery_queue = queue.Queue()
            self.receive_times = []

            # Start timer for UI updates (~16ms loop)
            self.timer = QTimer()
            self.timer.timeout.connect(self.update_plots)
            self.timer.start(16)

            # Start BLE Thread
            self.ble_thread = threading.Thread(target=start_ble_loop, args=(self.data_queue, self.timestamp_queue, self.battery_queue), daemon=True)
            self.ble_thread.start()

        def update_plots(self):
            # Handle Battery updates
            while not self.battery_queue.empty():
                batt_level = self.battery_queue.get()
                self.battery_label.setText(f"Battery: {batt_level}%")
                if batt_level < 20:
                    self.battery_label.setStyleSheet("color: red;")
                else:
                    self.battery_label.setStyleSheet("color: green;")

            updated = False
            while not self.data_queue.empty():
                data = self.data_queue.get()
                ts = self.timestamp_queue.get()
                
                self.receive_times.append(ts)
                if len(self.receive_times) > 50:
                    self.receive_times.pop(0)
                
                accel, gyro = data['accel'], data['gyro']
                
                for ax, i in zip(['X', 'Y', 'Z'], [0, 1, 2]):
                    self.data_accel[ax].pop(0)
                    self.data_accel[ax].append(accel[i])
                    
                    self.data_gyro[ax].pop(0)
                    self.data_gyro[ax].append(gyro[i])
                    
                updated = True
                
            if updated:
                if len(self.receive_times) > 1:
                    duration = self.receive_times[-1] - self.receive_times[0]
                    if duration > 0:
                        freq = (len(self.receive_times) - 1) / duration
                        self.freq_label.setText(f"Live Frequency: {freq:.1f} Hz")

                for ax in ['X', 'Y', 'Z']:
                    self.curves_accel[ax].setData(self.data_accel[ax])
                    self.curves_gyro[ax].setData(self.data_gyro[ax])

# ==========================================
# BLE HANDLERS
# ==========================================
from bleak import BleakClient, BleakScanner

def notification_handler(sender, data, data_queue, timestamp_queue):
    if len(data) == 18:
        ts = time.time()
        values = struct.unpack('<hhhhhhhhh', data)
        
        # Gyro
        gyro_x = (values[0] * 1.0) / (65536.0 / 500.0)
        gyro_y = (values[1] * 1.0) / (65536.0 / 500.0)
        gyro_z = (values[2] * 1.0) / (65536.0 / 500.0)
        
        # Accel (2G range scaling)
        accel_x = (values[3] * 1.0) / (32768.0 / 2.0)
        accel_y = (values[4] * 1.0) / (32768.0 / 2.0)
        accel_z = (values[5] * 1.0) / (32768.0 / 2.0)
        
        # 1. Plot Logic
        if ENABLE_PLOT:
            parsed = {
                'gyro': (gyro_x, gyro_y, gyro_z),
                'accel': (accel_x, accel_y, accel_z)
            }
            data_queue.put(parsed)
            timestamp_queue.put(ts)
            
        # 2. UDP Broadcasting
        if ENABLE_UDP and sock is not None:
            payload = {
                "accel": {"x": accel_x, "y": accel_y, "z": accel_z},
                "gyro":  {"x": gyro_x, "y": gyro_y, "z": gyro_z}
            }
            json_data = json.dumps(payload).encode('utf-8')
            sock.sendto(json_data, (UDP_IP, UDP_PORT))

        # 3. Virtual Gamepad (X360) Mapping
        if ENABLE_GAMEPAD and gamepad is not None:
            # Scale -2.0 -> 2.0 G limits onto the -1.0 -> 1.0 joystick axis.
            # Invert or swap axes later depending on physically orienting the SensorTag.
            joy_x = max(min(accel_x, 1.0), -1.0)
            joy_y = max(min(accel_y, 1.0), -1.0)
            
            gamepad.left_joystick_float(x_value_float=joy_x, y_value_float=joy_y)
            gamepad.update()

def battery_handler(sender, data, battery_queue):
    if len(data) >= 1:
        batt_level = int(data[0])
        battery_queue.put(batt_level)

def button_handler(sender, data):
    if len(data) >= 1:
        val = data[0]
        btn1 = (val & 0x01) > 0  # Left / Button 1
        btn2 = (val & 0x02) > 0  # Right / Button 2
        
        if ENABLE_GAMEPAD and gamepad is not None:
            if btn1:
                gamepad.press_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_A)
            else:
                gamepad.release_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_A)
                
            if btn2:
                gamepad.press_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_B)
            else:
                gamepad.release_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_B)
                
            gamepad.update()
            
        if not ENABLE_PLOT:
            print(f"[STATUS] Button State -> Left (A): {btn1}, Right (B): {btn2}")

async def ble_worker(data_queue, timestamp_queue, battery_queue):
    print("Scanning for TI SensorTag (CC2650)...")
    devices = await BleakScanner.discover(timeout=5.0)
    target_address = None
    for d in devices:
        if d.name and "SensorTag" in d.name:
            print(f"Found SensorTag: {d.name} [{d.address}]")
            target_address = d.address
            break
            
    if not target_address:
        print("SensorTag not found. Ensure it is turned on and flashing green.")
        return
        
    print(f"Connecting to {target_address}...")
    async with BleakClient(target_address) as client:
        # Battery setup
        try:
            print("Fetching Battery Level...")
            def _battery_callback(sender, data):
                battery_handler(sender, data, battery_queue)
            
            # Start notifications
            await client.start_notify(BATTERY_LEVEL_UUID, _battery_callback)
            
            # Read explicitly because notifications only fire on value change
            batt_raw = await client.read_gatt_char(BATTERY_LEVEL_UUID)
            battery_handler(None, batt_raw, battery_queue)
        except Exception as e:
            print(f"Could not read battery: {e}")

        # Button Service setup
        try:
            print("Subscribing to Buttons...")
            await client.start_notify(IO_DATA_UUID, button_handler)
        except Exception as e:
            print(f"Could not subscribe to buttons: {e}")

        # Movement initialization
        print("Setting Period Frequency to ~10Hz (0x0A)...")
        await client.write_gatt_char(MOVEMENT_PERIOD_UUID, b'\x0A', response=True)
        await asyncio.sleep(0.5)

        print("Enabling Accel/Gyro Movement axes...")
        await client.write_gatt_char(MOVEMENT_CONFIG_UUID, b'\x3F\x00', response=True)
        await asyncio.sleep(0.5)
        
        print("Subscribing to Characteristics...")
        def _mov_callback(sender, data):
            notification_handler(sender, data, data_queue, timestamp_queue)

        await client.start_notify(MOVEMENT_DATA_UUID, _mov_callback)
        print("Subscribed. Streaming real-time data...")
        
        # Keep connection alive forever
        while True:
            await asyncio.sleep(1.0)

def start_ble_loop(data_queue, timestamp_queue, battery_queue):
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        loop.run_until_complete(ble_worker(data_queue, timestamp_queue, battery_queue))
    except Exception as e:
        print(f"BLE Error: {e}")

# ==========================================
# BOOTSTRAPPER
# ==========================================
if __name__ == '__main__':
    if ENABLE_PLOT:
        # GUI mode spins up the UI loop natively and delegates BLE to a background thread.
        app = QApplication(sys.argv)
        window = SensorTagVisualizer()
        window.show()
        sys.exit(app.exec_())
    else:
        # Headless mode handles BLE gracefully in the main execution thread.
        print("--- SensorTag Headless Mode ---")
        print(f"Plotting UI:  DISABLED")
        print(f"UDP Stream:   {'ENABLED ' + UDP_IP + ':' + str(UDP_PORT) if ENABLE_UDP else 'DISABLED'}")
        print(f"vGamepad:     {'ENABLED [X360]' if ENABLE_GAMEPAD else 'DISABLED'}")
        print("-------------------------------")
        
        d_q = queue.Queue()
        t_q = queue.Queue()
        b_q = queue.Queue()

        def battery_printer():
            last_batt = None
            while True:
                if not b_q.empty():
                    batt = b_q.get()
                    if batt != last_batt:
                        print(f"[STATUS] Live Battery: {batt}%")
                        last_batt = batt
                time.sleep(1)

        threading.Thread(target=battery_printer, daemon=True).start()
        start_ble_loop(d_q, t_q, b_q)
