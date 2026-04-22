import sys
import asyncio
import struct
import queue
import threading
import time
import socket
import json
import math

# ==========================================
# MODULE TOGGLES
# ==========================================
ENABLE_PLOT        = True   # Launch PyQtGraph dashboard UI
ENABLE_TCP_RELAY   = False  # Forward all parsed data to TCP clients (port 4243)
ENABLE_UDP         = False  # Stream JSON to local UDP socket
ENABLE_GAMEPAD     = False  # Emulate Xbox 360 controller via vgamepad

# ==========================================
# NETWORK CONFIG
# ==========================================
UDP_IP         = "127.0.0.1"
UDP_PORT       = 4242
TCP_RELAY_HOST = "0.0.0.0"
TCP_RELAY_PORT = 4243

# ==========================================
# SENSOR UUIDs
# ==========================================
MOVEMENT_DATA_UUID   = "f000aa81-0451-4000-b000-000000000000"
MOVEMENT_CONFIG_UUID = "f000aa82-0451-4000-b000-000000000000"
MOVEMENT_PERIOD_UUID = "f000aa83-0451-4000-b000-000000000000"

IR_TEMP_DATA_UUID    = "f000aa01-0451-4000-b000-000000000000"
IR_TEMP_CONFIG_UUID  = "f000aa02-0451-4000-b000-000000000000"

HUMIDITY_DATA_UUID   = "f000aa21-0451-4000-b000-000000000000"
HUMIDITY_CONFIG_UUID = "f000aa22-0451-4000-b000-000000000000"

BARO_DATA_UUID       = "f000aa41-0451-4000-b000-000000000000"
BARO_CONFIG_UUID     = "f000aa42-0451-4000-b000-000000000000"

LIGHT_DATA_UUID      = "f000aa71-0451-4000-b000-000000000000"
LIGHT_CONFIG_UUID    = "f000aa72-0451-4000-b000-000000000000"

BATTERY_LEVEL_UUID   = "00002a19-0000-1000-8000-00805f9b34fb"
IO_DATA_UUID         = "0000ffe1-0000-1000-8000-00805f9b34fb"

# ==========================================
# DASHBOARD CONFIG
# ==========================================
MAX_POINTS = 200   # Number of data points shown in scrolling plots

# ==========================================
# SENSOR PARSING — pure functions, no side effects
# ==========================================

def parse_movement(data: bytes) -> dict:
    """18-byte MPU9250 packet -> gyro (deg/s), accel (G), mag (µT)."""
    values = struct.unpack('<hhhhhhhhh', data)
    return {
        'gyro':  [values[0] / (65536.0 / 500.0),
                  values[1] / (65536.0 / 500.0),
                  values[2] / (65536.0 / 500.0)],
        'accel': [values[3] / (32768.0 / 8.0),
                  values[4] / (32768.0 / 8.0),
                  values[5] / (32768.0 / 8.0)],
        'mag':   [float(values[6]),
                  float(values[7]),
                  float(values[8])],
    }


def parse_ir_temp(data: bytes) -> dict:
    """4-byte TMP007 packet -> ir_temp and amb_temp in °C."""
    raw_obj, raw_amb = struct.unpack('<hh', data[:4])
    return {
        'ir_temp':  (raw_obj >> 2) * 0.03125,
        'amb_temp': (raw_amb >> 2) * 0.03125,
    }


def parse_humidity(data: bytes) -> dict:
    """4-byte HDC1000 packet -> temp (°C) and humidity (%RH)."""
    raw_t, raw_h = struct.unpack('<HH', data[:4])
    return {
        'temp':     -40.0 + 165.0 * raw_t / 65536.0,
        'humidity': 100.0 * raw_h / 65536.0,
    }


def parse_baro(data: bytes) -> dict:
    """6-byte BMP280 packet -> temp (°C) and pressure (mbar)."""
    tL, tM, tH, pL, pM, pH = struct.unpack('<BBBBBB', data[:6])
    return {
        'temp':     (tH * 65536 + tM * 256 + tL) / 100.0,
        'pressure': (pH * 65536 + pM * 256 + pL) / 100.0,
    }


def parse_light(data: bytes) -> dict:
    """2-byte OPT3001 packet -> lux."""
    raw = struct.unpack('<H', data[:2])[0]
    m = raw & 0x0FFF
    e = (raw & 0xF000) >> 12
    return {'lux': 0.01 * (m << e)}


# ==========================================
# GYRO CALIBRATION
# ==========================================

class GyroCalibrator:
    """
    Collects N stationary gyro samples and computes a bias offset.
    Usage:
        cal = GyroCalibrator()
        cal.start()                  # begin collecting
        cal.add_sample(gx, gy, gz)   # call from BLE thread each movement packet
        if cal.is_ready():
            bias = cal.get_bias()    # [bx, by, bz] in deg/s
    """
    SAMPLES_NEEDED = 100

    def __init__(self):
        self._collecting = False
        self._samples = []
        self._bias = [0.0, 0.0, 0.0]

    def start(self):
        self._samples = []
        self._collecting = True

    def add_sample(self, gx: float, gy: float, gz: float):
        if not self._collecting:
            return
        self._samples.append((gx, gy, gz))
        if len(self._samples) >= self.SAMPLES_NEEDED:
            self._collecting = False
            n = len(self._samples)
            self._bias = [
                sum(s[0] for s in self._samples) / n,
                sum(s[1] for s in self._samples) / n,
                sum(s[2] for s in self._samples) / n,
            ]

    def is_collecting(self) -> bool:
        return self._collecting

    def is_ready(self) -> bool:
        return not self._collecting and len(self._samples) >= self.SAMPLES_NEEDED

    def get_bias(self) -> list:
        return list(self._bias)

    def progress(self) -> int:
        """Return number of samples collected so far."""
        return len(self._samples)


# ==========================================
# ORIENTATION (MADGWICK via imufusion)
# ==========================================

try:
    import imufusion
    _ahrs = imufusion.Ahrs()
    _ahrs.settings = imufusion.Settings(
        imufusion.CONVENTION_NED,
        0.5,   # gain
        2000,  # gyroscope range (deg/s)
        10,    # acceleration rejection
        10,    # magnetic rejection
        5 * 100,  # recovery trigger period (5 s at 100 Hz)
    )
    IMUFUSION_AVAILABLE = True
except ImportError:
    IMUFUSION_AVAILABLE = False
    print("WARNING: imufusion not installed. Orientation tab will be disabled.")
    print("         Run: pip install imufusion")

_last_orientation_time = None


def update_orientation(accel: list, gyro: list, mag: list,
                       gyro_bias: list):
    """
    Feed one IMU sample into the Madgwick filter.
    Returns {'roll': float, 'pitch': float, 'yaw': float} in degrees,
    or None if imufusion is unavailable.
    """
    global _last_orientation_time

    if not IMUFUSION_AVAILABLE:
        return None

    import numpy as np

    now = time.time()
    if _last_orientation_time is None:
        dt = 0.1
    else:
        dt = now - _last_orientation_time
        dt = max(0.001, min(dt, 1.0))  # clamp to sane range
    _last_orientation_time = now

    g_cal = [gyro[i] - gyro_bias[i] for i in range(3)]

    _ahrs.update(
        np.array(g_cal,   dtype=float),
        np.array(accel,   dtype=float),
        np.array(mag,     dtype=float),
        dt,
    )

    euler = _ahrs.quaternion.to_euler()  # [roll, pitch, yaw] degrees
    return {'roll': euler[0], 'pitch': euler[1], 'yaw': euler[2]}


# ==========================================
# SHARED STATE (BLE thread <-> UI thread)
# ==========================================

# Queues — BLE thread pushes, UI thread drains
q_movement    = queue.Queue()   # dicts from parse_movement + orientation
q_ir_temp     = queue.Queue()   # dicts from parse_ir_temp
q_humidity    = queue.Queue()   # dicts from parse_humidity
q_baro        = queue.Queue()   # dicts from parse_baro
q_light       = queue.Queue()   # dicts from parse_light
q_battery     = queue.Queue()   # int (percent)
q_button      = queue.Queue()   # dict {btn1, btn2}
q_rssi        = queue.Queue()   # int (dBm, updated each scan/reconnect)
q_cal_status  = queue.Queue()   # str status messages from calibration

# Calibrator instance — UI thread calls .start(), BLE thread calls .add_sample()
gyro_cal = GyroCalibrator()


# ==========================================
# BLE WORKER
# ==========================================
from bleak import BleakClient, BleakScanner


def _movement_callback(sender, data):
    if len(data) != 18:
        return
    parsed = parse_movement(bytes(data))
    gyro = parsed['gyro']

    # Feed calibrator if active
    if gyro_cal.is_collecting():
        gyro_cal.add_sample(*gyro)
        prog = gyro_cal.progress()
        if prog % 10 == 0:
            q_cal_status.put(f"Collecting... {prog}/{GyroCalibrator.SAMPLES_NEEDED}")
        if gyro_cal.is_ready():
            q_cal_status.put(f"Done! Bias: {[f'{b:.3f}' for b in gyro_cal.get_bias()]}")

    bias = gyro_cal.get_bias()
    orientation = update_orientation(parsed['accel'], gyro, parsed['mag'], bias)

    packet = {
        'ts':          time.time(),
        'gyro':        gyro,
        'accel':       parsed['accel'],
        'mag':         parsed['mag'],
        'orientation': orientation,
    }
    q_movement.put(packet)
    _relay_send({'type': 'movement', **packet})


def _ir_temp_callback(sender, data):
    if len(data) >= 4:
        p = parse_ir_temp(bytes(data))
        q_ir_temp.put(p)
        _relay_send({'type': 'ir_temp', **p})


def _humidity_callback(sender, data):
    if len(data) >= 4:
        p = parse_humidity(bytes(data))
        q_humidity.put(p)
        _relay_send({'type': 'humidity', **p})


def _baro_callback(sender, data):
    if len(data) >= 6:
        p = parse_baro(bytes(data))
        q_baro.put(p)
        _relay_send({'type': 'baro', **p})


def _light_callback(sender, data):
    if len(data) >= 2:
        p = parse_light(bytes(data))
        q_light.put(p)
        _relay_send({'type': 'light', **p})


def _battery_callback(sender, data):
    if data:
        level = int(data[0])
        q_battery.put(level)
        _relay_send({'type': 'battery', 'level': level})


def _button_callback(sender, data):
    if data:
        val = data[0]
        p = {'btn1': bool(val & 0x01), 'btn2': bool(val & 0x02)}
        q_button.put(p)
        _relay_send({'type': 'button', **p})


async def ble_worker():
    print("[BLE] Scanning for TI SensorTag (CC2650)...")

    target = None
    target_rssi = 0

    def _detection_cb(device, adv_data):
        nonlocal target, target_rssi
        if target is None and device.name and "SensorTag" in device.name:
            target = device
            target_rssi = adv_data.rssi if adv_data.rssi is not None else 0

    async with BleakScanner(detection_callback=_detection_cb):
        await asyncio.sleep(5.0)

    if target is None:
        # fallback: plain discover
        devices = await BleakScanner.discover(timeout=3.0)
        for d in devices:
            if d.name and "SensorTag" in d.name:
                target = d
                break

    if target is None:
        print("[BLE] SensorTag not found. Make sure it is on and blinking green.")
        return

    print(f"[BLE] Found: {target.name} [{target.address}]  RSSI: {target_rssi} dBm")
    q_rssi.put(target_rssi)

    print(f"[BLE] Connecting to {target.address}...")
    async with BleakClient(target.address) as client:
        await asyncio.sleep(0.5)  # Allow GATT service discovery to settle

        # Battery — read first (always works), notify separately (may not be supported)
        try:
            raw = await client.read_gatt_char(BATTERY_LEVEL_UUID)
            _battery_callback(None, raw)
        except Exception as exc:
            print(f"[BLE] Battery read failed: {exc}")
        try:
            await client.start_notify(BATTERY_LEVEL_UUID, _battery_callback)
        except Exception:
            pass  # Battery Level notify not supported on this firmware — read-only is fine

        # Buttons
        try:
            await client.start_notify(IO_DATA_UUID, _button_callback)
        except Exception as exc:
            print(f"[BLE] Buttons failed: {exc}")

        # IR Temperature
        try:
            await client.write_gatt_char(IR_TEMP_CONFIG_UUID, b'\x01', response=True)
            await client.start_notify(IR_TEMP_DATA_UUID, _ir_temp_callback)
        except Exception as exc:
            print(f"[BLE] IR Temp failed: {exc}")

        # Humidity
        try:
            await client.write_gatt_char(HUMIDITY_CONFIG_UUID, b'\x01', response=True)
            await client.start_notify(HUMIDITY_DATA_UUID, _humidity_callback)
        except Exception as exc:
            print(f"[BLE] Humidity failed: {exc}")

        # Barometer
        try:
            await client.write_gatt_char(BARO_CONFIG_UUID, b'\x01', response=True)
            await client.start_notify(BARO_DATA_UUID, _baro_callback)
        except Exception as exc:
            print(f"[BLE] Barometer failed: {exc}")

        # Light
        try:
            await client.write_gatt_char(LIGHT_CONFIG_UUID, b'\x01', response=True)
            await client.start_notify(LIGHT_DATA_UUID, _light_callback)
        except Exception as exc:
            print(f"[BLE] Light failed: {exc}")

        # Movement: subscribe first, then configure sensor
        await client.start_notify(MOVEMENT_DATA_UUID, _movement_callback)
        await client.write_gatt_char(MOVEMENT_PERIOD_UUID, b"\x0A", response=True)
        await asyncio.sleep(0.1)
        await client.write_gatt_char(MOVEMENT_CONFIG_UUID, b"\x00\x00", response=True)
        await asyncio.sleep(0.1)
        # 0x7F = gyro XYZ + accel XYZ + mag; 0x02 = 8G accel range
        await client.write_gatt_char(MOVEMENT_CONFIG_UUID, b"\x7F\x02", response=True)
        await asyncio.sleep(0.5)

        print("[BLE] All sensors enabled. Streaming...")
        while True:
            await asyncio.sleep(1.0)


def start_ble_loop():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        loop.run_until_complete(ble_worker())
    except Exception as exc:
        print(f"[BLE] Error: {exc}")


# ==========================================
# TCP RELAY OUTPUT (optional)
# ==========================================

class TCPRelay:
    """
    Listens for TCP clients and forwards all parsed sensor data as
    newline-delimited JSON. Enabled when ENABLE_TCP_RELAY = True.
    """

    def __init__(self, host: str, port: int):
        self._host = host
        self._port = port
        self._clients: list = []
        self._lock = threading.Lock()
        t = threading.Thread(target=self._accept_loop, daemon=True)
        t.start()

    def _accept_loop(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self._host, self._port))
        srv.listen(5)
        print(f"[TCP RELAY] Listening on {self._host}:{self._port}")
        while True:
            try:
                conn, addr = srv.accept()
                print(f"[TCP RELAY] Client connected: {addr}")
                with self._lock:
                    self._clients.append(conn)
            except OSError:
                break

    def send(self, obj: dict):
        line = (json.dumps(obj) + "\n").encode("utf-8")
        with self._lock:
            dead = []
            for conn in self._clients:
                try:
                    conn.sendall(line)
                except OSError:
                    dead.append(conn)
            for conn in dead:
                self._clients.remove(conn)


# Singleton — created only if ENABLE_TCP_RELAY is True
_tcp_relay = None

def _relay_send(obj: dict):
    """Send a dict to TCP relay clients if relay is active."""
    if _tcp_relay is not None:
        _tcp_relay.send(obj)


def init_tcp_relay():
    """Call once at startup when ENABLE_TCP_RELAY = True."""
    global _tcp_relay
    _tcp_relay = TCPRelay(TCP_RELAY_HOST, TCP_RELAY_PORT)


# ==========================================
# DASHBOARD UI — skeleton
# ==========================================
if ENABLE_PLOT:
    from PyQt5.QtWidgets import (
        QApplication, QMainWindow, QWidget,
        QVBoxLayout, QHBoxLayout,
        QTabWidget, QLabel, QPushButton,
    )
    from PyQt5.QtCore import QTimer, Qt
    from PyQt5.QtGui import QFont
    import pyqtgraph as pg

    class SensorTagDashboard(QMainWindow):
        def __init__(self):
            super().__init__()
            self.setWindowTitle("SensorTag CC2650 — Dashboard")
            self.resize(1100, 800)

            root = QWidget()
            self.setCentralWidget(root)
            root_layout = QVBoxLayout(root)
            root_layout.setContentsMargins(6, 6, 6, 6)
            root_layout.setSpacing(4)

            # ── Top bar ──────────────────────────────────
            top = QHBoxLayout()
            top.setSpacing(16)

            def _status_label(text):
                lbl = QLabel(text)
                f = QFont()
                f.setPointSize(11)
                f.setBold(True)
                lbl.setFont(f)
                return lbl

            self.lbl_freq    = _status_label("Freq: --  Hz")
            self.lbl_battery = _status_label("Battery: -- %")
            self.lbl_rssi    = _status_label("RSSI: -- dBm")
            self.lbl_battery.setStyleSheet("color: green;")

            top.addWidget(self.lbl_freq)
            top.addStretch()
            top.addWidget(self.lbl_rssi)
            top.addWidget(self.lbl_battery)
            root_layout.addLayout(top)

            # ── Tab widget ───────────────────────────────
            self.tabs = QTabWidget()
            root_layout.addWidget(self.tabs)

            # Tabs are populated in Steps 7–10
            self._tab_imu         = QWidget()
            self._tab_orientation = QWidget()
            self._tab_environment = QWidget()
            self._tab_status      = QWidget()

            self.tabs.addTab(self._tab_imu,         "IMU")
            self.tabs.addTab(self._tab_orientation, "Orientation")
            self.tabs.addTab(self._tab_environment, "Environment")
            self.tabs.addTab(self._tab_status,      "Status")

            self._build_imu_tab()
            self._build_orientation_tab()
            self._build_environment_tab()
            self._build_status_tab()

            # Pending packets for tabs that share the movement queue
            self._pending_movement: list = []

            # ── Receive-time ring for frequency calc ─────
            self._recv_times: list = []

            # ── QTimer drives all UI updates ─────────────
            self._timer = QTimer()
            self._timer.timeout.connect(self._update)
            self._timer.start(16)  # ~60 Hz UI refresh

            # ── Start BLE thread ─────────────────────────
            self._ble_thread = threading.Thread(
                target=start_ble_loop, daemon=True
            )
            self._ble_thread.start()

        def _update(self):
            """Drain all queues and refresh widgets. Called by QTimer."""
            # Movement → IMU + Orientation tabs
            while not q_movement.empty():
                pkt = q_movement.get()
                self._recv_times.append(pkt['ts'])
                if len(self._recv_times) > 60:
                    self._recv_times.pop(0)
                self._update_imu(pkt)
                self._update_orientation(pkt)

            # Update frequency label
            if len(self._recv_times) > 1:
                span = self._recv_times[-1] - self._recv_times[0]
                if span > 0:
                    freq = (len(self._recv_times) - 1) / span
                    self.lbl_freq.setText(f"Freq: {freq:.1f} Hz")

            self._update_environment()
            self._update_status()

            # RSSI
            while not q_rssi.empty():
                rssi = q_rssi.get()
                self.lbl_rssi.setText(f"RSSI: {rssi} dBm")

        # ── helpers ──────────────────────────────────────────────────────────
        @staticmethod
        def _make_plot(title, y_min, y_max, y_label=""):
            pw = pg.PlotWidget(title=title)
            pw.addLegend()
            pw.setYRange(y_min, y_max)
            pw.enableAutoRange(axis='y', enable=False)
            pw.setLabel('left', y_label)
            pw.showGrid(x=False, y=True, alpha=0.3)
            return pw

        @staticmethod
        def _make_xyz_curves(plot):
            return {
                'X': plot.plot(pen='r', name='X'),
                'Y': plot.plot(pen='g', name='Y'),
                'Z': plot.plot(pen='b', name='Z'),
            }

        @staticmethod
        def _scroll(buf, val):
            buf.pop(0)
            buf.append(val)

        # ── Step 7: IMU tab ───────────────────────────────────────────────────
        def _build_imu_tab(self):
            layout = QVBoxLayout(self._tab_imu)

            self._plot_accel = self._make_plot("Accelerometer (G)",      -8,   8,  "G")
            self._plot_gyro  = self._make_plot("Gyroscope (deg/s)",    -500, 500, "deg/s")
            self._plot_mag   = self._make_plot("Magnetometer (µT)",    -500, 500, "µT")

            layout.addWidget(self._plot_accel)
            layout.addWidget(self._plot_gyro)
            layout.addWidget(self._plot_mag)

            self._curves_accel = self._make_xyz_curves(self._plot_accel)
            self._curves_gyro  = self._make_xyz_curves(self._plot_gyro)
            self._curves_mag   = self._make_xyz_curves(self._plot_mag)

            z = [0.0] * MAX_POINTS
            self._buf_accel = {'X': z[:], 'Y': z[:], 'Z': z[:]}
            self._buf_gyro  = {'X': z[:], 'Y': z[:], 'Z': z[:]}
            self._buf_mag   = {'X': z[:], 'Y': z[:], 'Z': z[:]}

        def _update_imu(self, pkt):
            accel = pkt['accel']
            gyro  = pkt['gyro']
            mag   = pkt['mag']
            for ax, i in zip(['X', 'Y', 'Z'], range(3)):
                self._scroll(self._buf_accel[ax], accel[i])
                self._scroll(self._buf_gyro[ax],  gyro[i])
                self._scroll(self._buf_mag[ax],   mag[i])
            for ax in ['X', 'Y', 'Z']:
                self._curves_accel[ax].setData(self._buf_accel[ax])
                self._curves_gyro[ax].setData(self._buf_gyro[ax])
                self._curves_mag[ax].setData(self._buf_mag[ax])

        # ── Step 8: Orientation tab ───────────────────────────────────────────
        def _build_orientation_tab(self):
            layout = QVBoxLayout(self._tab_orientation)

            if not IMUFUSION_AVAILABLE:
                lbl = QLabel("imufusion not installed.\nRun: pip install imufusion")
                lbl.setStyleSheet("color: red; font-size: 14pt;")
                layout.addWidget(lbl)
                self._orientation_available = False
                return

            self._orientation_available = True

            self._plot_roll  = self._make_plot("Roll (deg)",  -180, 180, "deg")
            self._plot_pitch = self._make_plot("Pitch (deg)", -90,   90, "deg")
            self._plot_yaw   = self._make_plot("Yaw (deg)",  -180,  180, "deg")

            for plot, name, color in [
                (self._plot_roll,  "Roll",  'c'),
                (self._plot_pitch, "Pitch", 'm'),
                (self._plot_yaw,   "Yaw",   'y'),
            ]:
                layout.addWidget(plot)

            self._curve_roll  = self._plot_roll.plot(pen='c',  name="Roll")
            self._curve_pitch = self._plot_pitch.plot(pen='m', name="Pitch")
            self._curve_yaw   = self._plot_yaw.plot(pen='y',   name="Yaw")

            z = [0.0] * MAX_POINTS
            self._buf_roll  = z[:]
            self._buf_pitch = z[:]
            self._buf_yaw   = z[:]

        def _update_orientation(self, pkt):
            if not getattr(self, '_orientation_available', False):
                return
            ori = pkt.get('orientation')
            if ori is None:
                return
            self._scroll(self._buf_roll,  ori['roll'])
            self._scroll(self._buf_pitch, ori['pitch'])
            self._scroll(self._buf_yaw,   ori['yaw'])
            self._curve_roll.setData(self._buf_roll)
            self._curve_pitch.setData(self._buf_pitch)
            self._curve_yaw.setData(self._buf_yaw)

        # ── Step 9: Environment tab ───────────────────────────────────────────
        def _build_environment_tab(self):
            from PyQt5.QtWidgets import QGridLayout, QGroupBox

            outer = QVBoxLayout(self._tab_environment)

            # ── Current-value readouts ────────────────────────────────────────
            grid_box = QGroupBox("Current Values")
            grid = QGridLayout(grid_box)

            def _val_label(text="--"):
                lbl = QLabel(text)
                f = QFont()
                f.setPointSize(13)
                f.setBold(True)
                lbl.setFont(f)
                return lbl

            self.lbl_ir_temp  = _val_label()
            self.lbl_amb_temp = _val_label()
            self.lbl_hum_temp = _val_label()
            self.lbl_humidity = _val_label()
            self.lbl_pressure = _val_label()
            self.lbl_lux      = _val_label()

            for col, header in enumerate(["Sensor", "Value"]):
                h = QLabel(header)
                f = h.font(); f.setBold(True); h.setFont(f)
                grid.addWidget(h, 0, col)

            rows = [
                ("IR Temperature",  self.lbl_ir_temp),
                ("Ambient Temp (TMP007)", self.lbl_amb_temp),
                ("Humidity Temp",   self.lbl_hum_temp),
                ("Humidity",        self.lbl_humidity),
                ("Pressure",        self.lbl_pressure),
                ("Light",           self.lbl_lux),
            ]
            for r, (name, lbl) in enumerate(rows, start=1):
                grid.addWidget(QLabel(name), r, 0)
                grid.addWidget(lbl,          r, 1)

            outer.addWidget(grid_box)

            # ── Trend plots ───────────────────────────────────────────────────
            self._plot_temp     = self._make_plot("Temperature (°C)", -10, 60, "°C")
            self._plot_hum      = self._make_plot("Humidity (%RH)",     0, 100, "%RH")
            self._plot_pressure = self._make_plot("Pressure (mbar)",  900, 1100, "mbar")
            self._plot_lux      = self._make_plot("Light (lux)",        0, 2000, "lux")

            outer.addWidget(self._plot_temp)
            outer.addWidget(self._plot_hum)
            outer.addWidget(self._plot_pressure)
            outer.addWidget(self._plot_lux)

            # Temperature plot: 3 curves
            self._curve_ir_temp  = self._plot_temp.plot(pen='r', name="IR Temp")
            self._curve_amb_temp = self._plot_temp.plot(pen='y', name="Ambient (TMP007)")
            self._curve_hum_temp = self._plot_temp.plot(pen='c', name="Humidity Temp")
            self._curve_humidity = self._plot_hum.plot(pen='b',  name="Humidity")
            self._curve_pressure = self._plot_pressure.plot(pen='g', name="Pressure")
            self._curve_lux      = self._plot_lux.plot(pen='w',  name="Lux")

            z = [0.0] * MAX_POINTS
            self._buf_ir_temp  = z[:]
            self._buf_amb_temp = z[:]
            self._buf_hum_temp = z[:]
            self._buf_humidity = z[:]
            self._buf_pressure = z[:]
            self._buf_lux      = z[:]

        def _update_environment(self):
            while not q_ir_temp.empty():
                p = q_ir_temp.get()
                self._scroll(self._buf_ir_temp,  p['ir_temp'])
                self._scroll(self._buf_amb_temp, p['amb_temp'])
                self.lbl_ir_temp.setText(f"{p['ir_temp']:.1f} °C")
                self.lbl_amb_temp.setText(f"{p['amb_temp']:.1f} °C")
                self._curve_ir_temp.setData(self._buf_ir_temp)
                self._curve_amb_temp.setData(self._buf_amb_temp)

            while not q_humidity.empty():
                p = q_humidity.get()
                self._scroll(self._buf_hum_temp, p['temp'])
                self._scroll(self._buf_humidity, p['humidity'])
                self.lbl_hum_temp.setText(f"{p['temp']:.1f} °C")
                self.lbl_humidity.setText(f"{p['humidity']:.1f} %")
                self._curve_hum_temp.setData(self._buf_hum_temp)
                self._curve_humidity.setData(self._buf_humidity)

            while not q_baro.empty():
                p = q_baro.get()
                self._scroll(self._buf_pressure, p['pressure'])
                self.lbl_pressure.setText(f"{p['pressure']:.1f} mbar")
                self._curve_pressure.setData(self._buf_pressure)

            while not q_light.empty():
                p = q_light.get()
                self._scroll(self._buf_lux, p['lux'])
                self.lbl_lux.setText(f"{p['lux']:.1f} lux")
                self._curve_lux.setData(self._buf_lux)

        # ── Step 10: Status tab ───────────────────────────────────────────────
        def _build_status_tab(self):
            from PyQt5.QtWidgets import QGroupBox, QGridLayout, QProgressBar

            outer = QVBoxLayout(self._tab_status)
            outer.setSpacing(12)

            # ── Battery ───────────────────────────────────────────────────────
            batt_box = QGroupBox("Battery")
            batt_lay = QVBoxLayout(batt_box)
            self._batt_bar = QProgressBar()
            self._batt_bar.setRange(0, 100)
            self._batt_bar.setValue(0)
            self._batt_bar.setTextVisible(True)
            self._batt_bar.setFormat("%v %")
            self._batt_bar.setMinimumHeight(28)
            batt_lay.addWidget(self._batt_bar)
            outer.addWidget(batt_box)

            # ── Buttons ───────────────────────────────────────────────────────
            btn_box = QGroupBox("Buttons")
            btn_lay = QHBoxLayout(btn_box)

            def _indicator(text):
                lbl = QLabel(text)
                f = QFont(); f.setPointSize(14); f.setBold(True)
                lbl.setFont(f)
                lbl.setFixedSize(160, 48)
                lbl.setStyleSheet("background: #444; color: #aaa; border-radius: 6px; qproperty-alignment: AlignCenter;")
                return lbl

            self._lbl_btn1 = _indicator("BTN 1  OFF")
            self._lbl_btn2 = _indicator("BTN 2  OFF")
            btn_lay.addWidget(self._lbl_btn1)
            btn_lay.addWidget(self._lbl_btn2)
            btn_lay.addStretch()
            outer.addWidget(btn_box)

            # ── RSSI ──────────────────────────────────────────────────────────
            rssi_box = QGroupBox("Signal Strength")
            rssi_lay = QVBoxLayout(rssi_box)
            self._lbl_rssi_big = QLabel("RSSI: -- dBm")
            f2 = QFont(); f2.setPointSize(13); f2.setBold(True)
            self._lbl_rssi_big.setFont(f2)
            rssi_lay.addWidget(self._lbl_rssi_big)
            outer.addWidget(rssi_box)

            # ── Gyro calibration ──────────────────────────────────────────────
            cal_box = QGroupBox("Gyroscope Calibration")
            cal_lay = QVBoxLayout(cal_box)

            self._lbl_cal_status = QLabel(
                "Hold sensor still, then press Calibrate."
            )
            self._lbl_cal_bias = QLabel("Bias: [0.000, 0.000, 0.000] deg/s")
            f3 = QFont(); f3.setPointSize(11)
            self._lbl_cal_status.setFont(f3)
            self._lbl_cal_bias.setFont(f3)

            self._btn_calibrate = QPushButton("Calibrate Gyro")
            self._btn_calibrate.setFixedHeight(36)
            self._btn_calibrate.clicked.connect(self._on_calibrate)

            cal_lay.addWidget(self._lbl_cal_status)
            cal_lay.addWidget(self._lbl_cal_bias)
            cal_lay.addWidget(self._btn_calibrate)
            outer.addWidget(cal_box)
            outer.addStretch()

        def _on_calibrate(self):
            if gyro_cal.is_collecting():
                return  # already running
            gyro_cal.start()
            self._btn_calibrate.setEnabled(False)
            self._lbl_cal_status.setText("Collecting samples… keep sensor still.")

        def _update_status(self):
            # Button states
            while not q_button.empty():
                p = q_button.get()
                for lbl, key, name in [
                    (self._lbl_btn1, 'btn1', 'BTN 1'),
                    (self._lbl_btn2, 'btn2', 'BTN 2'),
                ]:
                    if p[key]:
                        lbl.setText(f"{name}  ON")
                        lbl.setStyleSheet(
                            "background: #2a7; color: white; border-radius: 6px;"
                        )
                    else:
                        lbl.setText(f"{name}  OFF")
                        lbl.setStyleSheet(
                            "background: #444; color: #aaa; border-radius: 6px;"
                        )

            # Battery bar + top-bar label
            while not q_battery.empty():
                lvl = q_battery.get()
                self._batt_bar.setValue(lvl)
                color = "red" if lvl < 20 else ("orange" if lvl < 40 else "green")
                self.lbl_battery.setText(f"Battery: {lvl} %")
                self.lbl_battery.setStyleSheet(f"color: {color};")
                if lvl < 20:
                    chunk = "QProgressBar::chunk { background: red; }"
                elif lvl < 40:
                    chunk = "QProgressBar::chunk { background: orange; }"
                else:
                    chunk = "QProgressBar::chunk { background: #2a7; }"
                self._batt_bar.setStyleSheet(chunk)

            # RSSI — status tab + top bar
            while not q_rssi.empty():
                rssi = q_rssi.get()
                self._lbl_rssi_big.setText(f"RSSI: {rssi} dBm")
                self.lbl_rssi.setText(f"RSSI: {rssi} dBm")

            # Calibration status messages
            while not q_cal_status.empty():
                msg = q_cal_status.get()
                self._lbl_cal_status.setText(msg)
                if gyro_cal.is_ready():
                    bias = gyro_cal.get_bias()
                    self._lbl_cal_bias.setText(
                        f"Bias: [{bias[0]:.3f}, {bias[1]:.3f}, {bias[2]:.3f}] deg/s"
                    )
                    self._btn_calibrate.setEnabled(True)


# ==========================================
# BOOTSTRAPPER
# ==========================================

def main():
    if ENABLE_TCP_RELAY:
        init_tcp_relay()

    if ENABLE_PLOT:
        from PyQt5.QtWidgets import QApplication
        app = QApplication(sys.argv)
        window = SensorTagDashboard()
        window.show()
        sys.exit(app.exec_())
    else:
        # Headless mode: BLE only, data goes to TCP relay / UDP
        print("--- SensorTag Headless Mode ---")
        print(f"TCP Relay: {'ENABLED on port ' + str(TCP_RELAY_PORT) if ENABLE_TCP_RELAY else 'DISABLED'}")
        print(f"UDP:       {'ENABLED ' + UDP_IP + ':' + str(UDP_PORT) if ENABLE_UDP else 'DISABLED'}")
        print("-------------------------------")
        start_ble_loop()


if __name__ == '__main__':
    main()
