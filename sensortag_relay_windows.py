"""
sensortag_relay_windows.py — Run this on Windows

Connects to the TI SensorTag CC2650 over Bluetooth LE (using Windows BLE APIs
via bleak) and relays all sensor notifications to TCP clients as newline-
delimited JSON messages.

Usage (Windows PowerShell / cmd):
    pip install bleak
    python sensortag_relay_windows.py

TCP clients connect to port 4243 and receive one JSON object per line.
Each message has a "type" field identifying the sensor.
"""

import asyncio
import json
import struct
import socket
import threading
import time
from typing import List

from bleak import BleakClient, BleakScanner

# ==========================================
# CONFIGURATION
# ==========================================
RELAY_HOST = "0.0.0.0"
RELAY_PORT = 4243

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
# TCP CLIENT TRACKING
# ==========================================
_clients: List[socket.socket] = []
_clients_lock = threading.Lock()
_rssi: int = 0


def _send_all_clients(line: bytes) -> None:
    with _clients_lock:
        dead = []
        for conn in _clients:
            try:
                conn.sendall(line)
            except OSError:
                dead.append(conn)
        for conn in dead:
            _clients.remove(conn)
            print("[RELAY] Client disconnected.")


def _send_json(obj: dict) -> None:
    line = (json.dumps(obj) + "\n").encode("utf-8")
    _send_all_clients(line)


# ==========================================
# BLE CALLBACKS
# ==========================================
def _on_movement(sender, data: bytearray) -> None:
    if len(data) != 18:
        return
    values = struct.unpack('<hhhhhhhhh', data)
    gyro_x  = values[0] / (65536.0 / 500.0)
    gyro_y  = values[1] / (65536.0 / 500.0)
    gyro_z  = values[2] / (65536.0 / 500.0)
    accel_x = values[3] / (32768.0 / 8.0)
    accel_y = values[4] / (32768.0 / 8.0)
    accel_z = values[5] / (32768.0 / 8.0)
    mag_x   = float(values[6])
    mag_y   = float(values[7])
    mag_z   = float(values[8])
    _send_json({
        "type":  "movement",
        "ts":    time.time(),
        "gyro":  [gyro_x,  gyro_y,  gyro_z],
        "accel": [accel_x, accel_y, accel_z],
        "mag":   [mag_x,   mag_y,   mag_z],
        "rssi":  _rssi,
    })


def _on_battery(sender, data: bytearray) -> None:
    if data:
        _send_json({"type": "battery", "level": int(data[0])})


def _on_button(sender, data: bytearray) -> None:
    if data:
        val = data[0]
        _send_json({
            "type": "button",
            "btn1": bool(val & 0x01),
            "btn2": bool(val & 0x02),
        })


def _on_ir_temp(sender, data: bytearray) -> None:
    if len(data) < 4:
        return
    raw_obj, raw_amb = struct.unpack('<hh', data[:4])
    t_obj = (raw_obj >> 2) * 0.03125
    t_amb = (raw_amb >> 2) * 0.03125
    _send_json({"type": "ir_temp", "ir_temp": t_obj, "amb_temp": t_amb})


def _on_humidity(sender, data: bytearray) -> None:
    if len(data) < 4:
        return
    raw_t, raw_h = struct.unpack('<HH', data[:4])
    temp = -40.0 + 165.0 * raw_t / 65536.0
    hum  = 100.0 * raw_h / 65536.0
    _send_json({"type": "humidity", "temp": temp, "humidity": hum})


def _on_baro(sender, data: bytearray) -> None:
    if len(data) < 6:
        return
    tL, tM, tH, pL, pM, pH = struct.unpack('<BBBBBB', data[:6])
    temp  = (tH * 65536 + tM * 256 + tL) / 100.0
    press = (pH * 65536 + pM * 256 + pL) / 100.0
    _send_json({"type": "baro", "temp": temp, "pressure": press})


def _on_light(sender, data: bytearray) -> None:
    if len(data) < 2:
        return
    raw = struct.unpack('<H', data[:2])[0]
    m = raw & 0x0FFF
    e = (raw & 0xF000) >> 12
    lux = 0.01 * (m << e)
    _send_json({"type": "light", "lux": lux})


# ==========================================
# TCP SERVER
# ==========================================
def _tcp_server_thread() -> None:
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((RELAY_HOST, RELAY_PORT))
    srv.listen(5)
    print(f"[RELAY] TCP relay listening on port {RELAY_PORT}.")
    while True:
        try:
            conn, addr = srv.accept()
            print(f"[RELAY] Client connected from {addr}")
            with _clients_lock:
                _clients.append(conn)
        except OSError:
            break


# ==========================================
# BLE WORKER
# ==========================================
async def ble_worker() -> None:
    global _rssi
    print("[BLE] Scanning for TI SensorTag (CC2650)...")

    target = None

    def _detection_cb(device, adv_data):
        nonlocal target
        if target is None and device.name and "SensorTag" in device.name:
            target = device
            _rssi = adv_data.rssi if adv_data.rssi is not None else 0

    async with BleakScanner(detection_callback=_detection_cb):
        await asyncio.sleep(5.0)

    if target is None:
        devices = await BleakScanner.discover(timeout=3.0)
        for d in devices:
            if d.name and "SensorTag" in d.name:
                target = d
                break

    if target is None:
        print("[BLE] SensorTag not found. Make sure it is on and blinking green.")
        return

    print(f"[BLE] Found: {target.name} [{target.address}]  RSSI: {_rssi} dBm")

    print(f"[BLE] Connecting to {target.address}...")
    async with BleakClient(target.address) as client:
        await asyncio.sleep(0.5)  # Allow GATT service discovery to settle

        # Battery — read first (always works), notify separately (may not be supported)
        try:
            raw = await client.read_gatt_char(BATTERY_LEVEL_UUID)
            _on_battery(None, raw)
        except Exception as exc:
            print(f"[BLE] Battery read failed: {exc}")
        try:
            await client.start_notify(BATTERY_LEVEL_UUID, _on_battery)
        except Exception:
            pass  # Battery Level notify not supported on this firmware — read-only is fine

        # Buttons
        try:
            await client.start_notify(IO_DATA_UUID, _on_button)
        except Exception as exc:
            print(f"[BLE] Button subscribe failed: {exc}")

        # IR Temperature
        try:
            await client.write_gatt_char(IR_TEMP_CONFIG_UUID, b'\x01', response=True)
            await client.start_notify(IR_TEMP_DATA_UUID, _on_ir_temp)
        except Exception as exc:
            print(f"[BLE] IR Temp failed: {exc}")

        # Humidity
        try:
            await client.write_gatt_char(HUMIDITY_CONFIG_UUID, b'\x01', response=True)
            await client.start_notify(HUMIDITY_DATA_UUID, _on_humidity)
        except Exception as exc:
            print(f"[BLE] Humidity failed: {exc}")

        # Barometer
        try:
            await client.write_gatt_char(BARO_CONFIG_UUID, b'\x01', response=True)
            await client.start_notify(BARO_DATA_UUID, _on_baro)
        except Exception as exc:
            print(f"[BLE] Barometer failed: {exc}")

        # Light
        try:
            await client.write_gatt_char(LIGHT_CONFIG_UUID, b'\x01', response=True)
            await client.start_notify(LIGHT_DATA_UUID, _on_light)
        except Exception as exc:
            print(f"[BLE] Light failed: {exc}")

        # Movement: subscribe first, then configure sensor
        await client.start_notify(MOVEMENT_DATA_UUID, _on_movement)
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


# ==========================================
# ENTRY POINT
# ==========================================
if __name__ == "__main__":
    tcp_thread = threading.Thread(target=_tcp_server_thread, daemon=True)
    tcp_thread.start()

    try:
        asyncio.run(ble_worker())
    except KeyboardInterrupt:
        print("\n[RELAY] Stopped.")
