"""Helper: writes Step 1 scaffold to sensortag_plot.py. Delete after use."""
import pathlib

content = '''\
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
'''

out = pathlib.Path(__file__).parent / "sensortag_plot.py"
out.write_text(content, encoding="utf-8")
print(f"Written {out}  ({len(content.splitlines())} lines)")
