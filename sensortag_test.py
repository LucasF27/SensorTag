"""
sensortag_test.py — Movement/IMU sensor diagnostic for TI SensorTag CC2650

Scans for nearby SensorTags, lets you pick one, then collects MPU-9250
(gyro + accel + mag) notifications for a configurable duration and prints
a plain-text summary including MAC address and average sample frequency.

Usage:
    python sensortag_test.py
    python sensortag_test.py --duration 20
"""

import argparse
import asyncio
import statistics
import struct
import time

from bleak import BleakClient, BleakScanner
from bleak.exc import BleakError

# ---------------------------------------------------------------------------
# GATT UUIDs — TI CC2650 SensorTag movement service
# ---------------------------------------------------------------------------
MOVEMENT_DATA_UUID   = "f000aa81-0451-4000-b000-000000000000"
MOVEMENT_CONFIG_UUID = "f000aa82-0451-4000-b000-000000000000"
MOVEMENT_PERIOD_UUID = "f000aa83-0451-4000-b000-000000000000"

# Thresholds for PASS/FAIL
MIN_SAMPLES   = 50
MIN_FREQ_HZ   = 5.0


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def _decode_movement(data: bytearray):
    """Parse the 18-byte MPU-9250 notification into (gyro, accel, mag) tuples."""
    if len(data) != 18:
        return None
    v = struct.unpack('<hhhhhhhhh', data)
    gyro  = (v[0] / (65536.0 / 500.0),
             v[1] / (65536.0 / 500.0),
             v[2] / (65536.0 / 500.0))
    accel = (v[3] / (32768.0 / 8.0),
             v[4] / (32768.0 / 8.0),
             v[5] / (32768.0 / 8.0))
    mag   = (float(v[6]), float(v[7]), float(v[8]))
    return gyro, accel, mag


def _axis_stats(samples, index):
    """Return (min, max, mean) for one axis across all samples."""
    vals = [s[index] for s in samples]
    return min(vals), max(vals), statistics.mean(vals)


def _print_axis_row(label, values):
    mn, mx, avg = values
    print(f"    {label:<20s}  min={mn:+9.4f}   max={mx:+9.4f}   avg={avg:+9.4f}")


def _hr(char="─", width=62):
    print(char * width)


# ---------------------------------------------------------------------------
# Discovery
# ---------------------------------------------------------------------------
async def discover_sensortags(scan_duration: float = 5.0):
    """Return list of (name, address, rssi) for every found SensorTag."""
    found = {}

    def _cb(device, adv):
        if device.name and "SensorTag" in device.name:
            rssi = adv.rssi if adv.rssi is not None else -999
            found[device.address] = (device.name, device.address, rssi)

    print(f"[SCAN] Scanning for SensorTags ({scan_duration:.0f} s)...")
    async with BleakScanner(detection_callback=_cb):
        await asyncio.sleep(scan_duration)

    return list(found.values())


# ---------------------------------------------------------------------------
# Main test
# ---------------------------------------------------------------------------
async def run_test(address: str, duration: float):
    samples = []          # list of (ts, gyro, accel, mag)
    disconnected = False  # set True if device drops connection unexpectedly

    def _on_disconnect(client):
        nonlocal disconnected
        disconnected = True
        print("[BLE]  Device disconnected unexpectedly.")

    def _on_movement(sender, data: bytearray):
        decoded = _decode_movement(data)
        if decoded is not None:
            samples.append((time.time(), *decoded))

    print(f"[BLE]  Connecting to {address} ...")
    async with BleakClient(address, disconnected_callback=_on_disconnect) as client:
        if not client.is_connected:
            print("[ERROR] Failed to connect.")
            return

        print("[BLE]  Connected. Enabling MPU-9250 movement sensor...")

        # Set 100 ms period (0x0A × 10 ms = 100 ms → ~10 Hz)
        await client.write_gatt_char(MOVEMENT_PERIOD_UUID, b"\x0A")
        # Enable all axes + magnetometer, 8G accel range (0x7F02)
        await client.write_gatt_char(MOVEMENT_CONFIG_UUID, b"\x7F\x02")
        await client.start_notify(MOVEMENT_DATA_UUID, _on_movement)

        print(f"[TEST] Collecting samples for {duration:.0f} s  ...")
        await asyncio.sleep(duration)

        try:
            await client.stop_notify(MOVEMENT_DATA_UUID)
        except (OSError, BleakError):
            pass  # device already disconnected — cleanup not possible
        try:
            await client.write_gatt_char(MOVEMENT_CONFIG_UUID, b"\x00\x00")
        except (OSError, BleakError):
            pass

    # -----------------------------------------------------------------------
    # Analysis
    # -----------------------------------------------------------------------
    n = len(samples)

    _hr("═")
    print("  SENSORTAG MOVEMENT SENSOR TEST RESULTS")
    _hr("═")
    print(f"  MAC Address      : {address}")
    print(f"  Test duration    : {duration:.0f} s")
    print(f"  Samples received : {n}")
    print(f"  Unexpected drop  : {'YES — device disconnected during test' if disconnected else 'No'}")

    if n < 2:
        print("  Average frequency: N/A (insufficient samples)")
        _hr()
        print("  RESULT  ►  FAIL  (no data received)")
        _hr("═")
        return

    ts_all = [s[0] for s in samples]
    span   = ts_all[-1] - ts_all[0]
    freq   = (n - 1) / span if span > 0 else 0.0

    deltas = [ts_all[i + 1] - ts_all[i] for i in range(len(ts_all) - 1)]
    jitter = statistics.stdev(deltas) * 1000 if len(deltas) > 1 else 0.0

    print(f"  Average frequency: {freq:.2f} Hz")
    print(f"  Sample jitter    : {jitter:.2f} ms (stdev of Δt)")
    _hr()

    gyros  = [s[1] for s in samples]
    accels = [s[2] for s in samples]
    mags   = [s[3] for s in samples]

    print("  GYROSCOPE  (deg/s)")
    _print_axis_row("X", _axis_stats(gyros,  0))
    _print_axis_row("Y", _axis_stats(gyros,  1))
    _print_axis_row("Z", _axis_stats(gyros,  2))
    _hr()
    print("  ACCELEROMETER  (G)")
    _print_axis_row("X", _axis_stats(accels, 0))
    _print_axis_row("Y", _axis_stats(accels, 1))
    _print_axis_row("Z", _axis_stats(accels, 2))
    _hr()
    print("  MAGNETOMETER  (raw counts)")
    _print_axis_row("X", _axis_stats(mags,   0))
    _print_axis_row("Y", _axis_stats(mags,   1))
    _print_axis_row("Z", _axis_stats(mags,   2))
    _hr()

    mag_dead = all(
        s[3][i] == 0.0 for s in samples for i in range(3)
    )

    passed = n >= MIN_SAMPLES and freq >= MIN_FREQ_HZ and not disconnected and not mag_dead
    verdict = "PASS" if passed else "FAIL"
    reasons = []
    if n < MIN_SAMPLES:
        reasons.append(f"only {n} samples (need ≥ {MIN_SAMPLES})")
    if freq < MIN_FREQ_HZ:
        reasons.append(f"frequency {freq:.2f} Hz (need ≥ {MIN_FREQ_HZ:.0f} Hz)")
    if disconnected:
        reasons.append("device dropped BLE connection during test")
    if mag_dead:
        reasons.append("magnetometer reads all zeros (hardware fault)")
    reason_str = "  (" + "; ".join(reasons) + ")" if reasons else ""

    print(f"  RESULT  ►  {verdict}{reason_str}")
    _hr("═")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
async def main():
    parser = argparse.ArgumentParser(description="TI SensorTag CC2650 movement sensor test")
    parser.add_argument("--duration", type=float, default=10.0,
                        help="Collection duration in seconds (default: 10)")
    args = parser.parse_args()

    devices = await discover_sensortags()

    if not devices:
        print("[ERROR] No SensorTag devices found. Make sure the sensor is powered on and nearby.")
        return

    if len(devices) == 1:
        name, address, rssi = devices[0]
        print(f"[SCAN] Found: {name}  [{address}]  RSSI={rssi} dBm")
        chosen = address
    else:
        print(f"[SCAN] Found {len(devices)} SensorTag device(s):\n")
        for i, (name, address, rssi) in enumerate(devices):
            print(f"  [{i + 1}]  {name}  [{address}]  RSSI={rssi} dBm")
        print()
        while True:
            raw = input(f"Select device [1–{len(devices)}]: ").strip()
            if raw.isdigit() and 1 <= int(raw) <= len(devices):
                chosen = devices[int(raw) - 1][1]
                break
            print("      Invalid selection, try again.")

    await run_test(chosen, args.duration)


if __name__ == "__main__":
    asyncio.run(main())
