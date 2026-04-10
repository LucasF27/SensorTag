# TI SensorTag CC2650 Python Router

A high-performance Python backend for the Texas Instruments **SensorTag CC2650**. It connects locally over Bluetooth Low Energy (BLE) and offers three configurable output routes for the MPU-9250 movement data:
1. **PyQtGraph Plotting Interface**: A native GUI widget rendering 60FPS fluid graphs for the 6-axis Gyroscope & Accelerometer.
2. **Godot/Game Engine Integration (UDP)**: Fires raw, normalized JSON packets continuously to a local UDP port (`127.0.0.1:4242`).
3. **System-wide Virtual Gamepad Emulator (vgamepad)**: Masks the SensorTag as a virtual Xbox 360 controller in Windows, directly mapping Pitch/Roll telemetry to joystick movements and physical SensorTag button presses to (A/B) controller buttons.

## Prerequisites
* Python 3.8+ (Windows environments inherently utilize `bleak` for WinRT Bluetooth connections).
* `pip install -r requirements.txt` (Instals `bleak`, `PyQt5`, `pyqtgraph`, `matplotlib`, and `vgamepad`).

### Notes on Virtual Gamepad 
The gamepad emulation option relies on `vgamepad`, which internally interfaces with the [ViGEmBus](https://github.com/nefarius/ViGEmBus) driver. Ensure this driver is available/installed on your Windows machine if you choose to enable controller emulation.

## Usage
Simply edit `sensortag_plot.py` and flip the respective booleans at the top of the file:
```python
ENABLE_PLOT = False       # Set to True to spawn PyQt Graph window
ENABLE_UDP = False        # Set to True to push JSON telemetry to UDP
ENABLE_GAMEPAD = True     # Set to True to spawn virtual Xbox 360 Controller
```

If `ENABLE_PLOT` is deactivated, the program will spin up cleanly in **Headless Background Mode** as an agile terminal daemon.

Run the main file:
```pwsh
python sensortag_plot.py
```

### Simple Keys Integration
By default, the SensorTag's physical hardware buttons are bound to the emulator:
- **Left Button (1):** Gamepad `A`
- **Right Button (2):** Gamepad `B`
