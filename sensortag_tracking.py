"""
sensortag_tracking.py

3D orientation + position tracking tab for the SensorTag dashboard.

Orientation: live rotating XYZ triad driven by the Madgwick quaternion.
Position:    dead-reckoning by rotating accel to world frame, subtracting
             gravity, and double-integrating.  Drift is expected — use
             Reset Position or Zero Velocity Update to correct.
"""

import time
import numpy as np

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QCheckBox,
)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont
import pyqtgraph.opengl as gl
from pyqtgraph.opengl import GLViewWidget


# ==========================================
# CONSTANTS
# ==========================================
GRAVITY_MS2      = 9.80665          # m/s²
TRAIL_LENGTH     = 300              # position trail history points
STILL_THRESHOLD  = 0.04            # deg/s — below this → sensor is still
STILL_FRAMES     = 10              # consecutive still frames for ZUPT


# ==========================================
# HELPER — quaternion rotation
# ==========================================

def _quat_rotate(q_wxyz, v):
    """Rotate vector v (3,) by unit quaternion q [w,x,y,z] -> world vector."""
    w, x, y, z = q_wxyz
    qv = np.array([x, y, z])
    return v + 2.0 * w * np.cross(qv, v) + 2.0 * np.cross(qv, np.cross(qv, v))


# ==========================================
# TRACKING TAB WIDGET
# ==========================================

class TrackingTab(QWidget):
    """
    Drop-in QWidget tab.  Call feed(pkt) for each movement packet where
    pkt = {'accel': [...], 'gyro': [...], 'orientation': {...}, 'ts': float}
    """

    def __init__(self, parent=None):
        super().__init__(parent)

        # ── state ─────────────────────────────────────────────────────────────
        self._pos      = np.zeros(3)     # metres
        self._vel      = np.zeros(3)     # m/s
        self._last_ts  = None
        self._trail    = []              # list of np.array(3)
        self._still_count = 0

        # Gravity estimate — will settle once orientation converges
        self._gravity_world = np.array([0.0, 0.0, GRAVITY_MS2])

        # ── layout ────────────────────────────────────────────────────────────
        root = QVBoxLayout(self)
        root.setContentsMargins(4, 4, 4, 4)
        root.setSpacing(4)

        # Control bar
        ctrl = QHBoxLayout()

        self._btn_reset = QPushButton("Reset Position")
        self._btn_reset.setFixedHeight(30)
        self._btn_reset.clicked.connect(self._reset_position)

        self._btn_zupt = QPushButton("Zero Velocity (ZUPT)")
        self._btn_zupt.setFixedHeight(30)
        self._btn_zupt.clicked.connect(self._apply_zupt)

        self._chk_auto_zupt = QCheckBox("Auto-ZUPT when still")
        self._chk_auto_zupt.setChecked(True)

        self._lbl_pos = QLabel("Pos:  0.000  0.000  0.000 m")
        f = QFont(); f.setPointSize(10); f.setFamily("Courier")
        self._lbl_pos.setFont(f)

        self._lbl_warn = QLabel("⚠ Dead-reckoning drifts. Use ZUPT / Reset often.")
        self._lbl_warn.setStyleSheet("color: orange;")

        ctrl.addWidget(self._btn_reset)
        ctrl.addWidget(self._btn_zupt)
        ctrl.addWidget(self._chk_auto_zupt)
        ctrl.addStretch()
        ctrl.addWidget(self._lbl_pos)
        root.addLayout(ctrl)
        root.addWidget(self._lbl_warn)

        # ── GL view ───────────────────────────────────────────────────────────
        self._view = GLViewWidget()
        self._view.setMinimumHeight(400)
        self._view.setCameraPosition(distance=2.0, elevation=20, azimuth=45)
        root.addWidget(self._view)

        # Grid
        grid = gl.GLGridItem()
        grid.setSize(4, 4)
        grid.setSpacing(0.5, 0.5)
        self._view.addItem(grid)

        # Origin marker (white dot)
        origin_dot = gl.GLScatterPlotItem(
            pos=np.array([[0, 0, 0]]),
            color=np.array([[1, 1, 1, 1]]),
            size=8,
        )
        self._view.addItem(origin_dot)

        # Axis triad — X=red, Y=green, Z=blue (drawn as line pairs from origin)
        self._triad = self._make_triad()
        for item in self._triad:
            self._view.addItem(item)

        # Position trail
        self._trail_item = gl.GLLinePlotItem(
            pos=np.zeros((2, 3)),
            color=(0.3, 0.8, 1.0, 0.8),
            width=2,
            antialias=True,
        )
        self._view.addItem(self._trail_item)

        # Current position dot
        self._pos_dot = gl.GLScatterPlotItem(
            pos=np.array([[0, 0, 0]]),
            color=np.array([[0.3, 0.8, 1.0, 1.0]]),
            size=12,
        )
        self._view.addItem(self._pos_dot)

    # ── triad construction ────────────────────────────────────────────────────

    @staticmethod
    def _make_triad():
        """Return 3 GLLinePlotItems as [x_item, y_item, z_item]."""
        axes = [
            (np.array([[0,0,0],[1,0,0]]), (1, 0, 0, 1)),   # X red
            (np.array([[0,0,0],[0,1,0]]), (0, 1, 0, 1)),   # Y green
            (np.array([[0,0,0],[0,0,1]]), (0, 0.5, 1, 1)), # Z blue
        ]
        items = []
        for pts, color in axes:
            item = gl.GLLinePlotItem(
                pos=pts.astype(np.float32),
                color=color,
                width=4,
                antialias=True,
            )
            items.append(item)
        return items

    # ── public API ────────────────────────────────────────────────────────────

    def feed(self, pkt: dict):
        """Called from the UI thread for every movement packet."""
        ori = pkt.get('orientation')
        if ori is None:
            return

        q = ori.get('quaternion')
        if q is None:
            return

        q = np.array(q, dtype=float)
        self._update_triad(q)
        self._integrate_position(pkt, q)

    # ── orientation triad ─────────────────────────────────────────────────────

    def _update_triad(self, q_wxyz):
        """Rotate the XYZ triad to match current quaternion."""
        dirs = [
            np.array([1, 0, 0]),  # X
            np.array([0, 1, 0]),  # Y
            np.array([0, 0, 1]),  # Z
        ]
        for item, d in zip(self._triad, dirs):
            tip = _quat_rotate(q_wxyz, d)
            pts = np.array([[0, 0, 0], tip], dtype=np.float32)
            item.setData(pos=pts)

    # ── position integration ─────────────────────────────────────────────────

    def _integrate_position(self, pkt: dict, q_wxyz):
        ts    = pkt['ts']
        accel = np.array(pkt['accel'], dtype=float) * GRAVITY_MS2  # G → m/s²
        gyro  = np.array(pkt['gyro'],  dtype=float)

        if self._last_ts is None:
            self._last_ts = ts
            return
        dt = ts - self._last_ts
        self._last_ts = ts
        if dt <= 0 or dt > 0.5:
            return

        # Rotate accel to world frame and remove gravity
        accel_world = _quat_rotate(q_wxyz, accel)
        accel_lin   = accel_world + self._gravity_world

        # Zero-out very small accelerations (deadband: 0.05 m/s²)
        accel_lin[np.abs(accel_lin) < 0.05] = 0.0

        # Auto-ZUPT: if gyro magnitude is tiny, sensor is stationary
        gyro_mag = float(np.linalg.norm(gyro))
        if self._chk_auto_zupt.isChecked():
            if gyro_mag < STILL_THRESHOLD:
                self._still_count += 1
            else:
                self._still_count = 0
            if self._still_count >= STILL_FRAMES:
                self._vel[:] = 0.0

        # Integrate
        self._vel += accel_lin * dt
        self._pos += self._vel * dt

        # Update trail
        self._trail.append(self._pos.copy())
        if len(self._trail) > TRAIL_LENGTH:
            self._trail.pop(0)

        # Update GL items
        p = self._pos
        self._lbl_pos.setText(
            f"Pos:  {p[0]:+.3f}  {p[1]:+.3f}  {p[2]:+.3f} m"
        )
        self._pos_dot.setData(pos=np.array([p]))

        if len(self._trail) >= 2:
            trail_arr = np.array(self._trail, dtype=np.float32)
            self._trail_item.setData(pos=trail_arr)

    # ── button handlers ───────────────────────────────────────────────────────

    def _reset_position(self):
        self._pos[:]  = 0.0
        self._vel[:]  = 0.0
        self._trail   = []
        self._pos_dot.setData(pos=np.array([[0, 0, 0]]))
        self._trail_item.setData(pos=np.zeros((2, 3)))
        self._lbl_pos.setText("Pos:  0.000  0.000  0.000 m")

    def _apply_zupt(self):
        self._vel[:] = 0.0
