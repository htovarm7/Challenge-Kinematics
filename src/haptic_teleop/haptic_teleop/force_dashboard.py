#!/usr/bin/env python3
"""
haptic_teleop/force_dashboard.py
=================================
Dashboard en tiempo real para visualizar fuerzas, esfuerzos y estado
del sistema de teleoperación bilateral.

Autor : Jose Luis Dominguez – ITESM Robótica
Curso  : TE3001B – Kinematics Challenge

Uso:
    ros2 run haptic_teleop force_dashboard
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import JointState
from std_msgs.msg import Int32, Float64MultiArray, Bool

import numpy as np
from collections import deque
import threading
import time

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import FancyBboxPatch
import matplotlib.gridspec as gridspec

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
N_JOINTS = 6
HISTORY = 300  # 6 seconds at 50 Hz


class ForceDashboard(Node):

    def __init__(self):
        super().__init__("force_dashboard")

        qos_be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5)
        qos_rel = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=10)

        # ── Data buffers ──────────────────────────────────
        self._lock = threading.Lock()
        self._t0 = time.time()

        self._time_buf = deque(maxlen=HISTORY)
        self._effort_buf = [deque(maxlen=HISTORY) for _ in range(N_JOINTS)]
        self._reflected_buf = [deque(maxlen=HISTORY) for _ in range(N_JOINTS)]
        self._qerr_buf = [deque(maxlen=HISTORY) for _ in range(N_JOINTS)]
        self._force_buf = deque(maxlen=HISTORY)
        self._force_time = deque(maxlen=HISTORY)

        self._tau_slave = np.zeros(N_JOINTS)
        self._tau_baseline = np.zeros(N_JOINTS)
        self._baseline_ok = False
        self._baseline_samples = []

        self._q_master = np.zeros(N_JOINTS)
        self._q_slave = np.zeros(N_JOINTS)
        self._reflected = np.zeros(N_JOINTS)
        self._force_raw = 4095
        self._collision_active = False
        self._effort_collision = False

        # ── Calibration: collect baseline for 2 seconds ───
        self._calib_countdown = 100  # ~2s at 50Hz

        # ── Subscriptions ─────────────────────────────────
        self.create_subscription(
            JointState, "/joint_states", self._cb_slave, qos_be)
        self.create_subscription(
            JointState, "/master/joint_states", self._cb_master, qos_be)
        self.create_subscription(
            Float64MultiArray, "/master/reflected_torques",
            self._cb_reflected, qos_rel)
        self.create_subscription(
            Int32, "/force_sensor", self._cb_force, qos_be)
        self.create_subscription(
            Bool, "/slave/collision_active", self._cb_collision, qos_rel)

        self.get_logger().info("[Dashboard] Force dashboard started")

    # ── Callbacks ──────────────────────────────────────────
    def _cb_slave(self, msg: JointState):
        name_map = dict(zip(msg.name, range(len(msg.name))))
        try:
            indices = [name_map[j] for j in JOINT_NAMES]
        except KeyError:
            return

        pos = np.array([msg.position[i] for i in indices])
        eff = np.zeros(N_JOINTS)
        if len(msg.effort) >= N_JOINTS:
            eff = np.array([msg.effort[i] for i in indices])

        with self._lock:
            self._q_slave = pos
            self._tau_slave = eff

            # Auto-calibrate baseline
            if not self._baseline_ok:
                self._baseline_samples.append(eff.copy())
                self._calib_countdown -= 1
                if self._calib_countdown <= 0 and len(self._baseline_samples) >= 50:
                    arr = np.array(self._baseline_samples[-80:])
                    self._tau_baseline = arr.mean(axis=0)
                    self._baseline_ok = True
                    self.get_logger().info(
                        f"[Dashboard] Baseline calibrated: {np.round(self._tau_baseline, 2)}")
                return

            t = time.time() - self._t0
            tau_ext = eff - self._tau_baseline
            self._time_buf.append(t)
            for i in range(N_JOINTS):
                self._effort_buf[i].append(tau_ext[i])

            q_err = np.rad2deg(self._q_master + self._compute_offset() - pos)
            for i in range(N_JOINTS):
                self._qerr_buf[i].append(q_err[i])

    def _compute_offset(self):
        """Simple offset — assumes both start near HOME."""
        # Use the same approach as bilateral_teleop
        return self._q_slave * 0  # offset applied externally; show raw error

    def _cb_master(self, msg: JointState):
        name_map = dict(zip(msg.name, range(len(msg.name))))
        try:
            indices = [name_map[j] for j in JOINT_NAMES]
            with self._lock:
                self._q_master = np.array([msg.position[i] for i in indices])
        except KeyError:
            pass

    def _cb_reflected(self, msg: Float64MultiArray):
        if len(msg.data) < N_JOINTS:
            return
        with self._lock:
            self._reflected = np.array(msg.data[:N_JOINTS])
            t = time.time() - self._t0
            for i in range(N_JOINTS):
                self._reflected_buf[i].append(msg.data[i])
            # Pad if reflected comes at different rate
            while len(self._reflected_buf[0]) < len(self._time_buf):
                for i in range(N_JOINTS):
                    self._reflected_buf[i].appendleft(0.0)

    def _cb_force(self, msg: Int32):
        with self._lock:
            self._force_raw = msg.data
            self._force_buf.append(msg.data)
            self._force_time.append(time.time() - self._t0)

    def _cb_collision(self, msg: Bool):
        with self._lock:
            self._collision_active = msg.data


def main(args=None):
    rclpy.init(args=args)
    node = ForceDashboard()

    # Spin ROS in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # ── Matplotlib setup ──────────────────────────────────
    plt.style.use("dark_background")
    fig = plt.figure(figsize=(14, 9))
    fig.canvas.manager.set_window_title("Haptic Teleop — Force Dashboard")

    gs = gridspec.GridSpec(3, 2, hspace=0.35, wspace=0.25,
                           left=0.07, right=0.97, top=0.93, bottom=0.06)

    # Subplot 1: External torques (tau_ext per joint)
    ax_effort = fig.add_subplot(gs[0, :])
    ax_effort.set_title("Torque Externo Estimado (τ_ext = τ_slave − τ_baseline)", fontsize=11)
    ax_effort.set_ylabel("N·m")
    ax_effort.set_xlim(0, HISTORY / 50.0)
    ax_effort.set_ylim(-10, 10)
    ax_effort.axhline(6.0, color="red", ls="--", alpha=0.4, label="Umbral colisión")
    ax_effort.axhline(-6.0, color="red", ls="--", alpha=0.4)
    colors = plt.cm.Set1(np.linspace(0, 1, N_JOINTS))
    effort_lines = []
    for i in range(N_JOINTS):
        ln, = ax_effort.plot([], [], color=colors[i], lw=1.2,
                             label=f"J{i+1}")
        effort_lines.append(ln)
    ax_effort.legend(loc="upper left", fontsize=8, ncol=6)
    ax_effort.grid(alpha=0.2)

    # Subplot 2: Reflected torques to master
    ax_refl = fig.add_subplot(gs[1, 0])
    ax_refl.set_title("Torque Reflejado al Maestro (háptico)", fontsize=10)
    ax_refl.set_ylabel("N·m")
    ax_refl.set_ylim(-4, 4)
    ax_refl.set_xlim(0, HISTORY / 50.0)
    refl_lines = []
    for i in range(N_JOINTS):
        ln, = ax_refl.plot([], [], color=colors[i], lw=1.0)
        refl_lines.append(ln)
    ax_refl.grid(alpha=0.2)

    # Subplot 3: Force sensor (FSR)
    ax_fsr = fig.add_subplot(gs[1, 1])
    ax_fsr.set_title("Sensor de Fuerza (FSR — ADC)", fontsize=10)
    ax_fsr.set_ylabel("ADC")
    ax_fsr.set_ylim(3500, 4200)
    ax_fsr.set_xlim(0, HISTORY / 50.0)
    ax_fsr.axhline(3900, color="orange", ls="--", alpha=0.6, label="Umbral contacto")
    fsr_line, = ax_fsr.plot([], [], color="#00ff88", lw=1.5)
    ax_fsr.legend(loc="lower left", fontsize=8)
    ax_fsr.grid(alpha=0.2)

    # Subplot 4: Position tracking error
    ax_err = fig.add_subplot(gs[2, 0])
    ax_err.set_title("Error de Seguimiento (maestro − esclavo)", fontsize=10)
    ax_err.set_ylabel("grados")
    ax_err.set_xlabel("tiempo (s)")
    ax_err.set_ylim(-15, 15)
    ax_err.set_xlim(0, HISTORY / 50.0)
    err_lines = []
    for i in range(N_JOINTS):
        ln, = ax_err.plot([], [], color=colors[i], lw=1.0)
        err_lines.append(ln)
    ax_err.grid(alpha=0.2)

    # Subplot 5: Status panel
    ax_status = fig.add_subplot(gs[2, 1])
    ax_status.set_xlim(0, 1)
    ax_status.set_ylim(0, 1)
    ax_status.axis("off")
    ax_status.set_title("Estado del Sistema", fontsize=10)

    status_texts = {
        "state": ax_status.text(0.05, 0.85, "Estado: ---", fontsize=12,
                                fontweight="bold", color="white",
                                transform=ax_status.transAxes),
        "fsr": ax_status.text(0.05, 0.68, "FSR: ---", fontsize=11,
                              color="#00ff88", transform=ax_status.transAxes),
        "collision": ax_status.text(0.05, 0.51, "Colisión: ---", fontsize=11,
                                    color="white", transform=ax_status.transAxes),
        "tau_max": ax_status.text(0.05, 0.34, "|τ_ext|_max: ---", fontsize=11,
                                  color="white", transform=ax_status.transAxes),
        "refl_max": ax_status.text(0.05, 0.17, "|τ_refl|_max: ---", fontsize=11,
                                   color="white", transform=ax_status.transAxes),
        "baseline": ax_status.text(0.05, 0.02, "Baseline: calibrando...",
                                   fontsize=10, color="yellow",
                                   transform=ax_status.transAxes),
    }

    # ── Animation update ──────────────────────────────────
    def update(_frame):
        with node._lock:
            t_arr = np.array(node._time_buf) if node._time_buf else np.array([0])
            n = len(t_arr)

            if n > 1:
                t_min = t_arr[-1] - HISTORY / 50.0
                t_max = t_arr[-1]
                for ax in [ax_effort, ax_refl, ax_err]:
                    ax.set_xlim(t_min, t_max)

            for i in range(N_JOINTS):
                eff_arr = np.array(node._effort_buf[i]) if node._effort_buf[i] else np.array([0])
                effort_lines[i].set_data(t_arr[:len(eff_arr)], eff_arr)

                refl_arr = np.array(node._reflected_buf[i]) if node._reflected_buf[i] else np.array([0])
                m = min(len(t_arr), len(refl_arr))
                refl_lines[i].set_data(t_arr[:m], refl_arr[:m])

                err_arr = np.array(node._qerr_buf[i]) if node._qerr_buf[i] else np.array([0])
                err_lines[i].set_data(t_arr[:len(err_arr)], err_arr)

            # FSR
            ft = np.array(node._force_time) if node._force_time else np.array([0])
            fv = np.array(node._force_buf) if node._force_buf else np.array([4095])
            fsr_line.set_data(ft, fv)
            if len(ft) > 1:
                ax_fsr.set_xlim(ft[-1] - HISTORY / 50.0, ft[-1])

            # Status panel
            fsr_val = node._force_raw
            contact = fsr_val < 3900
            collision = node._collision_active

            tau_ext = node._tau_slave - node._tau_baseline if node._baseline_ok else np.zeros(N_JOINTS)
            tau_max = np.max(np.abs(tau_ext))
            refl_max = np.max(np.abs(node._reflected))

            if not node._baseline_ok:
                state_str = "CALIBRANDO"
                state_color = "yellow"
            elif collision or tau_max > 6.0:
                state_str = "COLISIÓN"
                state_color = "#ff4444"
            elif contact:
                state_str = "CONTACTO FSR"
                state_color = "orange"
            else:
                state_str = "NORMAL"
                state_color = "#00ff88"

            status_texts["state"].set_text(f"Estado: {state_str}")
            status_texts["state"].set_color(state_color)
            status_texts["fsr"].set_text(f"FSR: {fsr_val}  {'⚡ CONTACTO' if contact else '✓'}")
            status_texts["fsr"].set_color("orange" if contact else "#00ff88")
            status_texts["collision"].set_text(
                f"Colisión esfuerzo: {'⚠ ACTIVA' if collision or tau_max > 6 else 'No'}")
            status_texts["collision"].set_color("#ff4444" if collision or tau_max > 6 else "white")
            status_texts["tau_max"].set_text(f"|τ_ext|_max: {tau_max:.2f} N·m")
            status_texts["tau_max"].set_color("#ff4444" if tau_max > 6 else "white")
            status_texts["refl_max"].set_text(f"|τ_refl|_max: {refl_max:.2f} N·m")
            status_texts["baseline"].set_text(
                f"Baseline: {'✓ OK' if node._baseline_ok else 'calibrando...'}")
            status_texts["baseline"].set_color("#00ff88" if node._baseline_ok else "yellow")

        return (effort_lines + refl_lines + err_lines + [fsr_line]
                + list(status_texts.values()))

    ani = FuncAnimation(fig, update, interval=80, blit=False, cache_frame_data=False)
    plt.show()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
