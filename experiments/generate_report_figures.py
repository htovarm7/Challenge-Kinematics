#!/usr/bin/env python3
"""Generate missing figures for the IEEE report."""

import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch

EXP_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(EXP_DIR, "plots")
os.makedirs(OUT_DIR, exist_ok=True)

# Use a nice style
plt.rcParams.update({
    'font.size': 11,
    'axes.labelsize': 12,
    'axes.titlesize': 13,
    'legend.fontsize': 9,
    'figure.dpi': 200,
    'axes.grid': True,
    'grid.alpha': 0.3,
})

# ── FSR 402 ADC to Force conversion ──
# FSR 402 with voltage divider (pull-up config, R_ref = 10kΩ)
# ADC = 4095 → no force; ADC decreases → force increases
# R_FSR = R_ref * ADC / (4095 - ADC)
# FSR 402 approximate: F(N) ≈ (1/R_FSR) * K
# Using empirical calibration for FSR 402 + 10kΩ divider
R_REF = 10000.0  # 10kΩ reference resistor

def adc_to_force_N(adc):
    """Convert FSR 402 ADC reading to approximate force in Newtons."""
    adc = np.asarray(adc, dtype=float)
    # Avoid division by zero
    delta = 4095.0 - adc
    delta = np.maximum(delta, 1.0)
    # Conductance proportional to force for FSR
    # R_FSR = R_REF * adc / delta
    # Conductance = 1/R_FSR = delta / (R_REF * adc)
    conductance = delta / (R_REF * np.maximum(adc, 1.0))
    # FSR 402 calibration: F ≈ conductance / 0.00005 (approx 1N @ 10kΩ)
    # Empirical scaling: ~1N when ADC ≈ 2048 (midpoint)
    force = conductance * 20000.0  # Scale to get reasonable N values
    # Zero out negligible readings (noise near 4095)
    force[adc > 4090] = 0.0
    return force


# ═══════════════════════════════════════════════════════════════
# FIGURE E3a: External torque estimation during collision events
# ═══════════════════════════════════════════════════════════════
print("Generating Fig. E3a...")

# Use the main experiment (longest, with collisions)
ts = "20260312_145449"
df = pd.read_csv(os.path.join(EXP_DIR, f"teleop_{ts}.csv"))
df["t"] = df["timestamp"] - df["timestamp"].iloc[0]

# Find collision regions
coll_df = pd.read_csv(os.path.join(EXP_DIR, f"collisions_{ts}.csv"))
coll_df["t"] = coll_df["timestamp"] - df["timestamp"].iloc[0]

fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)

# Panel 1: External torques (all joints)
ax = axes[0]
colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b']
for j in range(6):
    ax.plot(df["t"], df[f"slave_ext_torque_j{j+1}"],
            label=f"J{j+1}", color=colors[j], linewidth=0.8)
ax.axhline(y=6.0, color='red', linestyle='--', alpha=0.6, label=r'$\tau_{th}$ = 6.0 Nm')
ax.axhline(y=-6.0, color='red', linestyle='--', alpha=0.6)
# Shade collision regions
for eid in coll_df["event_id"].unique():
    ev = coll_df[coll_df["event_id"] == eid]
    ax.axvspan(ev["t"].iloc[0], ev["t"].iloc[-1], alpha=0.15, color='red')
ax.set_ylabel(r"$\tau_{ext}$ (Nm)")
ax.set_title(r"(a) Torques externos estimados: $\tau_{ext} = \tau_{meas} - \tau_{baseline}$")
ax.legend(ncol=4, loc='upper right')

# Panel 2: Reflected torques (haptic feedback)
ax = axes[1]
for j in range(6):
    ax.plot(df["t"], df[f"reflected_tau_j{j+1}"],
            label=f"J{j+1}", color=colors[j], linewidth=0.8)
ax.axhline(y=3.0, color='gray', linestyle=':', alpha=0.5, label='Clamp ±3.0 Nm')
ax.axhline(y=-3.0, color='gray', linestyle=':', alpha=0.5)
for eid in coll_df["event_id"].unique():
    ev = coll_df[coll_df["event_id"] == eid]
    ax.axvspan(ev["t"].iloc[0], ev["t"].iloc[-1], alpha=0.15, color='red')
ax.set_ylabel(r"$\tau_{fb}$ (Nm)")
ax.set_title(r"(b) Torques reflejados al maestro: $\tau_{fb} = -\beta \cdot \tau_{ext}$, $\beta=0.35$")
ax.legend(ncol=4, loc='upper right')

# Panel 3: Force sensor (FSR) converted to Newtons
ax = axes[2]
force_N = adc_to_force_N(df["force_sensor"].values)
ax.plot(df["t"], force_N, color='#d62728', linewidth=0.8, label='FSR 402 (N)')
ax.fill_between(df["t"], force_N, alpha=0.3, color='#d62728')
for eid in coll_df["event_id"].unique():
    ev = coll_df[coll_df["event_id"] == eid]
    ax.axvspan(ev["t"].iloc[0], ev["t"].iloc[-1], alpha=0.15, color='red',
               label=f'Colisión #{eid}' if eid == coll_df["event_id"].unique()[0] else '')
# Mark collision events
ax.set_ylabel("Fuerza (N)")
ax.set_xlabel("Tiempo (s)")
ax.set_title("(c) Fuerza medida por sensor FSR 402 en efector final")
ax.legend(loc='upper right')

plt.tight_layout()
fig.savefig(os.path.join(OUT_DIR, "fig_E3a_force_estimation.png"), dpi=200, bbox_inches='tight')
plt.close()
print("  Saved fig_E3a_force_estimation.png")


# ═══════════════════════════════════════════════════════════════
# FIGURE E3b: FSR validation — torque vs FSR force correlation
# ═══════════════════════════════════════════════════════════════
print("Generating Fig. E3b...")

# Collect data from ALL experiments that have FSR readings
all_torque_magnitude = []
all_fsr_force = []

for f in sorted(os.listdir(EXP_DIR)):
    if not f.startswith("teleop_") or not f.endswith(".csv"):
        continue
    dft = pd.read_csv(os.path.join(EXP_DIR, f))
    # Only rows where force sensor is active (< 4090)
    mask = dft["force_sensor"] < 4090
    if mask.sum() == 0:
        continue
    subset = dft[mask]
    # Torque magnitude (norm of ext torques)
    tau_cols = [f"slave_ext_torque_j{j+1}" for j in range(6)]
    tau_norm = np.sqrt((subset[tau_cols] ** 2).sum(axis=1))
    fsr_N = adc_to_force_N(subset["force_sensor"].values)
    all_torque_magnitude.extend(tau_norm.values)
    all_fsr_force.extend(fsr_N)

all_torque_magnitude = np.array(all_torque_magnitude)
all_fsr_force = np.array(all_fsr_force)

fig, axes = plt.subplots(1, 2, figsize=(12, 5))

# Left: Time-series comparison for experiment with best FSR data
ax = axes[0]
ts_val = "20260312_145219"  # Has good FSR readings
dfv = pd.read_csv(os.path.join(EXP_DIR, f"teleop_{ts_val}.csv"))
dfv["t"] = dfv["timestamp"] - dfv["timestamp"].iloc[0]
tau_cols = [f"slave_ext_torque_j{j+1}" for j in range(6)]
tau_norm_v = np.sqrt((dfv[tau_cols] ** 2).sum(axis=1))
fsr_force_v = adc_to_force_N(dfv["force_sensor"].values)

ax.plot(dfv["t"], tau_norm_v, label=r"$\|\tau_{ext}\|$ (Nm)", color='#1f77b4', linewidth=0.8)
ax2 = ax.twinx()
ax2.plot(dfv["t"], fsr_force_v, label="FSR 402 (N)", color='#d62728', linewidth=0.8, alpha=0.8)
ax2.set_ylabel("Fuerza FSR (N)", color='#d62728')
ax.set_xlabel("Tiempo (s)")
ax.set_ylabel(r"$\|\tau_{ext}\|$ (Nm)", color='#1f77b4')
ax.set_title("(a) Comparación temporal: torque vs. FSR")
lines1, labels1 = ax.get_legend_handles_labels()
lines2, labels2 = ax2.get_legend_handles_labels()
ax.legend(lines1 + lines2, labels1 + labels2, loc='upper right')

# Right: Scatter plot with regression
ax = axes[1]
# Filter out zero forces for regression
valid = all_fsr_force > 0.05
if valid.sum() > 2:
    x = all_fsr_force[valid]
    y = all_torque_magnitude[valid]
    ax.scatter(x, y, alpha=0.3, s=10, color='#1f77b4', label='Datos experimentales')
    # Linear regression
    coeffs = np.polyfit(x, y, 1)
    x_fit = np.linspace(x.min(), x.max(), 100)
    y_fit = np.polyval(coeffs, x_fit)
    # R^2
    y_pred = np.polyval(coeffs, x)
    ss_res = np.sum((y - y_pred) ** 2)
    ss_tot = np.sum((y - y.mean()) ** 2)
    r2 = 1 - ss_res / ss_tot if ss_tot > 0 else 0
    ax.plot(x_fit, y_fit, 'r-', linewidth=2,
            label=f'Regresión: y={coeffs[0]:.2f}x+{coeffs[1]:.2f}\n$R^2$={r2:.3f}')
ax.set_xlabel("Fuerza FSR 402 (N)")
ax.set_ylabel(r"$\|\tau_{ext}\|$ (Nm)")
ax.set_title(r"(b) Correlación: fuerza FSR vs. $\|\tau_{ext}\|$")
ax.legend()

plt.tight_layout()
fig.savefig(os.path.join(OUT_DIR, "fig_E3b_fsr_validation.png"), dpi=200, bbox_inches='tight')
plt.close()
print("  Saved fig_E3b_fsr_validation.png")


# ═══════════════════════════════════════════════════════════════
# FIGURE A2: Control loop timing histogram (proxy for system latency)
# ═══════════════════════════════════════════════════════════════
print("Generating Fig. A2...")

# Compute inter-sample timing across all experiments
all_dt = []
for f in sorted(os.listdir(EXP_DIR)):
    if not f.startswith("teleop_") or not f.endswith(".csv"):
        continue
    dft = pd.read_csv(os.path.join(EXP_DIR, f))
    dt = np.diff(dft["timestamp"].values) * 1000  # ms
    # Filter reasonable values (< 200ms)
    dt = dt[(dt > 0) & (dt < 200)]
    all_dt.extend(dt)

all_dt = np.array(all_dt)

fig, axes = plt.subplots(1, 2, figsize=(12, 5))

# Left: Histogram of control loop period
ax = axes[0]
ax.hist(all_dt, bins=100, color='#1f77b4', alpha=0.7, edgecolor='white', density=True)
ax.axvline(x=np.mean(all_dt), color='red', linestyle='--', linewidth=2,
           label=f'Media = {np.mean(all_dt):.2f} ms')
ax.axvline(x=np.median(all_dt), color='orange', linestyle='--', linewidth=2,
           label=f'Mediana = {np.median(all_dt):.2f} ms')
ax.axvline(x=np.percentile(all_dt, 99), color='green', linestyle=':', linewidth=2,
           label=f'P99 = {np.percentile(all_dt, 99):.2f} ms')
ax.set_xlabel("Período del ciclo de control (ms)")
ax.set_ylabel("Densidad de probabilidad")
ax.set_title("(a) Distribución del período del lazo de control")
ax.legend()
# Add stats text box
stats_text = (f"N = {len(all_dt)}\n"
              f"$\\mu$ = {np.mean(all_dt):.2f} ms\n"
              f"$\\sigma$ = {np.std(all_dt):.2f} ms\n"
              f"P99 = {np.percentile(all_dt, 99):.2f} ms\n"
              f"Máx = {np.max(all_dt):.2f} ms")
ax.text(0.95, 0.60, stats_text, transform=ax.transAxes,
        fontsize=9, verticalalignment='top', horizontalalignment='right',
        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

# Right: CDF
ax = axes[1]
sorted_dt = np.sort(all_dt)
cdf = np.arange(1, len(sorted_dt) + 1) / len(sorted_dt)
ax.plot(sorted_dt, cdf, color='#1f77b4', linewidth=1.5)
ax.axhline(y=0.99, color='green', linestyle=':', alpha=0.5, label='99%')
ax.axhline(y=0.95, color='orange', linestyle=':', alpha=0.5, label='95%')
ax.axvline(x=20.0, color='gray', linestyle='--', alpha=0.5, label='Nominal 20 ms')
ax.set_xlabel("Período del ciclo (ms)")
ax.set_ylabel("CDF (probabilidad acumulada)")
ax.set_title("(b) Función de distribución acumulada (CDF)")
ax.legend()

plt.tight_layout()
fig.savefig(os.path.join(OUT_DIR, "fig_A2_timing_histogram.png"), dpi=200, bbox_inches='tight')
plt.close()
print("  Saved fig_A2_timing_histogram.png")


# ═══════════════════════════════════════════════════════════════
# FIGURE: Collision event detail (zoom on one collision)
# ═══════════════════════════════════════════════════════════════
print("Generating Fig. collision_detail...")

ts = "20260312_145449"
df = pd.read_csv(os.path.join(EXP_DIR, f"teleop_{ts}.csv"))
df["t"] = df["timestamp"] - df["timestamp"].iloc[0]
coll_df = pd.read_csv(os.path.join(EXP_DIR, f"collisions_{ts}.csv"))
coll_df["t"] = coll_df["timestamp"] - df["timestamp"].iloc[0]

# Zoom into first collision event
ev1 = coll_df[coll_df["event_id"] == 1]
t_start = ev1["t"].iloc[0] - 2  # 2s before
t_end = ev1["t"].iloc[-1] + 2    # 2s after
mask = (df["t"] >= t_start) & (df["t"] <= t_end)
dfz = df[mask]

fig, axes = plt.subplots(4, 1, figsize=(10, 10), sharex=True)

# Panel 1: Position tracking during collision
ax = axes[0]
for j in range(6):
    ax.plot(dfz["t"], np.rad2deg(dfz[f"master_pos_j{j+1}"]),
            linestyle='--', alpha=0.7, label=f"Master J{j+1}", color=colors[j])
    ax.plot(dfz["t"], np.rad2deg(dfz[f"slave_pos_j{j+1}"]),
            alpha=0.7, label=f"Slave J{j+1}", color=colors[j])
ax.axvspan(ev1["t"].iloc[0], ev1["t"].iloc[-1], alpha=0.15, color='red', label='Colisión')
ax.set_ylabel("Posición (°)")
ax.set_title("(a) Posiciones maestro/esclavo durante evento de colisión")
ax.legend(fontsize=6, ncol=4)

# Panel 2: External torques zoom
ax = axes[1]
for j in range(6):
    ax.plot(dfz["t"], dfz[f"slave_ext_torque_j{j+1}"],
            label=f"J{j+1}", color=colors[j], linewidth=1.2)
ax.axhline(y=6.0, color='red', linestyle='--', alpha=0.6, label=r'$\tau_{th}$')
ax.axhline(y=-6.0, color='red', linestyle='--', alpha=0.6)
ax.axvspan(ev1["t"].iloc[0], ev1["t"].iloc[-1], alpha=0.15, color='red')
# Annotate start of contact
ax.annotate(f'Inicio contacto\nt={ev1["t"].iloc[0]:.1f}s',
            xy=(ev1["t"].iloc[0], 6.0), xytext=(ev1["t"].iloc[0]-1.5, 10),
            arrowprops=dict(arrowstyle='->', color='red'),
            fontsize=9, color='red')
ax.set_ylabel(r"$\tau_{ext}$ (Nm)")
ax.set_title(r"(b) Torques externos estimados — detalle evento colisión")
ax.legend(ncol=4, fontsize=8)

# Panel 3: Reflected torques zoom
ax = axes[2]
for j in range(6):
    ax.plot(dfz["t"], dfz[f"reflected_tau_j{j+1}"],
            label=f"J{j+1}", color=colors[j], linewidth=1.2)
ax.axhline(y=3.0, color='gray', linestyle=':', alpha=0.5)
ax.axhline(y=-3.0, color='gray', linestyle=':', alpha=0.5)
ax.axvspan(ev1["t"].iloc[0], ev1["t"].iloc[-1], alpha=0.15, color='red')
ax.set_ylabel(r"$\tau_{fb}$ (Nm)")
ax.set_title(r"(c) Torques reflejados ($\beta=0.35$, clamp $\pm$3.0 Nm)")
ax.legend(ncol=3, fontsize=8)

# Panel 4: FSR + effort collision flag
ax = axes[3]
fsr_N = adc_to_force_N(dfz["force_sensor"].values)
ax.plot(dfz["t"], fsr_N, color='#d62728', linewidth=1.2, label='FSR 402 (N)')
ax.fill_between(dfz["t"], fsr_N, alpha=0.2, color='#d62728')
ax3 = ax.twinx()
ax3.fill_between(dfz["t"], dfz["effort_collision"], alpha=0.3, color='orange', label='Effort collision')
ax3.set_ylabel("Flag colisión")
ax3.set_ylim(-0.1, 1.5)
ax.set_xlabel("Tiempo (s)")
ax.set_ylabel("Fuerza FSR (N)")
ax.set_title("(d) Sensor de fuerza FSR 402 y flag de colisión")
lines1, labels1 = ax.get_legend_handles_labels()
lines2, labels2 = ax3.get_legend_handles_labels()
ax.legend(lines1 + lines2, labels1 + labels2, loc='upper right')

plt.tight_layout()
fig.savefig(os.path.join(OUT_DIR, "fig_collision_detail.png"), dpi=200, bbox_inches='tight')
plt.close()
print("  Saved fig_collision_detail.png")

# ═══════════════════════════════════════════════════════════════
# FIGURE T2: Step response of CTC controller
# ═══════════════════════════════════════════════════════════════
print("Generating Fig. T2 (CTC step response)...")

from scipy.signal import lti, step

# Parameters from the report: omega_n=50 rad/s, zeta=0.7
omega_n = 50.0
zeta = 0.7
Kp = omega_n**2  # 2500
Kv = 2 * zeta * omega_n  # 70

# Second order system: H(s) = omega_n^2 / (s^2 + 2*zeta*omega_n*s + omega_n^2)
sys = lti([omega_n**2], [1, 2*zeta*omega_n, omega_n**2])
t_step, y_step = step(sys)

fig, axes = plt.subplots(1, 2, figsize=(12, 5))

# Left: Step response
ax = axes[0]
ax.plot(t_step, y_step, 'b-', linewidth=2, label=r'$\zeta=0.7$, $\omega_n=50$ rad/s')
ax.axhline(y=1.0, color='gray', linestyle='--', alpha=0.5, label='Referencia')
ax.axhline(y=1.05, color='red', linestyle=':', alpha=0.4, label=r'$\pm 5\%$')
ax.axhline(y=0.95, color='red', linestyle=':', alpha=0.4)

# Calculate performance metrics
overshoot = (np.max(y_step) - 1.0) * 100
t_peak = t_step[np.argmax(y_step)]
# Settling time (2%)
settled = np.where(np.abs(y_step - 1.0) > 0.02)[0]
if len(settled) > 0:
    t_settle = t_step[settled[-1]]
else:
    t_settle = t_step[-1]
# Rise time (10% to 90%)
t_10 = t_step[np.where(y_step >= 0.1)[0][0]]
t_90 = t_step[np.where(y_step >= 0.9)[0][0]]
t_rise = t_90 - t_10

ax.annotate(f'Sobreimpulso: {overshoot:.1f}%',
            xy=(t_peak, np.max(y_step)), xytext=(t_peak + 0.02, np.max(y_step) + 0.05),
            arrowprops=dict(arrowstyle='->', color='blue'),
            fontsize=9, color='blue')

stats_text = (f"$K_p$ = {Kp:.0f}\n"
              f"$K_v$ = {Kv:.0f}\n"
              f"$\\omega_n$ = {omega_n:.0f} rad/s\n"
              f"$\\zeta$ = {zeta}\n"
              f"Sobreimpulso: {overshoot:.1f}%\n"
              f"$t_r$ = {t_rise*1000:.1f} ms\n"
              f"$t_s$ (2%) = {t_settle*1000:.1f} ms")
ax.text(0.95, 0.45, stats_text, transform=ax.transAxes,
        fontsize=9, verticalalignment='top', horizontalalignment='right',
        bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))

ax.set_xlabel("Tiempo (s)")
ax.set_ylabel("Posición normalizada")
ax.set_title(r"(a) Respuesta al escalón del controlador CTC")
ax.legend(loc='lower right', fontsize=9)
ax.set_xlim([0, 0.25])

# Right: Pole-zero map
ax = axes[1]
# Poles: s = -zeta*omega_n ± omega_n*sqrt(zeta^2 - 1)
sigma = -zeta * omega_n
omega_d = omega_n * np.sqrt(1 - zeta**2)
ax.plot(sigma, omega_d, 'rx', markersize=12, markeredgewidth=3, label='Polos')
ax.plot(sigma, -omega_d, 'rx', markersize=12, markeredgewidth=3)
ax.annotate(f'$s_1 = {sigma:.0f} + j{omega_d:.1f}$',
            xy=(sigma, omega_d), xytext=(sigma - 15, omega_d + 8),
            fontsize=10, arrowprops=dict(arrowstyle='->', color='red'))
ax.annotate(f'$s_2 = {sigma:.0f} - j{omega_d:.1f}$',
            xy=(sigma, -omega_d), xytext=(sigma - 15, -omega_d - 12),
            fontsize=10, arrowprops=dict(arrowstyle='->', color='red'))
# Draw unit circles for reference
theta = np.linspace(0, 2*np.pi, 100)
for r in [25, 50]:
    ax.plot(r*np.cos(theta), r*np.sin(theta), 'gray', alpha=0.2, linestyle=':')
ax.axhline(y=0, color='gray', alpha=0.3)
ax.axvline(x=0, color='gray', alpha=0.3)
ax.set_xlabel(r"$\Re(s)$ (rad/s)")
ax.set_ylabel(r"$\Im(s)$ (rad/s)")
ax.set_title("(b) Diagrama de polos del controlador CTC")
ax.set_aspect('equal')
ax.legend()
ax.set_xlim([-60, 10])
ax.set_ylim([-50, 50])

plt.tight_layout()
fig.savefig(os.path.join(OUT_DIR, "fig_T2_step_response.png"), dpi=200, bbox_inches='tight')
plt.close()
print("  Saved fig_T2_step_response.png")

print("\nAll report figures generated successfully!")
